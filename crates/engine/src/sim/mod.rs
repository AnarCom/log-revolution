//! Event-driven simulation core (PLAN.md §3). A min-heap of (time, gate)
//! events; each `step()` pops every event sharing the current minimum
//! time and evaluates that whole batch.
//!
//! Why that batch is safe to evaluate in parallel: every gate that can
//! actually be *downstream* of another gate (And/Or/Not) has propagation
//! delay >= 1 tick (see `Gate::delay`), and subcircuit boundaries are wire-
//! elided at compile time (`netlist.rs`), not simulated as zero-delay
//! components. So two events land on the same timestamp only when nothing
//! in that batch feeds anything else in it — by construction, not by
//! coincidence. `Component::eval` only reads pin values committed by the
//! *previous* batch, so parallel evaluation can't observe a half-updated
//! neighbor.
//!
//! `priming` (this module's submodule) computes the initial-priming
//! schedule described in `Simulation::new`'s doc comment; the exhaustive
//! test suite lives in `tests` (its own file, since it's easily half this
//! module's actual content).

mod priming;
#[cfg(test)]
mod tests;

use crate::netlist::{Netlist, PinRef};
use plugin_abi::{Bit, Component, Signal};
use rayon::prelude::*;
use std::collections::{BinaryHeap, HashSet};

pub type Time = u64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Event {
    time: Time,
    gate: usize,
}

impl Ord for Event {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Reversed: BinaryHeap is a max-heap, we want the *smallest* time out.
        other.time.cmp(&self.time).then_with(|| other.gate.cmp(&self.gate))
    }
}
impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

pub struct Simulation {
    netlist: Netlist,
    /// Committed output values: [gate][output pin].
    outputs: Vec<Vec<Signal>>,
    time: Time,
    queue: BinaryHeap<Event>,
    /// The *one* shared clock-tick counter for the whole simulation
    /// (`Propagator.ticks` in `logisim-port`) — every `Gate::Clock`
    /// instance reads this same counter, not its own timer; see the
    /// `Gate::Clock` doc comment. Distinct from `time`: `time` is
    /// propagation-delay units within a single settle, `global_ticks` only
    /// advances on an explicit `tick()` call.
    global_ticks: u64,
    /// Precomputed once so `tick()` doesn't have to scan `netlist.gates`
    /// (which can include the same subcircuit's clocks duplicated once per
    /// instance, same as real Logisim walking every `CircuitState`).
    clock_gates: Vec<usize>,
}

impl Simulation {
    pub fn new(netlist: Netlist) -> Self {
        // Unknown, not Zero: nothing has actually been computed yet, and
        // claiming Zero would be asserting a value no gate produced.
        let outputs = netlist
            .gates
            .iter()
            .map(|g| vec![vec![Bit::Unknown]; g.output_count()])
            .collect();
        let clock_gates = netlist
            .gates
            .iter()
            .enumerate()
            .filter(|(_, g)| matches!(g, crate::components::Gate::Clock { .. }))
            .map(|(idx, _)| idx)
            .collect();
        let priming_depth = priming::priming_depths(&netlist);
        let mut sim = Simulation {
            netlist,
            outputs,
            time: 0,
            queue: BinaryHeap::new(),
            global_ticks: 0,
            clock_gates,
        };
        // Prime every gate once so sources (InputPin, Constant, Clock, ...)
        // and everything downstream settles before anything reads it — but
        // staggered by dependency depth (`priming_depth`), not all at the
        // same instant `t=0`. A stateless/combinational gate would self-
        // correct either way (a stale first read just gets superseded by a
        // later, correct one, same as any ordinary propagation ripple —
        // `run_to_quiescence` doesn't stop until the queue is empty). A
        // *stateful* gate doesn't get that do-over: `Register`'s
        // `last_clock` is written unconditionally on every `eval`, so if
        // its first-ever read of `ck` landed on a not-yet-committed
        // placeholder (`Unknown`, because the real upstream `Clock` hadn't
        // had its own turn yet in the same simultaneous batch) that
        // `Unknown` permanently overwrites the true initial history —
        // there is no later batch that goes back and fixes it, because
        // nothing re-derives history from scratch the way combinational
        // `eval` re-derives outputs from current inputs. Staggering by
        // depth guarantees every gate's *first* eval only ever reads
        // already-committed (real, not placeholder) values from whatever
        // feeds it directly.
        for (idx, &depth) in priming_depth.iter().enumerate() {
            sim.queue.push(Event { time: depth, gate: idx });
        }
        sim
    }

    pub fn time(&self) -> Time {
        self.time
    }

    pub fn global_ticks(&self) -> u64 {
        self.global_ticks
    }

    /// Advances the shared clock counter by one and updates every
    /// `Gate::Clock` in the design (mirrors `Simulator.doTick`'s
    /// `propagator.tick()` — a single counter increment, applied to every
    /// clock in the whole hierarchy at once, not per-instance timers).
    /// Only marks the changed ones dirty; propagating that change through
    /// the rest of the circuit is `step()`/`run_to_quiescence()`'s job, same
    /// as any other external stimulus (`invoke`) — real Logisim's `tick()`
    /// likewise only invalidates, a separate `propagate()` loop settles.
    pub fn tick(&mut self) -> Time {
        self.global_ticks += 1;
        let t = self.global_ticks;
        for idx in self.clock_gates.clone() {
            if self.netlist.gates[idx].tick(t) {
                self.queue.push(Event { time: self.time, gate: idx });
            }
        }
        self.time
    }

    /// One pin's value, one bit lane at a time: `combine` every *real*
    /// driver connected to that exact lane (PLAN.md §9 — reproducing
    /// tri-state/short-circuit behavior, not just "take the one source");
    /// zero real drivers folds to `Unknown`, two disagreeing ones fold to
    /// `Error`. `PullResistor` drivers are kept out of that fold and
    /// applied afterward, only if the real-driver result is `Unknown` — a
    /// lone well-defined driver or an actual conflict both override the
    /// pull untouched. Mirrors `CircuitWires.getThreadValue`/`pullValue` in
    /// `logisim-port` exactly, not a simplification of it.
    ///
    /// `netlist.input_sources[gate][pin]` is pre-sized to that pin's true
    /// declared width (`Netlist::input_sources`'s doc) — so a lane with
    /// zero registered drivers still gets its own `Unknown` entry in the
    /// assembled `Signal`, at the right position, rather than shortening
    /// it. Each lane's drivers are explicit `BitRef`s now (not an implicit
    /// "bit `i` of every whole-pin source"), because a `Splitter` can wire
    /// two lanes of the very same destination pin to entirely different,
    /// differently-widthed source pins.
    fn gather_inputs(&self, gate: usize) -> Vec<Signal> {
        self.netlist.input_sources[gate]
            .iter()
            .map(|per_bit| {
                per_bit
                    .iter()
                    .map(|sources| {
                        let mut real = Bit::Unknown;
                        let mut pull = Bit::Unknown;
                        for &(g, p, b) in sources {
                            let v = self.outputs[g][p].get(b as usize).copied().unwrap_or(Bit::Unknown);
                            let target = if matches!(self.netlist.gates[g], crate::components::Gate::PullResistor { .. }) {
                                &mut pull
                            } else {
                                &mut real
                            };
                            *target = target.combine(v);
                        }
                        if real == Bit::Unknown {
                            pull
                        } else {
                            real
                        }
                    })
                    .collect()
            })
            .collect()
    }

    /// Drives a top-level `InputPin`/`OutputPin`-style gate's action
    /// (`on`/`off`/`toggle`/`set`) and schedules it for re-evaluation.
    /// Errors from `invoke` propagate — see PLAN.md §5/§7 (this is what a
    /// `.ctest` `component[id] -> action` statement will eventually call).
    pub fn invoke(
        &mut self,
        gate: usize,
        action: &str,
        arg: Option<plugin_abi::Value>,
    ) -> Result<(), plugin_abi::ActionError> {
        self.netlist.gates[gate].invoke(action, arg)?;
        self.queue.push(Event {
            time: self.time,
            gate,
        });
        Ok(())
    }

    pub fn read(&self, gate: usize, name: &str) -> Result<plugin_abi::Value, plugin_abi::ReadoutError> {
        self.netlist.gates[gate].read(name)
    }

    /// Evaluates one batch — every event at the current minimum time.
    /// Returns `false` if the queue was empty (nothing to do).
    pub fn step(&mut self) -> bool {
        let Some(&Event { time, .. }) = self.queue.peek() else {
            return false;
        };
        self.time = time;

        let mut batch = HashSet::new();
        while let Some(&ev) = self.queue.peek() {
            if ev.time != time {
                break;
            }
            self.queue.pop();
            batch.insert(ev.gate);
        }

        let gathered: Vec<(usize, Vec<Signal>)> = batch
            .iter()
            .map(|&idx| (idx, self.gather_inputs(idx)))
            .collect();
        let gathered: std::collections::HashMap<usize, Vec<Signal>> = gathered.into_iter().collect();

        let results: Vec<(usize, Vec<Signal>)> = self
            .netlist
            .gates
            .par_iter_mut()
            .enumerate()
            .filter_map(|(idx, gate)| gathered.get(&idx).map(|ins| (idx, gate.eval(ins))))
            .collect();

        for (idx, new_outputs) in results {
            if new_outputs != self.outputs[idx] {
                self.outputs[idx] = new_outputs;
                let targets: Vec<usize> = self.netlist.fanout[idx]
                    .iter()
                    .flatten()
                    .flatten()
                    .map(|&(dst_gate, _dst_pin, _dst_bit)| dst_gate)
                    .collect();
                for dst_gate in targets {
                    self.schedule(dst_gate);
                }
            }
        }
        true
    }

    fn schedule(&mut self, gate: usize) {
        // `.max(1)` is load-bearing, not just a floor: it's what guarantees
        // a gate scheduled *this* round can never land in the batch that's
        // scheduling it, which is the whole reason same-timestamp batches
        // are safe to evaluate in parallel (module doc above). Don't drop
        // it even for a gate kind whose own `delay()` happens to be 0.
        let delay = self.netlist.gates[gate].delay();
        self.queue.push(Event {
            time: self.time + delay.max(1),
            gate,
        });
    }

    /// Runs until the event queue drains — "settle" for a combinational
    /// (or already-clocked) circuit. `.ctest`'s bare `simulate` (PLAN.md
    /// §7) maps to this.
    pub fn run_to_quiescence(&mut self) -> Time {
        while self.step() {}
        self.time
    }

    pub fn output_of(&self, pin: PinRef) -> Signal {
        self.outputs[pin.0][pin.1].clone()
    }
}
