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
        let priming_depth = priming_depths(&netlist);
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
        for idx in 0..sim.netlist.gates.len() {
            sim.queue.push(Event { time: priming_depth[idx], gate: idx });
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

    /// One pin's value: `combine` every *real* driver connected to it
    /// (PLAN.md §9 — reproducing tri-state/short-circuit behavior, not just
    /// "take the one source"); zero real drivers folds to `Unknown`, two
    /// disagreeing ones fold to `Error`. `PullResistor` drivers are kept
    /// out of that fold and applied afterward, only if the real-driver
    /// result is `Unknown` — a lone well-defined driver or an actual
    /// conflict both override the pull untouched. Mirrors
    /// `CircuitWires.getThreadValue`/`pullValue` in `logisim-port`
    /// exactly, not a simplification of it.
    ///
    /// Per-bit, not per-pin: a pin can be a multi-bit bus (up to 32 —
    /// `Value.MAX_WIDTH`), and `combine` is applied independently at each
    /// bit position (a driver narrower than the widest one on the same
    /// point reads as `Unknown` on the missing high bits, same as
    /// `Component::eval`'s own `bit_at`/`Signal::get` convention).
    fn gather_inputs(&self, gate: usize) -> Vec<Signal> {
        self.netlist.input_sources[gate]
            .iter()
            .map(|sources| {
                let width = sources.iter().map(|&(g, p)| self.outputs[g][p].len()).max().unwrap_or(1);
                let mut real = vec![Bit::Unknown; width];
                let mut pull = vec![Bit::Unknown; width];
                for &(g, p) in sources {
                    let signal = &self.outputs[g][p];
                    let target = if matches!(self.netlist.gates[g], crate::components::Gate::PullResistor { .. }) {
                        &mut pull
                    } else {
                        &mut real
                    };
                    for i in 0..width {
                        let v = signal.get(i).copied().unwrap_or(Bit::Unknown);
                        target[i] = target[i].combine(v);
                    }
                }
                (0..width).map(|i| if real[i] == Bit::Unknown { pull[i] } else { real[i] }).collect()
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
                    .map(|&(dst_gate, _dst_pin)| dst_gate)
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

/// Each gate's distance (in hops) from the nearest gate with no real
/// drivers on any input pin — i.e. a source, or something floating enough
/// to be treated as one. Used only to stagger `Simulation::new`'s initial
/// priming events (see its doc comment for why staggering matters, not
/// just simultaneous `t=0` for everyone).
///
/// A fixed-point relaxation, not a single topological pass, because
/// Logisim circuits can genuinely contain feedback loops (cross-coupled
/// gates, oscillators) where no acyclic topological order exists at all —
/// gates that never resolve through the relaxation (because they're part
/// of, or depend only on, such a cycle) fall back to depth `0`, the same
/// treatment as an actual source. That's a reasonable fallback, not a
/// hack: a cycle has no well-defined "settled initial value" to begin
/// with, so there's nothing a smarter depth assignment could preserve for
/// it anyway — same as today, it settles (or is detected oscillating)
/// through ordinary re-triggering once the simulation actually starts
/// stepping.
fn priming_depths(netlist: &Netlist) -> Vec<Time> {
    let n = netlist.gates.len();
    let direct_sources: Vec<Vec<usize>> =
        netlist.input_sources.iter().map(|pins| pins.iter().flatten().map(|&(g, _)| g).collect()).collect();

    let mut depth: Vec<Option<Time>> = vec![None; n];
    loop {
        let mut progressed = false;
        for i in 0..n {
            if depth[i].is_some() {
                continue;
            }
            if direct_sources[i].is_empty() {
                depth[i] = Some(0);
                progressed = true;
                continue;
            }
            let mut max_dep = 0;
            let mut all_resolved = true;
            for &s in &direct_sources[i] {
                match depth[s] {
                    Some(d) => max_dep = max_dep.max(d),
                    None => {
                        all_resolved = false;
                        break;
                    }
                }
            }
            if all_resolved {
                depth[i] = Some(max_dep + 1);
                progressed = true;
            }
        }
        if !progressed {
            break;
        }
    }
    depth.into_iter().map(|d| d.unwrap_or(0)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::netlist::{flatten, CircuitTemplate, TemplateNode};
    use plugin_abi::Value;
    use std::collections::HashMap;

    fn set(sim: &mut Simulation, gate: usize, on: bool) {
        sim.invoke(gate, if on { "on" } else { "off" }, None).unwrap();
    }

    fn get_bit(sim: &Simulation, gate: usize) -> Bit {
        match sim.read(gate, "get").unwrap() {
            Value::Bits(bits) => bits[0],
            other => panic!("unexpected readout {other:?}"),
        }
    }

    /// a=1, b=1 -> InputPin -> InputPin -> And -> OutputPin, straight line.
    /// Gate indices match node order 1:1 (no subcircuits in this template).
    fn and_circuit() -> CircuitTemplate {
        CircuitTemplate {
            name: "and_circuit".to_string(),
            nodes: vec![
                TemplateNode::InputPin { bits: 1 },  // 0: a
                TemplateNode::InputPin { bits: 1 },  // 1: b
                TemplateNode::And { bits: 1, inputs: 2 },       // 2
                TemplateNode::OutputPin { bits: 1 }, // 3: out
            ],
            connections: vec![((0, 0), (2, 0)), ((1, 0), (2, 1)), ((2, 0), (3, 0))],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        }
    }

    #[test]
    fn simple_and_circuit_propagates() {
        let mut library = HashMap::new();
        library.insert("main".to_string(), and_circuit());
        let netlist = flatten("main", &library).unwrap();
        let mut sim = Simulation::new(netlist);

        set(&mut sim, 0, true);
        set(&mut sim, 1, true);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::One);

        set(&mut sim, 1, false);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::Zero);
    }

    /// A reusable 2-input AND with no leaf Pin nodes of its own — its ports
    /// point straight at the And gate, proving ports don't need a
    /// materialized boundary gate (netlist.rs's wire-elision claim).
    fn and2_subcircuit() -> CircuitTemplate {
        CircuitTemplate {
            name: "and2".to_string(),
            nodes: vec![TemplateNode::And { bits: 1, inputs: 2 }], // 0
            connections: Vec::new(),
            input_ports: vec![vec![(0, 0)], vec![(0, 1)]],
            output_ports: vec![vec![(0, 0)]],
            port_marker_nodes: Vec::new(),
        }
    }

    /// Two independent instances of `and2`, fed different inputs, to prove
    /// flattening gives each instance its own gates (not shared state).
    fn two_instances_circuit() -> CircuitTemplate {
        CircuitTemplate {
            name: "main".to_string(),
            nodes: vec![
                TemplateNode::InputPin { bits: 1 },             // 0: a1
                TemplateNode::InputPin { bits: 1 },             // 1: b1
                TemplateNode::InputPin { bits: 1 },             // 2: a2
                TemplateNode::InputPin { bits: 1 },             // 3: b2
                TemplateNode::Subcircuit("and2".to_string()), // 4: inst1
                TemplateNode::Subcircuit("and2".to_string()), // 5: inst2
                TemplateNode::OutputPin { bits: 1 },            // 6: out1
                TemplateNode::OutputPin { bits: 1 },            // 7: out2
            ],
            connections: vec![
                ((0, 0), (4, 0)),
                ((1, 0), (4, 1)),
                ((4, 2), (6, 0)), // inst1's sole output port is pin index 2 (after the 2 input ports)
                ((2, 0), (5, 0)),
                ((3, 0), (5, 1)),
                ((5, 2), (7, 0)),
            ],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        }
    }

    #[test]
    fn subcircuit_instances_are_independent() {
        let mut library = HashMap::new();
        library.insert("and2".to_string(), and2_subcircuit());
        library.insert("main".to_string(), two_instances_circuit());
        let netlist = flatten("main", &library).unwrap();
        // Elaboration allocates leaves in node order, recursing into each
        // Subcircuit node as it's reached: a1,b1,a2,b2 -> 0..4, inst1's And
        // -> 4, inst2's And -> 5, out1,out2 -> 6,7.
        assert_eq!(netlist.gates.len(), 8);

        let mut sim = Simulation::new(netlist);
        set(&mut sim, 0, true); // a1
        set(&mut sim, 1, true); // b1
        set(&mut sim, 2, true); // a2
        set(&mut sim, 3, false); // b2
        sim.run_to_quiescence();

        assert_eq!(get_bit(&sim, 6), Bit::One); // out1 = a1 & b1 = 1
        assert_eq!(get_bit(&sim, 7), Bit::Zero); // out2 = a2 & b2 = 0
    }

    /// Two drivers on the same point: agreement passes through, conflict
    /// is `Error` — PLAN.md §9's short-circuit/tri-state requirement.
    fn shared_point_circuit() -> CircuitTemplate {
        CircuitTemplate {
            name: "main".to_string(),
            nodes: vec![
                TemplateNode::InputPin { bits: 1 },  // 0: a
                TemplateNode::InputPin { bits: 1 },  // 1: b
                TemplateNode::OutputPin { bits: 1 }, // 2: out — fed by both a and b
            ],
            connections: vec![((0, 0), (2, 0)), ((1, 0), (2, 0))],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        }
    }

    #[test]
    fn multiple_drivers_combine_and_detect_short_circuit() {
        let mut library = HashMap::new();
        library.insert("main".to_string(), shared_point_circuit());
        let netlist = flatten("main", &library).unwrap();
        let mut sim = Simulation::new(netlist);

        set(&mut sim, 0, true);
        set(&mut sim, 1, true);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 2), Bit::One, "agreeing drivers pass through");

        set(&mut sim, 1, false);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 2), Bit::Error, "disagreeing drivers short-circuit");
    }

    /// Two entirely disjoint AND chains in one netlist — nothing in one
    /// depends on the other, so they land in the same batch every step
    /// and get evaluated in the same `par_iter_mut` pass (sim.rs module
    /// doc). Correctness here is what "independent branches parallelize
    /// safely" actually cashes out to for a single-threaded test.
    fn two_disjoint_chains() -> CircuitTemplate {
        CircuitTemplate {
            name: "main".to_string(),
            nodes: vec![
                TemplateNode::InputPin { bits: 1 },  // 0: a1
                TemplateNode::InputPin { bits: 1 },  // 1: b1
                TemplateNode::And { bits: 1, inputs: 2 },       // 2
                TemplateNode::OutputPin { bits: 1 }, // 3: out1
                TemplateNode::InputPin { bits: 1 },  // 4: a2
                TemplateNode::InputPin { bits: 1 },  // 5: b2
                TemplateNode::And { bits: 1, inputs: 2 },       // 6
                TemplateNode::OutputPin { bits: 1 }, // 7: out2
            ],
            connections: vec![
                ((0, 0), (2, 0)),
                ((1, 0), (2, 1)),
                ((2, 0), (3, 0)),
                ((4, 0), (6, 0)),
                ((5, 0), (6, 1)),
                ((6, 0), (7, 0)),
            ],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        }
    }

    #[test]
    fn independent_branches_evaluate_correctly_in_the_same_batch() {
        let mut library = HashMap::new();
        library.insert("main".to_string(), two_disjoint_chains());
        let netlist = flatten("main", &library).unwrap();
        let mut sim = Simulation::new(netlist);

        set(&mut sim, 0, true);
        set(&mut sim, 1, false);
        set(&mut sim, 4, true);
        set(&mut sim, 5, true);
        sim.run_to_quiescence();

        assert_eq!(get_bit(&sim, 3), Bit::Zero);
        assert_eq!(get_bit(&sim, 7), Bit::One);
    }

    /// out is fed by a PullResistor(One) alone, or by an InputPin and a
    /// PullResistor(One) together — exercises `pullValue`'s three real
    /// branches (`logisim-port`'s `CircuitWires.java`).
    fn pulled_output_circuit() -> CircuitTemplate {
        CircuitTemplate {
            name: "main".to_string(),
            nodes: vec![
                TemplateNode::InputPin { bits: 1 },             // 0: driver (starts off)
                TemplateNode::PullResistor(Bit::One), // 1: pull-up
                TemplateNode::OutputPin { bits: 1 },            // 2: out — fed by both
            ],
            connections: vec![((0, 0), (2, 0)), ((1, 0), (2, 0))],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        }
    }

    #[test]
    fn pull_resistor_only_wins_when_nothing_else_drives() {
        let mut library = HashMap::new();
        library.insert("main".to_string(), pulled_output_circuit());
        let netlist = flatten("main", &library).unwrap();
        let mut sim = Simulation::new(netlist);

        // Nothing real driving (InputPin defaults to Zero via `init`, but
        // it's *connected* — so this isn't "no real driver": test that
        // case with a lone pull separately, below).
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 2), Bit::Zero, "a real driver (even 0) beats the pull");

        set(&mut sim, 0, true);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 2), Bit::One, "real driver still wins, happens to agree with pull here");
    }

    #[test]
    fn pull_resistor_fills_in_when_truly_unconnected() {
        let mut library = HashMap::new();
        // Same shape, but nothing connects to the InputPin's output — only
        // the pull resistor drives `out`.
        library.insert(
            "main".to_string(),
            CircuitTemplate {
                name: "main".to_string(),
                nodes: vec![TemplateNode::PullResistor(Bit::One), TemplateNode::OutputPin { bits: 1 }],
                connections: vec![((0, 0), (1, 0))],
                input_ports: Vec::new(),
                output_ports: Vec::new(),
                port_marker_nodes: Vec::new(),
            },
        );
        let netlist = flatten("main", &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 1), Bit::One);
    }

    #[test]
    fn pull_resistor_does_not_mask_a_real_short_circuit() {
        let mut library = HashMap::new();
        library.insert(
            "main".to_string(),
            CircuitTemplate {
                name: "main".to_string(),
                nodes: vec![
                    TemplateNode::InputPin { bits: 1 },               // 0: a
                    TemplateNode::InputPin { bits: 1 },               // 1: b
                    TemplateNode::PullResistor(Bit::One), // 2
                    TemplateNode::OutputPin { bits: 1 },              // 3: out
                ],
                connections: vec![((0, 0), (3, 0)), ((1, 0), (3, 0)), ((2, 0), (3, 0))],
                input_ports: Vec::new(),
                output_ports: Vec::new(),
                port_marker_nodes: Vec::new(),
            },
        );
        let netlist = flatten("main", &library).unwrap();
        let mut sim = Simulation::new(netlist);

        set(&mut sim, 0, true);
        set(&mut sim, 1, false);
        sim.run_to_quiescence();
        assert_eq!(
            get_bit(&sim, 3),
            Bit::Error,
            "a real conflict stays an error, the pull resistor doesn't paper over it"
        );
    }

    /// Regression test for a real bug: staggering `Simulation::new`'s
    /// initial priming by dependency depth (`priming_depths`) instead of
    /// scheduling every gate at the same instant `t=0`. Before that fix,
    /// this exact sequence — `tick()` called *before* any settle has ever
    /// happened, which is exactly what a `.ctest` script does when its
    /// first line is `simulate` (no leading bare settle) — corrupted
    /// `Register`'s `last_clock` with a stale `Unknown` placeholder read
    /// of `Clock`'s not-yet-committed output, permanently hiding the real
    /// rising edge. `clock_drives_a_register_through_simulation_tick` in
    /// `compile.rs` didn't catch this because it happened to call
    /// `run_to_quiescence()` once *before* the first `tick()`, which
    /// incidentally let the corruption self-heal (`Unknown` -> `Zero`,
    /// coincidentally the value it needed to be) before the real edge
    /// occurred — this test deliberately uses the *other* (more common,
    /// via `.ctest`) call order, where that lucky self-heal can't happen.
    fn clock_feeds_register_circuit() -> CircuitTemplate {
        CircuitTemplate {
            name: "main".to_string(),
            nodes: vec![
                TemplateNode::Constant { bits: 1, value: 1 }, // 0: D, fixed at 1
                TemplateNode::Clock { high: 1, low: 1 },      // 1: CK, period 2
                TemplateNode::Register { bits: 1, trigger: crate::components::Trigger::Rising }, // 2
                TemplateNode::OutputPin { bits: 1 },          // 3: Q
            ],
            connections: vec![((0, 0), (2, 0)), ((1, 0), (2, 1)), ((2, 0), (3, 0))],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        }
    }

    #[test]
    fn register_sees_the_first_real_clock_edge_even_without_a_leading_settle() {
        let mut library = HashMap::new();
        library.insert("main".to_string(), clock_feeds_register_circuit());
        let netlist = flatten("main", &library).unwrap();
        let mut sim = Simulation::new(netlist);

        // No `run_to_quiescence()` here on purpose — `tick()` is the very
        // first thing called, exactly like a `.ctest` script whose first
        // statement is `simulate`.
        sim.tick(); // global tick 1: high=low=1 -> period 2, 1%2=1 -> high phase -> clock rises 0->1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::One, "the rising edge must be observed, not swallowed by a stale priming read");
    }
}
