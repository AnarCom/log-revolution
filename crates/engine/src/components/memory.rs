//! Clocked/stateful components: `Clock` and `Register`. See PLAN.md's
//! "`Clock`/`Register` — один общий счётчик тактов" section for the full
//! model (one shared tick counter, not independent timers) and the real
//! bug it surfaced in `sim.rs`'s initial priming order.

use super::{bit_to_byte, byte_to_bit, signal_to_u32_if_defined, u32_to_signal, Gate};
use plugin_abi::{ActionError, Bit, ReadoutError, Signal, Value};

/// `StdAttr.TRIGGER`'s four options, sames names as the JSON attribute
/// values (`"rising"`/`"falling"`/`"high"`/`"low"`) — see `compile.rs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trigger {
    Rising,
    Falling,
    High,
    Low,
}

impl Trigger {
    /// Mirrors `ClockState.updateClock`: `old`/`new` must be *exactly*
    /// `Zero`/`One` for an edge to count — an `Unknown`/`Error` clock input
    /// never counts as "was low" or "is high".
    fn fired(self, old: Bit, new: Bit) -> bool {
        match self {
            Trigger::Rising => old == Bit::Zero && new == Bit::One,
            Trigger::Falling => old == Bit::One && new == Bit::Zero,
            Trigger::High => new == Bit::One,
            Trigger::Low => new == Bit::Zero,
        }
    }
}

impl Gate {
    pub(super) fn init_memory(&mut self) {
        match self {
            Gate::Clock { clicks, sending, .. } => {
                *clicks = 0;
                *sending = Bit::Zero; // matches `ClockState.sending`'s initial `Value.FALSE`
            }
            Gate::Register { value, last_clock, .. } => {
                *value = 0;
                *last_clock = Bit::Zero; // matches memory `ClockState`'s initial `Value.FALSE`
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn input_count_memory(&self) -> usize {
        match self {
            // d, ck, clr, en — this fixed order is what `compile.rs`'s
            // register geometry and `eval_memory` below both key off of.
            Gate::Register { .. } => 4,
            Gate::Clock { .. } => 0,
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn eval_memory(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            // Never recomputed from `inputs` — only `Gate::tick` (driven by
            // the shared tick counter) changes `sending`; `eval` just
            // re-emits the cached value, mirroring `Clock.propagate`.
            Gate::Clock { sending, .. } => vec![vec![*sending]],
            Gate::Register { bits, trigger, value, last_clock } => {
                let d = &inputs[0];
                let ck = super::bit_at(&inputs[1], 0);
                let clr = super::bit_at(&inputs[2], 0);
                let en = super::bit_at(&inputs[3], 0);

                let triggered = trigger.fired(*last_clock, ck);
                *last_clock = ck;

                if clr == Bit::One {
                    *value = 0;
                } else if triggered && en != Bit::Zero {
                    if let Some(v) = signal_to_u32_if_defined(d, *bits) {
                        *value = v;
                    }
                }

                vec![u32_to_signal(*value, *bits)]
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn serialize_memory(&self) -> Vec<u8> {
        match self {
            Gate::Clock { clicks, sending, .. } => {
                let mut out = clicks.to_le_bytes().to_vec();
                out.push(bit_to_byte(*sending));
                out
            }
            Gate::Register { value, last_clock, .. } => {
                let mut out = value.to_le_bytes().to_vec();
                out.push(bit_to_byte(*last_clock));
                out
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn deserialize_memory(&mut self, state: &[u8]) {
        match self {
            Gate::Clock { clicks, sending, .. } => {
                *clicks = state.get(0..8).and_then(|b| b.try_into().ok()).map(u64::from_le_bytes).unwrap_or(0);
                *sending = state.get(8).copied().map(byte_to_bit).unwrap_or(Bit::Zero);
            }
            Gate::Register { value, last_clock, .. } => {
                *value = state.get(0..4).and_then(|b| b.try_into().ok()).map(u32::from_le_bytes).unwrap_or(0);
                *last_clock = state.get(4).copied().map(byte_to_bit).unwrap_or(Bit::Zero);
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn actions_memory(&self) -> Vec<&'static str> {
        match self {
            // Manual Poke-toggle (`ClockPoker`) — pulses the clock once,
            // independent of `Simulation::tick`, and permanently shifts its
            // phase parity (see `Gate::Clock`'s doc comment).
            Gate::Clock { .. } => vec!["toggle"],
            // Poke-tool-style direct write, bypassing the clock/enable
            // logic entirely — useful for `.ctest` to seed a register's
            // initial contents without stepping a clock edge.
            Gate::Register { .. } => vec!["set"],
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn invoke_memory(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        match self {
            // `ClockPoker.mouseReleased`, verbatim: flip `sending` *and*
            // bump `clicks` together — the parity shift is what keeps
            // future `tick()` calls consistent with this manual flip.
            Gate::Clock { clicks, sending, .. } if name == "toggle" => {
                *sending = sending.not();
                *clicks += 1;
                Ok(())
            }
            Gate::Clock { .. } => Err(ActionError::UnknownAction(name.to_string())),
            Gate::Register { bits, value, .. } => match (name, arg) {
                ("set", Some(Value::Int(i))) => {
                    *value = (i as u64 & (u64::MAX >> (64 - *bits as u32))) as u32;
                    Ok(())
                }
                ("set", arg) => Err(ActionError::InvalidArg {
                    action: "set".to_string(),
                    reason: format!("expected Int, got {arg:?}"),
                }),
                (other, _) => Err(ActionError::UnknownAction(other.to_string())),
            },
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn read_memory(&self, name: &str) -> Result<Value, ReadoutError> {
        match self {
            Gate::Clock { sending, .. } if name == "get" => Ok(Value::Bits(vec![*sending])),
            Gate::Register { bits, value, .. } if name == "get" => Ok(Value::Bits(u32_to_signal(*value, *bits))),
            Gate::Clock { .. } | Gate::Register { .. } => Err(ReadoutError::UnknownReadout(name.to_string())),
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    /// Advances a `Clock`'s cached output to reflect the *shared* tick
    /// counter (see the `Gate::Clock` doc comment) — a no-op, returning
    /// `false`, for every other gate kind (defensively, via the early
    /// return below, rather than routed through `mod.rs`'s dispatcher —
    /// `tick` isn't a `Component` method, `Simulation` calls it on every
    /// gate index in `clock_gates`, all of which are already `Clock`, so
    /// this is belt-and-suspenders, not load-bearing). Returns whether the
    /// cached value actually changed, so `Simulation::tick` only needs to
    /// reschedule the clocks that did (though rescheduling unconditionally
    /// would also be harmless — `step()` already no-ops on an unchanged
    /// output).
    pub(super) fn tick_memory(&mut self, global_tick: u64) -> bool {
        let Gate::Clock { high, low, clicks, sending } = self else {
            return false;
        };
        let period = *high + *low;
        let mut in_low_phase = global_tick % period < *low;
        if *clicks % 2 == 1 {
            in_low_phase = !in_low_phase;
        }
        let desired = if in_low_phase { Bit::Zero } else { Bit::One };
        if *sending == desired {
            false
        } else {
            *sending = desired;
            true
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plugin_abi::Component;

    fn clock(high: u64, low: u64) -> Gate {
        Gate::Clock { high, low, clicks: 0, sending: Bit::Zero }
    }

    #[test]
    fn clock_is_a_pure_function_of_the_shared_tick_counter_not_its_own_timer() {
        // high=2, low=1: period 3, low-phase is ticks%3 < 1 i.e. tick%3==0.
        let mut c = clock(2, 1);
        // eval never recomputes anything on its own — starts at the
        // `init`/construction default (Zero) until `tick` is called.
        assert_eq!(c.eval(&[]), vec![vec![Bit::Zero]]);

        let expected = [
            (1u64, Bit::One),  // 1%3=1 -> high phase
            (2, Bit::One),     // 2%3=2 -> high phase
            (3, Bit::Zero),    // 3%3=0 -> low phase
            (4, Bit::One),
        ];
        for (t, want) in expected {
            c.tick(t);
            assert_eq!(c.eval(&[])[0][0], want, "tick {t}");
        }
    }

    #[test]
    fn clock_tick_reports_whether_the_cached_value_actually_changed() {
        let mut c = clock(1, 1); // period 2: alternates every tick
        assert!(c.tick(1)); // 1%2=1 -> not < 1 -> high phase -> One, changed from Zero
        assert!(!c.tick(1), "calling tick again with the same tick number is idempotent");
    }

    #[test]
    fn clock_manual_toggle_flips_immediately_and_shifts_future_phase() {
        let mut c = clock(1, 1);
        c.invoke("toggle", None).unwrap();
        assert_eq!(c.read("get"), Ok(Value::Bits(vec![Bit::One])), "toggle flips immediately, no tick needed");

        // Without the toggle, tick(2) would be low phase (2%2=0 < 1) -> Zero.
        // The manual flip's odd `clicks` inverts every future phase decision.
        c.tick(2);
        assert_eq!(c.read("get"), Ok(Value::Bits(vec![Bit::One])), "parity shifted by the manual click");
    }

    fn register(bits: u8, trigger: Trigger) -> Gate {
        Gate::Register { bits, trigger, value: 0, last_clock: Bit::Zero }
    }

    fn reg_inputs(d: &[Bit], ck: Bit, clr: Bit, en: Bit) -> Vec<Signal> {
        vec![d.to_vec(), vec![ck], vec![clr], vec![en]]
    }

    fn zeros(bits: u8) -> Signal {
        vec![Bit::Zero; bits as usize]
    }

    #[test]
    fn register_latches_on_rising_edge_when_enabled() {
        let mut r = register(4, Trigger::Rising);
        let d = [Bit::One, Bit::Zero, Bit::One, Bit::Zero]; // 0b0101 = 5, LSB-first

        // Clock still low: no edge yet, output stays 0.
        let out = r.eval(&reg_inputs(&d, Bit::Zero, Bit::Zero, Bit::One));
        assert_eq!(out, vec![zeros(4)]);

        // Rising edge with EN=1: latches.
        let out = r.eval(&reg_inputs(&d, Bit::One, Bit::Zero, Bit::One));
        assert_eq!(out, vec![d.to_vec()]);

        // Staying high (no new edge) with a different D: does NOT relatch.
        let d2 = [Bit::Zero, Bit::Zero, Bit::Zero, Bit::Zero];
        let out = r.eval(&reg_inputs(&d2, Bit::One, Bit::Zero, Bit::One));
        assert_eq!(out, vec![d.to_vec()], "no edge, value must hold");
    }

    #[test]
    fn register_undriven_enable_still_latches_matching_value_ne_false() {
        // `state.getPort(EN) != Value.FALSE` in Register.java: Unknown is
        // "not exactly FALSE", so it still enables — a real, not obvious,
        // Logisim behavior (floating EN defaults to "on").
        let mut r = register(1, Trigger::Rising);
        let out = r.eval(&reg_inputs(&[Bit::One], Bit::One, Bit::Zero, Bit::Unknown));
        assert_eq!(out, vec![vec![Bit::One]], "Unknown EN still latches");
    }

    #[test]
    fn register_explicit_false_enable_blocks_latching() {
        let mut r = register(1, Trigger::Rising);
        let out = r.eval(&reg_inputs(&[Bit::One], Bit::One, Bit::Zero, Bit::Zero));
        assert_eq!(out, vec![vec![Bit::Zero]], "EN=0 blocks the edge");
    }

    #[test]
    fn register_clear_wins_over_everything_without_needing_an_edge() {
        let mut r = register(4, Trigger::Rising);
        r.eval(&reg_inputs(&[Bit::One, Bit::One, Bit::One, Bit::One], Bit::One, Bit::Zero, Bit::One));
        assert_eq!(r.eval(&reg_inputs(&zeros(4), Bit::One, Bit::Zero, Bit::One))[0], vec![Bit::One; 4]);

        // CLR=1, clock still high (no new edge) — clear still wins.
        let out = r.eval(&reg_inputs(&zeros(4), Bit::One, Bit::One, Bit::One));
        assert_eq!(out, vec![zeros(4)]);
    }

    #[test]
    fn register_ignores_a_partially_undefined_input_wholesale() {
        let mut r = register(2, Trigger::Rising);
        r.eval(&reg_inputs(&[Bit::One, Bit::One], Bit::One, Bit::Zero, Bit::One)); // latches 0b11
        // Rising edge again, but D has an Unknown bit -> must not relatch,
        // not even partially.
        r.eval(&reg_inputs(&[Bit::One, Bit::Zero], Bit::Zero, Bit::Zero, Bit::One)); // drop clock first
        let out = r.eval(&reg_inputs(&[Bit::One, Bit::Unknown], Bit::One, Bit::Zero, Bit::One));
        assert_eq!(out, vec![vec![Bit::One, Bit::One]], "undefined D leaves the old value untouched");
    }

    #[test]
    fn register_set_action_writes_directly_bypassing_clock_and_enable() {
        let mut r = register(4, Trigger::Rising);
        r.invoke("set", Some(Value::Int(0b1010))).unwrap();
        assert_eq!(r.read("get"), Ok(Value::Bits(vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One])));
    }
}
