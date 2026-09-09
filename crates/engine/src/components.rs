//! Built-in gates — start of PLAN.md §10 Phase 2. Deliberately just
//! And/Or/Not/InputPin/OutputPin for now (per the "start with simple
//! circuits" scope), all single-bit. Bus width, splitters, memory etc.
//! come later.
//!
//! `Gate` implements `plugin_abi::Component` directly, via `match` — same
//! contract as WASM plugins (§5), but dispatched statically (an enum, no
//! `dyn`/vtable), which is exactly the "built-ins pay no boundary cost"
//! decision from §3.
//!
//! Gate logic (`and`/`or`/`not`) is `Bit::and`/`or`/`not` verbatim — those
//! already mirror `logisim-port`'s `Value.java` (PLAN.md §9 fidelity), so
//! there's no separate truth table to keep in sync here.

use plugin_abi::{ActionError, Bit, Component, ReadoutError, Signal, Value};

#[derive(Debug, Clone, PartialEq)]
pub enum Gate {
    And,
    Or,
    Not,
    /// A source: value only changes via `invoke("set", ...)`, never via
    /// `eval` — nothing upstream drives it.
    InputPin { value: Bit },
    /// A sink: one input, no fanout of its own; `eval` just mirrors the
    /// input so `read("get")` can report it.
    OutputPin { value: Bit },
}

impl Gate {
    /// Propagation delay in ticks. Only `And`/`Or`/`Not` (the only things
    /// that can be *downstream* of another gate in the same instant) need
    /// a nonzero delay — see PLAN.md §3's parallel-batch note in
    /// `crates/engine/src/sim.rs` for why that matters, not just for
    /// realism.
    pub fn delay(&self) -> u64 {
        match self {
            Gate::And | Gate::Or | Gate::Not => 1,
            Gate::InputPin { .. } | Gate::OutputPin { .. } => 0,
        }
    }

    /// An unconnected pin reads as `Unknown` (nothing driving it), not
    /// `Zero` — matches `Value.createUnknown`, and matters: `Zero` would
    /// silently make a floating AND-gate input dominate as false instead
    /// of showing up as indeterminate.
    fn bit(signal: &Signal) -> Bit {
        signal.first().copied().unwrap_or(Bit::Unknown)
    }
}

impl Component for Gate {
    fn init(&mut self) {
        match self {
            Gate::InputPin { value } | Gate::OutputPin { value } => *value = Bit::Zero,
            Gate::And | Gate::Or | Gate::Not => {}
        }
    }

    fn input_count(&self) -> usize {
        match self {
            Gate::And | Gate::Or => 2,
            Gate::Not | Gate::OutputPin { .. } => 1,
            Gate::InputPin { .. } => 0,
        }
    }

    fn output_count(&self) -> usize {
        match self {
            Gate::And | Gate::Or | Gate::Not | Gate::InputPin { .. } => 1,
            Gate::OutputPin { .. } => 0,
        }
    }

    fn eval(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::And => vec![vec![Self::bit(&inputs[0]).and(Self::bit(&inputs[1]))]],
            Gate::Or => vec![vec![Self::bit(&inputs[0]).or(Self::bit(&inputs[1]))]],
            Gate::Not => vec![vec![Self::bit(&inputs[0]).not()]],
            Gate::InputPin { value } => vec![vec![*value]],
            Gate::OutputPin { value } => {
                *value = Self::bit(&inputs[0]);
                Vec::new()
            }
        }
    }

    fn serialize_state(&self) -> Vec<u8> {
        match self {
            Gate::InputPin { value } | Gate::OutputPin { value } => vec![bit_to_byte(*value)],
            Gate::And | Gate::Or | Gate::Not => Vec::new(),
        }
    }

    fn deserialize_state(&mut self, state: &[u8]) {
        if let Gate::InputPin { value } | Gate::OutputPin { value } = self {
            *value = state.first().copied().map(byte_to_bit).unwrap_or(Bit::Zero);
        }
    }

    fn actions(&self) -> Vec<&'static str> {
        match self {
            Gate::InputPin { .. } => vec!["set", "on", "off", "toggle"],
            _ => Vec::new(),
        }
    }

    fn invoke(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        let Gate::InputPin { value } = self else {
            return Err(ActionError::UnknownAction(name.to_string()));
        };
        match (name, arg) {
            ("on", _) => {
                *value = Bit::One;
                Ok(())
            }
            ("off", _) => {
                *value = Bit::Zero;
                Ok(())
            }
            ("toggle", _) => {
                *value = if *value == Bit::One { Bit::Zero } else { Bit::One };
                Ok(())
            }
            ("set", Some(Value::Bool(b))) => {
                *value = if b { Bit::One } else { Bit::Zero };
                Ok(())
            }
            ("set", Some(Value::Int(i))) => {
                *value = if i != 0 { Bit::One } else { Bit::Zero };
                Ok(())
            }
            ("set", arg) => Err(ActionError::InvalidArg {
                action: "set".to_string(),
                reason: format!("expected Bool or Int, got {arg:?}"),
            }),
            (other, _) => Err(ActionError::UnknownAction(other.to_string())),
        }
    }

    fn readouts(&self) -> Vec<&'static str> {
        match self {
            Gate::InputPin { .. } | Gate::OutputPin { .. } => vec!["get"],
            _ => Vec::new(),
        }
    }

    fn read(&self, name: &str) -> Result<Value, ReadoutError> {
        match (self, name) {
            // Bits, not Bool: collapsing to true/false would silently hide
            // Unknown/Error, which a `.ctest` assertion should be able to
            // see (PLAN.md §7).
            (Gate::InputPin { value } | Gate::OutputPin { value }, "get") => {
                Ok(Value::Bits(vec![*value]))
            }
            (_, other) => Err(ReadoutError::UnknownReadout(other.to_string())),
        }
    }
}

fn bit_to_byte(b: Bit) -> u8 {
    match b {
        Bit::Zero => 0,
        Bit::One => 1,
        Bit::Unknown => 2,
        Bit::Error => 3,
    }
}

fn byte_to_bit(b: u8) -> Bit {
    match b {
        1 => Bit::One,
        2 => Bit::Unknown,
        3 => Bit::Error,
        _ => Bit::Zero,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eval1(gate: &mut Gate, inputs: &[Bit]) -> Bit {
        let signals: Vec<Signal> = inputs.iter().map(|&b| vec![b]).collect();
        gate.eval(&signals)[0][0]
    }

    #[test]
    fn and_or_not_match_logisim_truth_tables() {
        assert_eq!(eval1(&mut Gate::And, &[Bit::One, Bit::One]), Bit::One);
        assert_eq!(eval1(&mut Gate::And, &[Bit::Zero, Bit::Unknown]), Bit::Zero);
        assert_eq!(eval1(&mut Gate::Or, &[Bit::Zero, Bit::Zero]), Bit::Zero);
        assert_eq!(eval1(&mut Gate::Or, &[Bit::One, Bit::Error]), Bit::One);
        assert_eq!(eval1(&mut Gate::Not, &[Bit::One]), Bit::Zero);
    }

    #[test]
    fn floating_input_reads_as_unknown_not_zero() {
        // No input connected at all -> `inputs` is empty for that pin.
        let mut and = Gate::And;
        let out = and.eval(&[vec![Bit::One], vec![]]);
        assert_eq!(out, vec![vec![Bit::Error]]); // One.and(Unknown) = Error, not "One.and(Zero) = Zero"
    }

    #[test]
    fn input_pin_actions_and_readout_round_trip() {
        let mut pin = Gate::InputPin { value: Bit::Zero };
        pin.invoke("on", None).unwrap();
        assert_eq!(pin.read("get"), Ok(Value::Bits(vec![Bit::One])));
        pin.invoke("toggle", None).unwrap();
        assert_eq!(pin.read("get"), Ok(Value::Bits(vec![Bit::Zero])));
    }
}
