//! Sources, sinks, and the pull resistor: Constant/InputPin/OutputPin/
//! PullResistor.

use super::{bit_at, bit_to_byte, byte_to_bit, u32_to_signal, zeros, Gate};
use plugin_abi::{ActionError, Bit, ReadoutError, Signal, Value};

impl Gate {
    pub(super) fn init_wiring(&mut self) {
        match self {
            Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } => *value = zeros(*bits),
            // Not runtime state to reset — fixed at instantiation (an
            // attribute in Logisim terms, not simulated state).
            Gate::Constant { .. } | Gate::PullResistor { .. } => {}
            _ => unreachable!("dispatch bug: not a wiring gate"),
        }
    }

    pub(super) fn input_count_wiring(&self) -> usize {
        match self {
            Gate::OutputPin { .. } => 1,
            // Real Logisim's `propagate` is a no-op for these too — none of
            // them ever react to anything, they're all sources.
            Gate::InputPin { .. } | Gate::PullResistor { .. } | Gate::Constant { .. } => 0,
            _ => unreachable!("dispatch bug: not a wiring gate"),
        }
    }

    pub(super) fn eval_wiring(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::Constant { bits, value } => vec![u32_to_signal(*value, *bits)],
            Gate::InputPin { value, .. } => vec![value.clone()],
            Gate::OutputPin { bits, value } => {
                *value = (0..*bits as usize).map(|i| bit_at(&inputs[0], i)).collect();
                Vec::new()
            }
            Gate::PullResistor { to } => vec![vec![*to]],
            _ => unreachable!("dispatch bug: not a wiring gate"),
        }
    }

    /// Only ever called for `InputPin`/`OutputPin` (see `mod.rs`'s
    /// dispatcher — `Constant`/`PullResistor` are pure configuration and
    /// return `Vec::new()` directly there, never routed here).
    pub(super) fn serialize_wiring(&self) -> Vec<u8> {
        match self {
            Gate::InputPin { value, .. } | Gate::OutputPin { value, .. } => value.iter().copied().map(bit_to_byte).collect(),
            _ => unreachable!("dispatch bug: not InputPin/OutputPin"),
        }
    }

    pub(super) fn deserialize_wiring(&mut self, state: &[u8]) {
        if let Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } = self {
            *value = (0..*bits as usize).map(|i| state.get(i).copied().map(byte_to_bit).unwrap_or(Bit::Zero)).collect();
        }
    }

    /// Only ever called for `InputPin` (see `mod.rs`'s dispatcher).
    pub(super) fn actions_wiring(&self) -> Vec<&'static str> {
        match self {
            Gate::InputPin { bits, .. } if *bits == 1 => vec!["set", "on", "off", "toggle"],
            Gate::InputPin { .. } => vec!["set"],
            _ => unreachable!("dispatch bug: not InputPin"),
        }
    }

    pub(super) fn invoke_wiring(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        let Gate::InputPin { bits, value } = self else {
            unreachable!("dispatch bug: not InputPin");
        };
        match (name, arg) {
            ("on", _) if *bits == 1 => {
                *value = vec![Bit::One];
                Ok(())
            }
            ("off", _) if *bits == 1 => {
                *value = vec![Bit::Zero];
                Ok(())
            }
            ("toggle", _) if *bits == 1 => {
                *value = vec![if value[0] == Bit::One { Bit::Zero } else { Bit::One }];
                Ok(())
            }
            ("set", Some(Value::Bool(b))) if *bits == 1 => {
                *value = vec![if b { Bit::One } else { Bit::Zero }];
                Ok(())
            }
            // Little-endian, same convention as `ctest::bits_to_u64`.
            ("set", Some(Value::Int(i))) => {
                *value = (0..*bits as usize).map(|k| if (i >> k) & 1 != 0 { Bit::One } else { Bit::Zero }).collect();
                Ok(())
            }
            ("set", Some(Value::Bits(bs))) if bs.len() == *bits as usize => {
                *value = bs;
                Ok(())
            }
            ("set", arg) => Err(ActionError::InvalidArg {
                action: "set".to_string(),
                reason: format!("expected Bool (1-bit only), Int, or {bits}-bit Bits, got {arg:?}"),
            }),
            (other, _) => Err(ActionError::UnknownAction(other.to_string())),
        }
    }

    /// Only ever called for `InputPin`/`OutputPin` (see `mod.rs`'s
    /// dispatcher).
    pub(super) fn read_wiring(&self, name: &str) -> Result<Value, ReadoutError> {
        match (self, name) {
            // Bits, not Bool: collapsing to true/false would silently hide
            // Unknown/Error, which a `.ctest` assertion should be able to
            // see (PLAN.md §7).
            (Gate::InputPin { value, .. } | Gate::OutputPin { value, .. }, "get") => Ok(Value::Bits(value.clone())),
            (_, other) => Err(ReadoutError::UnknownReadout(other.to_string())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plugin_abi::Component;

    #[test]
    fn constant_ignores_inputs_and_masks_to_its_own_width() {
        let mut c = Gate::Constant { bits: 4, value: 0b1_1010 }; // 0x1A, only low 4 bits matter
        assert_eq!(c.eval(&[]), vec![vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One]]); // 0b1010 LSB-first
        assert_eq!(c.input_count(), 0);
    }

    #[test]
    fn thirty_two_bit_width_does_not_panic() {
        // The real ceiling (`Value.MAX_WIDTH`) — exercised so a future
        // off-by-one in the `1 << i` shift (i up to 31) would show up here
        // rather than only at i == 32 (which would be a shift overflow).
        let mut c = Gate::Constant { bits: 32, value: 0xFFFF_FFFF };
        let out = c.eval(&[]);
        assert_eq!(out[0].len(), 32);
        assert!(out[0].iter().all(|&b| b == Bit::One));
    }

    #[test]
    fn input_pin_actions_and_readout_round_trip() {
        let mut pin = Gate::InputPin { bits: 1, value: vec![Bit::Zero] };
        pin.invoke("on", None).unwrap();
        assert_eq!(pin.read("get"), Ok(Value::Bits(vec![Bit::One])));
        pin.invoke("toggle", None).unwrap();
        assert_eq!(pin.read("get"), Ok(Value::Bits(vec![Bit::Zero])));
    }

    #[test]
    fn wide_input_pin_sets_via_int_not_on_off_toggle() {
        let mut pin = Gate::InputPin { bits: 4, value: zeros(4) };
        assert_eq!(pin.actions(), vec!["set"]);
        assert_eq!(pin.invoke("on", None), Err(ActionError::UnknownAction("on".to_string())));
        pin.invoke("set", Some(Value::Int(0b1010))).unwrap();
        assert_eq!(pin.read("get"), Ok(Value::Bits(vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One])));
    }
}
