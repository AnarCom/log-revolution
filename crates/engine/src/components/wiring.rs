//! Sources, sinks, and the pull resistor: Constant/InputPin/OutputPin/
//! PullResistor/BitExtender.

use super::{bit_at, bit_to_byte, byte_to_bit, u32_to_signal, zeros, Gate};
use plugin_abi::{ActionError, Bit, ReadoutError, Signal, Value};

/// `BitExtender.java`'s `ATTR_TYPE` option (`"zero"`/`"one"`/`"sign"`/
/// `"input"`) — which bit fills every position beyond the input's own
/// width. See `Gate::BitExtender`'s doc comment for the per-variant
/// meaning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtendMode {
    Zero,
    One,
    Sign,
    Input,
}

impl Gate {
    pub(super) fn init_wiring(&mut self) {
        match self {
            Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } => *value = zeros(*bits),
            // Not runtime state to reset — fixed at instantiation (an
            // attribute in Logisim terms, not simulated state).
            Gate::Constant { .. } | Gate::PullResistor { .. } | Gate::BitExtender { .. } => {}
            _ => unreachable!("dispatch bug: not a wiring gate"),
        }
    }

    pub(super) fn input_count_wiring(&self) -> usize {
        match self {
            Gate::OutputPin { .. } => 1,
            Gate::BitExtender { mode, .. } => 1 + (*mode == ExtendMode::Input) as usize,
            // Real Logisim's `propagate` is a no-op for these too — none of
            // them ever react to anything, they're all sources.
            Gate::InputPin { .. } | Gate::PullResistor { .. } | Gate::Constant { .. } => 0,
            _ => unreachable!("dispatch bug: not a wiring gate"),
        }
    }

    pub(super) fn input_width_wiring(&self, pin: usize) -> u8 {
        match self {
            Gate::OutputPin { bits, .. } => *bits,
            Gate::BitExtender { in_bits, .. } => {
                if pin == 0 {
                    *in_bits
                } else {
                    1 // the optional `extend` control pin, `mode == Input` only
                }
            }
            _ => unreachable!("dispatch bug: not a wiring gate with an input"),
        }
    }

    /// `PullResistor` stays fixed at 1 bit — see `Gate::PullResistor`'s doc
    /// comment (per-net width inference for it is deferred, PLAN.md §14).
    pub(super) fn output_width_wiring(&self, _pin: usize) -> u8 {
        match self {
            Gate::Constant { bits, .. } | Gate::InputPin { bits, .. } => *bits,
            Gate::PullResistor { .. } => 1,
            Gate::BitExtender { out_bits, .. } => *out_bits,
            _ => unreachable!("dispatch bug: not a wiring gate with an output"),
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
            Gate::BitExtender { in_bits, out_bits, mode } => {
                let data = &inputs[0];
                // `Value.extendWidth`, verified: the fill bit is derived
                // *once* (not re-decided per position) and broadcast to
                // every bit beyond `in_bits` — same shape whether it's a
                // fixed constant (`Zero`/`One`) or read from `in`/a control
                // pin (`Sign`/`Input`). `in_bits >= 1` always (this
                // schema's own `width_attr` range), so `in_bits - 1` never
                // underflows the way `Sign`'s Java guard (`win > 0`) exists
                // to prevent.
                let fill = match mode {
                    ExtendMode::Zero => Bit::Zero,
                    ExtendMode::One => Bit::One,
                    ExtendMode::Sign => bit_at(data, *in_bits as usize - 1),
                    ExtendMode::Input => bit_at(&inputs[1], 0),
                };
                let out: Signal = (0..*out_bits as usize).map(|i| if i < *in_bits as usize { bit_at(data, i) } else { fill }).collect();
                vec![out]
            }
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

    fn four_bits(pattern: u8) -> Signal {
        (0..4).map(|i| if (pattern >> i) & 1 != 0 { Bit::One } else { Bit::Zero }).collect()
    }

    #[test]
    fn bit_extender_zero_mode_fills_high_bits_with_zero() {
        let mut e = Gate::BitExtender { in_bits: 4, out_bits: 8, mode: ExtendMode::Zero };
        let out = e.eval(&[four_bits(0b1011)]);
        assert_eq!(out, vec![vec![Bit::One, Bit::One, Bit::Zero, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, Bit::Zero]]);
    }

    #[test]
    fn bit_extender_one_mode_fills_high_bits_with_one() {
        let mut e = Gate::BitExtender { in_bits: 4, out_bits: 8, mode: ExtendMode::One };
        let out = e.eval(&[four_bits(0b0001)]);
        assert_eq!(out, vec![vec![Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, Bit::One, Bit::One, Bit::One, Bit::One]]);
    }

    /// The fill bit is `in`'s own MSB, derived once and broadcast — not
    /// re-decided per new position (matters when that MSB is itself
    /// `Unknown`/`Error`, not just `0`/`1`).
    #[test]
    fn bit_extender_sign_mode_repeats_the_input_msb() {
        let mut e = Gate::BitExtender { in_bits: 4, out_bits: 8, mode: ExtendMode::Sign };
        let out = e.eval(&[four_bits(0b1011)]); // MSB (bit 3) = 1
        assert_eq!(out, vec![vec![Bit::One, Bit::One, Bit::Zero, Bit::One, Bit::One, Bit::One, Bit::One, Bit::One]]);

        let out = e.eval(&[four_bits(0b0011)]); // MSB (bit 3) = 0
        assert_eq!(out, vec![vec![Bit::One, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, Bit::Zero, Bit::Zero, Bit::Zero]]);
    }

    #[test]
    fn bit_extender_input_mode_repeats_the_control_pin() {
        let mut e = Gate::BitExtender { in_bits: 4, out_bits: 8, mode: ExtendMode::Input };
        let out = e.eval(&[four_bits(0b0001), vec![Bit::Unknown]]);
        assert_eq!(out, vec![vec![Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, Bit::Unknown, Bit::Unknown, Bit::Unknown, Bit::Unknown]]);
    }

    /// `out_bits <= in_bits` truncates — keeps the low bits, drops the
    /// rest, regardless of `mode` (nothing to extend with).
    #[test]
    fn bit_extender_truncates_when_out_bits_is_smaller() {
        let mut e = Gate::BitExtender { in_bits: 4, out_bits: 2, mode: ExtendMode::One };
        let out = e.eval(&[four_bits(0b1011)]);
        assert_eq!(out, vec![vec![Bit::One, Bit::One]]);
    }
}
