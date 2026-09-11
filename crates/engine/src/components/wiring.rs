//! Sources, sinks, and the pull resistor: Constant/InputPin/OutputPin/
//! PullResistor/BitExtender/HexDigit/Transistor.
//!
//! `HexDigit`'s `value` is always 8 bits (`digit`'s 4 bits, `dot`'s 1 bit,
//! and the 7-segment lookup below never touch anything past that) —
//! unlike `InputPin`/`OutputPin`, whose width is configurable, so it has
//! no `bits` field of its own and can't always share their match arms.

use super::{bit_at, bit_to_byte, byte_to_bit, signal_to_u32_if_defined, u32_to_signal, zeros, Gate};
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
            Gate::HexDigit { value } => *value = zeros(8),
            // Not runtime state to reset — fixed at instantiation (an
            // attribute in Logisim terms, not simulated state).
            Gate::Constant { .. } | Gate::PullResistor { .. } | Gate::BitExtender { .. } | Gate::Transistor { .. } => {}
            _ => unreachable!("dispatch bug: not a wiring gate"),
        }
    }

    pub(super) fn input_count_wiring(&self) -> usize {
        match self {
            Gate::OutputPin { .. } => 1,
            Gate::BitExtender { mode, .. } => 1 + (*mode == ExtendMode::Input) as usize,
            Gate::Transistor { .. } => 2, // input, gate
            Gate::HexDigit { .. } => 2, // digit, dot
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
            Gate::HexDigit { .. } => {
                if pin == 0 {
                    4 // digit
                } else {
                    1 // dot
                }
            }
            Gate::Transistor { bits, .. } => {
                if pin == 0 {
                    *bits // input
                } else {
                    1 // gate
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
            Gate::Transistor { bits, .. } => *bits,
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
            Gate::HexDigit { value } => {
                let digit = &inputs[0];
                let dot = bit_at(&inputs[1], 0);
                *value = u32_to_signal(hex_digit_summary(digit, dot) as u32, 8);
                Vec::new()
            }
            Gate::Transistor { bits, conducts_on } => {
                vec![transistor_output(bit_at(&inputs[1], 0), &inputs[0], *bits, *conducts_on)]
            }
            _ => unreachable!("dispatch bug: not a wiring gate"),
        }
    }

    /// Only ever called for `InputPin`/`OutputPin` (see `mod.rs`'s
    /// dispatcher — `Constant`/`PullResistor` are pure configuration and
    /// return `Vec::new()` directly there, never routed here).
    pub(super) fn serialize_wiring(&self) -> Vec<u8> {
        match self {
            Gate::InputPin { value, .. } | Gate::OutputPin { value, .. } | Gate::HexDigit { value } => value.iter().copied().map(bit_to_byte).collect(),
            _ => unreachable!("dispatch bug: not InputPin/OutputPin/HexDigit"),
        }
    }

    pub(super) fn deserialize_wiring(&mut self, state: &[u8]) {
        if let Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } = self {
            *value = (0..*bits as usize).map(|i| state.get(i).copied().map(byte_to_bit).unwrap_or(Bit::Zero)).collect();
        } else if let Gate::HexDigit { value } = self {
            *value = (0..8).map(|i| state.get(i).copied().map(byte_to_bit).unwrap_or(Bit::Zero)).collect();
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
            (Gate::InputPin { value, .. } | Gate::OutputPin { value, .. } | Gate::HexDigit { value }, "get") => Ok(Value::Bits(value.clone())),
            (_, other) => Err(ReadoutError::UnknownReadout(other.to_string())),
        }
    }
}

/// `HexDigit.propagate`'s per-digit segment pattern, verified line-by-line
/// (index = digit value 0-15; each hex nibble of the `u32` is one raw
/// segment flag — see the Java source's own layout comment). Bit 4 (index
/// 16 in the loop) is unused in every entry; kept anyway so the table
/// matches the original literals exactly, not a hand-simplified version of
/// them.
const HEX_DIGIT_SEGMENTS: [u32; 16] = [
    0x1110111, 0x0000011, 0x0111110, 0x0011111, 0x1001011, 0x1011101, 0x1111101, 0x0010011, 0x1111111, 0x1011011, 0x1111011, 0x1101101,
    0x1110100, 0x0101111, 0x1111100, 0x1111000,
];

/// A digit outside 0-15 (not fully defined — see `Gate::HexDigit`'s doc
/// comment) always displays as a dash, verified against `propagate`'s own
/// `default` branch.
const HEX_DIGIT_DASH: u32 = 0x0001000;

/// `HexDigit.propagate`'s segment-flag -> summary-bit reassignment,
/// verified line-by-line: the raw per-nibble flags above get remapped into
/// a compact 8-bit "which of 7 segments + dot is lit" summary (bit
/// meanings themselves are arbitrary — nothing renders this yet, only that
/// they exactly mirror the Java source's mapping matters, so a future
/// renderer needs no translation table of its own).
fn hex_digit_summary(digit: &Signal, dot: Bit) -> u8 {
    let segs = signal_to_u32_if_defined(digit, 4).map(|v| HEX_DIGIT_SEGMENTS[v as usize]).unwrap_or(HEX_DIGIT_DASH);
    let mut summary = 0u8;
    if segs & 0x1 != 0 {
        summary |= 0b0000_0100;
    }
    if segs & 0x10 != 0 {
        summary |= 0b0000_0010;
    }
    if segs & 0x100 != 0 {
        summary |= 0b0000_1000;
    }
    if segs & 0x1000 != 0 {
        summary |= 0b0100_0000;
    }
    if segs & 0x1_0000 != 0 {
        summary |= 0b0000_0001;
    }
    if segs & 0x10_0000 != 0 {
        summary |= 0b0001_0000;
    }
    if segs & 0x100_0000 != 0 {
        summary |= 0b0010_0000;
    }
    // `state.getPort(1) == Value.TRUE` — a strict equality, not a
    // truthiness test: `Error`/`Unknown` on `dot` leaves this bit unset,
    // same as `Zero`.
    if dot == Bit::One {
        summary |= 0b1000_0000;
    }
    summary
}

/// `Transistor.computeOutput`, verified line-by-line. `gate` conducting
/// (`gate == conducts_on`, both fully defined) passes `input` straight
/// through, undisturbed — including any `Unknown`/`Error` bits already in
/// it. `gate` fully defined but *not* conducting floats every bit
/// (`Unknown`, high-impedance). An indeterminate `gate` is the one
/// non-obvious branch: a fully-defined `input` is forced to `Error`
/// outright, but a not-fully-defined `input` is mapped per bit instead —
/// `Unknown` bits stay `Unknown`, every other bit (whether it was `Zero`,
/// `One`, or already `Error`) becomes `Error`. That per-bit distinction
/// only has an observable effect while `input` is partway between defined
/// and undefined; the two rules agree once it settles either way.
fn transistor_output(gate: Bit, input: &Signal, bits: u8, conducts_on: Bit) -> Signal {
    match gate {
        Bit::Zero | Bit::One if gate == conducts_on => (0..bits as usize).map(|i| bit_at(input, i)).collect(),
        Bit::Zero | Bit::One => vec![Bit::Unknown; bits as usize],
        Bit::Unknown | Bit::Error => {
            if signal_to_u32_if_defined(input, bits).is_some() {
                vec![Bit::Error; bits as usize]
            } else {
                (0..bits as usize)
                    .map(|i| if bit_at(input, i) == Bit::Unknown { Bit::Unknown } else { Bit::Error })
                    .collect()
            }
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

    /// `HexDigit` has no output pins — `eval` stores into `value`, read
    /// back via `read("get")`, same access pattern as `OutputPin`/`LED`.
    fn hex_digit_get(digit: u8, dot: Bit) -> Value {
        let mut h = Gate::HexDigit { value: zeros(8) };
        h.eval(&[four_bits(digit), vec![dot]]);
        h.read("get").unwrap()
    }

    #[test]
    fn hex_digit_zero_lights_every_segment_except_the_middle_bar() {
        assert_eq!(hex_digit_get(0, Bit::Zero), Value::Bits(u32_to_signal(0x3F, 8)));
    }

    #[test]
    fn hex_digit_one_lights_only_the_two_right_verticals() {
        assert_eq!(hex_digit_get(1, Bit::Zero), Value::Bits(u32_to_signal(0b0000_0110, 8)));
    }

    #[test]
    fn hex_digit_eight_lights_every_segment() {
        assert_eq!(hex_digit_get(8, Bit::Zero), Value::Bits(u32_to_signal(0x7F, 8)));
    }

    #[test]
    fn hex_digit_dot_sets_the_top_bit() {
        assert_eq!(hex_digit_get(0, Bit::One), Value::Bits(u32_to_signal(0x3F | 0x80, 8)));
    }

    /// `Error`/`Unknown` on `dot` is not `One` — leaves the bit unset, same
    /// as `Zero` (`state.getPort(1) == Value.TRUE` is a strict equality).
    #[test]
    fn hex_digit_dot_error_does_not_set_the_bit() {
        assert_eq!(hex_digit_get(0, Bit::Error), Value::Bits(u32_to_signal(0x3F, 8)));
    }

    /// A `digit` outside 0-15 (not fully defined) always shows a dash —
    /// only the middle bar — not per-bit error propagation into the
    /// segments.
    #[test]
    fn hex_digit_undefined_value_displays_as_a_dash() {
        let mut h = Gate::HexDigit { value: zeros(8) };
        h.eval(&[vec![Bit::Zero, Bit::One, Bit::Unknown, Bit::Zero], vec![Bit::Zero]]);
        assert_eq!(h.read("get"), Ok(Value::Bits(u32_to_signal(0b0100_0000, 8))));
    }

    fn transistor_eval(t: &mut Gate, input: Signal, gate: Bit) -> Signal {
        t.eval(&[input, vec![gate]])[0].clone()
    }

    /// P-type conducts on `gate == Zero` (`Value.FALSE`, `ATTR_TYPE`'s
    /// default) — passes `input` through undisturbed, `Unknown`/`Error`
    /// bits included, not just a clean value.
    #[test]
    fn p_type_transistor_passes_input_through_when_gate_is_zero() {
        let mut t = Gate::Transistor { bits: 3, conducts_on: Bit::Zero };
        let input = vec![Bit::One, Bit::Unknown, Bit::Error];
        assert_eq!(transistor_eval(&mut t, input.clone(), Bit::Zero), input);
    }

    /// P-type with `gate == One` (fully defined, just not `conducts_on`) —
    /// floats every bit, doesn't just block the input.
    #[test]
    fn p_type_transistor_floats_when_gate_is_one() {
        let mut t = Gate::Transistor { bits: 3, conducts_on: Bit::Zero };
        let out = transistor_eval(&mut t, vec![Bit::One, Bit::One, Bit::One], Bit::One);
        assert_eq!(out, vec![Bit::Unknown; 3]);
    }

    /// N-type is the mirror image: conducts on `gate == One`.
    #[test]
    fn n_type_transistor_conducts_on_gate_one_not_zero() {
        let mut t = Gate::Transistor { bits: 2, conducts_on: Bit::One };
        assert_eq!(transistor_eval(&mut t, vec![Bit::One, Bit::Zero], Bit::One), vec![Bit::One, Bit::Zero]);
        assert_eq!(transistor_eval(&mut t, vec![Bit::One, Bit::Zero], Bit::Zero), vec![Bit::Unknown, Bit::Unknown]);
    }

    /// An indeterminate `gate` (`Unknown`/`Error`) with a *fully-defined*
    /// `input` forces `Error` outright — not `Unknown`, and not a
    /// per-bit pass of `input`.
    #[test]
    fn transistor_undefined_gate_with_defined_input_forces_error() {
        let mut t = Gate::Transistor { bits: 3, conducts_on: Bit::Zero };
        let out = transistor_eval(&mut t, vec![Bit::One, Bit::Zero, Bit::One], Bit::Unknown);
        assert_eq!(out, vec![Bit::Error; 3]);
        let out = transistor_eval(&mut t, vec![Bit::One, Bit::Zero, Bit::One], Bit::Error);
        assert_eq!(out, vec![Bit::Error; 3]);
    }

    /// The one branch that genuinely differs from `ControlledBuffer`'s
    /// coarser "any Unknown gate -> Error" rule: an indeterminate gate with
    /// a *partially* undefined input maps per bit — `Unknown` stays
    /// `Unknown`, everything else (including bits that were already
    /// cleanly `Zero`/`One`) becomes `Error`.
    #[test]
    fn transistor_undefined_gate_with_partially_undefined_input_maps_per_bit() {
        let mut t = Gate::Transistor { bits: 3, conducts_on: Bit::Zero };
        let out = transistor_eval(&mut t, vec![Bit::One, Bit::Unknown, Bit::Error], Bit::Unknown);
        assert_eq!(out, vec![Bit::Error, Bit::Unknown, Bit::Error]);
    }
}
