//! Combinational, stateless gates: And/Or/Not/Nand/Nor/Xor/Xnor/Buffer.
//!
//! Multi-bit truth tables are *not* reimplemented from `Value.java`'s
//! bitmask arithmetic (`and`/`or`/`xor`/`not` for width > 1 pack each bit
//! into an `int` and combine with `&`/`|`/`^`/`~` plus a `falses`/`trues`
//! mask). Working through that arithmetic bit-by-bit shows it's exactly
//! equivalent to applying the already-verified single-bit `Bit::and`/`or`/
//! `xor`/`not` (PLAN.md §9 fidelity) independently at each bit position —
//! Logisim packs bits into an `int` for speed, we already have `Vec<Bit>`,
//! so `fold_bits`/`fold_xor_one` below just do the elementwise version.
//! `GateFunctions.computeAnd`/`computeOr`/`computeOddParity` (the N-input
//! fold: seed = inputs[0], then combine each later input in turn) is
//! mirrored directly rather than derived, since it's already exactly that
//! shape.

use super::{bit_at, Gate};
use plugin_abi::{Bit, Signal};

impl Gate {
    pub(super) fn input_count_logic(&self) -> usize {
        match self {
            Gate::And { inputs, .. }
            | Gate::Or { inputs, .. }
            | Gate::Nand { inputs, .. }
            | Gate::Nor { inputs, .. }
            | Gate::Xor { inputs, .. }
            | Gate::Xnor { inputs, .. } => *inputs,
            Gate::Not { .. } | Gate::Buffer { .. } => 1,
            Gate::ControlledBuffer { .. } => 2, // data, enable
            _ => unreachable!("dispatch bug: not a logic gate"),
        }
    }

    pub(super) fn eval_logic(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::And { bits, .. } => vec![fold_bits(inputs, *bits, Bit::and)],
            Gate::Or { bits, .. } => vec![fold_bits(inputs, *bits, Bit::or)],
            Gate::Not { bits } => vec![(0..*bits as usize).map(|i| bit_at(&inputs[0], i).not()).collect()],
            Gate::Nand { bits, .. } => vec![not_bits(fold_bits(inputs, *bits, Bit::and))],
            Gate::Nor { bits, .. } => vec![not_bits(fold_bits(inputs, *bits, Bit::or))],
            Gate::Xor { bits, .. } => vec![fold_xor_one(inputs, *bits)],
            Gate::Xnor { bits, .. } => vec![not_bits(fold_xor_one(inputs, *bits))],
            Gate::Buffer { bits } => vec![(0..*bits as usize).map(|i| bit_at(&inputs[0], i)).collect()],
            Gate::ControlledBuffer { bits } => vec![controlled_buffer_output(bit_at(&inputs[1], 0), &inputs[0], *bits)],
            _ => unreachable!("dispatch bug: not a logic gate"),
        }
    }

    /// Every pin (however many inputs, per `input_count_logic`) is `bits`
    /// wide — none of these gates has a control pin narrower than its data,
    /// except `ControlledBuffer`, whose pin 1 (`enable`) is fixed at 1 bit
    /// regardless of `bits`.
    pub(super) fn input_width_logic(&self, pin: usize) -> u8 {
        match self {
            Gate::And { bits, .. }
            | Gate::Or { bits, .. }
            | Gate::Not { bits }
            | Gate::Nand { bits, .. }
            | Gate::Nor { bits, .. }
            | Gate::Xor { bits, .. }
            | Gate::Xnor { bits, .. }
            | Gate::Buffer { bits } => *bits,
            Gate::ControlledBuffer { bits } => {
                if pin == 0 {
                    *bits
                } else {
                    1
                }
            }
            _ => unreachable!("dispatch bug: not a logic gate"),
        }
    }

    pub(super) fn output_width_logic(&self, _pin: usize) -> u8 {
        self.input_width_logic(0)
    }
}

/// `GateFunctions.computeAnd`/`computeOr`'s N-input fold, generalized across
/// `bits` positions: seed with the first input's bit, then combine each
/// later input in turn (not a fold from an identity element — `op` isn't
/// commutative-with-`Unknown` the way that'd require, e.g. `Bit::and(Unknown,
/// Zero)` is `Zero` but `Bit::and(Unknown, One)` is `Error`, not `One`).
fn fold_bits(inputs: &[Signal], bits: u8, op: impl Fn(Bit, Bit) -> Bit) -> Signal {
    (0..bits as usize)
        .map(|i| {
            let mut acc = bit_at(&inputs[0], i);
            for signal in &inputs[1..] {
                acc = op(acc, bit_at(signal, i));
            }
            acc
        })
        .collect()
}

/// `GateFunctions.computeExactlyOne`: at each bit position, `Error` if any
/// input is `Unknown`/`Error` there, else `One` iff exactly one input is
/// `One` there.
fn fold_xor_one(inputs: &[Signal], bits: u8) -> Signal {
    (0..bits as usize)
        .map(|i| {
            let mut ones = 0u32;
            let mut error = false;
            for signal in inputs {
                match bit_at(signal, i) {
                    Bit::One => ones += 1,
                    Bit::Zero => {}
                    Bit::Unknown | Bit::Error => error = true,
                }
            }
            if error {
                Bit::Error
            } else if ones == 1 {
                Bit::One
            } else {
                Bit::Zero
            }
        })
        .collect()
}

fn not_bits(signal: Signal) -> Signal {
    signal.into_iter().map(Bit::not).collect()
}

/// `ControlledBuffer.propagate`, verified line-by-line against
/// `logisim-port`: `control == One` passes `data` through; `Error` *and*
/// `Unknown` both drive the output to `Error` outright. This is the
/// opposite convention from `Mux`/`Demux`'s `enable_status` (`plexers.rs`),
/// where an undriven `Unknown` enable defaults to "active" — here it's
/// grouped with a genuine conflict instead. Only `control == Zero` gives
/// the "disabled" reading, `Unknown` (floating/high-Z) — the real source's
/// remaining branch (`Value.NIL`, gated behind a global "undefined gate"
/// project option we don't model) is never reachable from a live
/// `getPort` read, so it's folded into the `Zero` case rather than modeled
/// separately.
fn controlled_buffer_output(control: Bit, data: &Signal, bits: u8) -> Signal {
    match control {
        Bit::One => (0..bits as usize).map(|i| bit_at(data, i)).collect(),
        Bit::Error | Bit::Unknown => vec![Bit::Error; bits as usize],
        Bit::Zero => vec![Bit::Unknown; bits as usize],
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plugin_abi::Component;

    fn eval1(gate: &mut Gate, inputs: &[Bit]) -> Bit {
        let signals: Vec<Signal> = inputs.iter().map(|&b| vec![b]).collect();
        gate.eval(&signals)[0][0]
    }

    fn and2() -> Gate {
        Gate::And { bits: 1, inputs: 2 }
    }
    fn or2() -> Gate {
        Gate::Or { bits: 1, inputs: 2 }
    }
    fn not1() -> Gate {
        Gate::Not { bits: 1 }
    }

    #[test]
    fn and_or_not_match_logisim_truth_tables() {
        assert_eq!(eval1(&mut and2(), &[Bit::One, Bit::One]), Bit::One);
        assert_eq!(eval1(&mut and2(), &[Bit::Zero, Bit::Unknown]), Bit::Zero);
        assert_eq!(eval1(&mut or2(), &[Bit::Zero, Bit::Zero]), Bit::Zero);
        assert_eq!(eval1(&mut or2(), &[Bit::One, Bit::Error]), Bit::One);
        assert_eq!(eval1(&mut not1(), &[Bit::One]), Bit::Zero);
    }

    #[test]
    fn floating_input_reads_as_unknown_not_zero() {
        // No input connected at all -> `inputs` is empty for that pin.
        let mut and = and2();
        let out = and.eval(&[vec![Bit::One], vec![]]);
        assert_eq!(out, vec![vec![Bit::Error]]); // One.and(Unknown) = Error, not "One.and(Zero) = Zero"
    }

    #[test]
    fn nand_nor_xnor_negate_their_positive_counterpart() {
        let mut nand = Gate::Nand { bits: 1, inputs: 2 };
        assert_eq!(eval1(&mut nand, &[Bit::One, Bit::One]), Bit::Zero);
        assert_eq!(eval1(&mut nand, &[Bit::Zero, Bit::One]), Bit::One);

        let mut nor = Gate::Nor { bits: 1, inputs: 2 };
        assert_eq!(eval1(&mut nor, &[Bit::Zero, Bit::Zero]), Bit::One);
        assert_eq!(eval1(&mut nor, &[Bit::One, Bit::Zero]), Bit::Zero);

        let mut xnor = Gate::Xnor { bits: 1, inputs: 2 };
        assert_eq!(eval1(&mut xnor, &[Bit::One, Bit::Zero]), Bit::Zero);
        assert_eq!(eval1(&mut xnor, &[Bit::One, Bit::One]), Bit::One);
    }

    #[test]
    fn xor_is_exactly_one_true_input_not_plain_parity_for_wider_arity() {
        // GateFunctions.computeExactlyOne: three inputs, all true -> not
        // "exactly one", so False — a plain 2-input-style parity chain
        // (True xor True xor True = True) would get this wrong.
        let mut xor3 = Gate::Xor { bits: 1, inputs: 3 };
        assert_eq!(eval1(&mut xor3, &[Bit::One, Bit::One, Bit::One]), Bit::Zero);
        assert_eq!(eval1(&mut xor3, &[Bit::One, Bit::Zero, Bit::Zero]), Bit::One);
        assert_eq!(eval1(&mut xor3, &[Bit::One, Bit::One, Bit::Zero]), Bit::Zero);
    }

    #[test]
    fn buffer_is_delayed_identity() {
        let mut buf = Gate::Buffer { bits: 1 };
        assert_eq!(buf.eval(&[vec![Bit::One]]), vec![vec![Bit::One]]);
        assert_eq!(buf.delay(), 1);
    }

    #[test]
    fn controlled_buffer_passes_data_through_when_enabled() {
        let mut cb = Gate::ControlledBuffer { bits: 4 };
        // inputs: [data, enable]
        let out = cb.eval(&[vec![Bit::One, Bit::Zero, Bit::One, Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::One, Bit::Zero, Bit::One, Bit::One]]);
        assert_eq!(cb.delay(), 1);
    }

    #[test]
    fn controlled_buffer_disabled_floats_not_zero() {
        let mut cb = Gate::ControlledBuffer { bits: 4 };
        let out = cb.eval(&[vec![Bit::One, Bit::One, Bit::One, Bit::One], vec![Bit::Zero]]);
        assert_eq!(out, vec![vec![Bit::Unknown; 4]]);
    }

    /// The key divergence from `Mux`/`Demux`'s `enable_status`: there, an
    /// undriven (`Unknown`) enable defaults to "active". Here it's grouped
    /// with `Error` instead — verified against `ControlledBuffer.propagate`.
    #[test]
    fn controlled_buffer_undriven_enable_yields_error_not_passthrough() {
        let mut cb = Gate::ControlledBuffer { bits: 2 };
        let out = cb.eval(&[vec![Bit::One, Bit::One], vec![Bit::Unknown]]);
        assert_eq!(out, vec![vec![Bit::Error, Bit::Error]]);
    }

    #[test]
    fn controlled_buffer_conflicting_enable_drivers_yield_error() {
        let mut cb = Gate::ControlledBuffer { bits: 2 };
        let out = cb.eval(&[vec![Bit::One, Bit::One], vec![Bit::Error]]);
        assert_eq!(out, vec![vec![Bit::Error, Bit::Error]]);
    }

    #[test]
    fn multi_bit_and_is_elementwise_not_a_single_wide_comparison() {
        let mut and8 = Gate::And { bits: 4, inputs: 2 };
        let a = vec![Bit::One, Bit::One, Bit::Zero, Bit::Unknown];
        let b = vec![Bit::One, Bit::Zero, Bit::Zero, Bit::Zero];
        // bit0: 1&1=1; bit1: 1&0=0; bit2: 0&0=0 (Zero absorbs even Unknown
        // elsewhere doesn't matter here); bit3: Unknown&Zero=Zero (Zero
        // still absorbing on that bit specifically).
        assert_eq!(and8.eval(&[a, b]), vec![vec![Bit::One, Bit::Zero, Bit::Zero, Bit::Zero]]);
    }
}
