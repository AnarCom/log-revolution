//! Adder/Subtractor/Comparator (`std/arith/Adder.java`/`Subtractor.java`/
//! `Comparator.java`).
//!
//! `Adder.computeSum` in the Java source has *two* code paths: a fast one
//! that packs operands into a native `int`/`long` and relies on
//! Java's defined-overflow (modular, two's-complement wraparound)
//! semantics, and a bit-serial ripple-carry fallback for when an operand
//! isn't fully defined. Traced through carefully (this is exactly the kind
//! of place a silent width-32 boundary bug could hide, so — same standing
//! requirement as everywhere else in this codebase — verified against the
//! Java source line by line rather than assumed): the fast path is not
//! buggy in Java, because Java `int`/`long` arithmetic *is* modular
//! wraparound by spec, and extracting any single bit of a wrapped value via
//! shift-and-mask is unaffected by where the "sign" ends up. But porting
//! that trick literally into Rust would be a real hazard — Rust's
//! arithmetic operators panic on overflow in a debug build, and the exact
//! width-32 special case (`Value.MAX_WIDTH`, where a naive `1u32 << 32` is
//! itself already UB) is precisely the boundary every prior gate in this
//! codebase had to special-case carefully (`logic.rs`'s module doc,
//! `BitWidth.getMask()` in `logisim-port`). `ripple_carry_add` below sides-
//! steps the whole class of bug by never doing native fixed-width integer
//! arithmetic at all: it's the bit-serial branch alone, used
//! unconditionally (not just as a fallback) — provably equivalent to the
//! fast path for fully-defined operands, since a full adder's per-bit truth
//! table *is* the definition of binary addition, not an approximation of
//! it. Same "elementwise fold over `Bit`, not reimplemented native
//! arithmetic" principle as `logic.rs`'s multi-bit gates.

use super::{bit_at, Gate};
use plugin_abi::{Bit, Signal};

/// `Adder.computeSum`'s bit-serial branch, used unconditionally (see this
/// module's doc comment for why the fast path isn't needed at all): each
/// bit is the usual full-adder truth table (`sum = ab + bb + carry`, output
/// bit = `sum & 1`, new carry = `sum >= 2`), except `Unknown`/`Error`
/// "poison" every bit from that point up once encountered — an
/// indeterminate/conflicting carry can't produce a determinate sum bit, and
/// once poisoned the carry itself stays poisoned for every higher bit
/// (mirrors the Java loop's `carry = Value.ERROR`/`Value.UNKNOWN`
/// reassignment exactly: it never recovers to a determinate value later in
/// the same evaluation).
///
/// `c_in`'s own `Unknown` (a floating carry-in pin — the common case for a
/// standalone `Adder` whose `C_IN` nobody wired) is coerced to `Zero`
/// first, matching `computeSum`'s own `if (c_in == UNKNOWN || c_in == NIL)
/// c_in = FALSE` — deliberately *not* left floating the way, say,
/// `Register.EN`'s "anything but exactly `Zero`" idiom would suggest; an
/// actual conflict (`Error`) is left untouched, still poisons everything.
fn ripple_carry_add(a: &Signal, b: &Signal, c_in: Bit, bits: u8) -> (Signal, Bit) {
    let mut carry = if c_in == Bit::Unknown { Bit::Zero } else { c_in };
    let mut out = Vec::with_capacity(bits as usize);
    for i in 0..bits as usize {
        let bit = if carry == Bit::Error {
            Bit::Error
        } else if carry == Bit::Unknown {
            Bit::Unknown
        } else {
            let ab = bit_at(a, i);
            let bb = bit_at(b, i);
            if ab == Bit::Error || bb == Bit::Error {
                carry = Bit::Error;
                Bit::Error
            } else if ab == Bit::Unknown || bb == Bit::Unknown {
                carry = Bit::Unknown;
                Bit::Unknown
            } else {
                let sum = (ab == Bit::One) as u8 + (bb == Bit::One) as u8 + (carry == Bit::One) as u8;
                carry = if sum >= 2 { Bit::One } else { Bit::Zero };
                if sum & 1 == 1 {
                    Bit::One
                } else {
                    Bit::Zero
                }
            }
        };
        out.push(bit);
    }
    (out, carry)
}

impl Gate {
    pub(super) fn input_count_arithmetic(&self) -> usize {
        match self {
            Gate::Adder { .. } | Gate::Subtractor { .. } => 3, // in0, in1, carry/borrow-in
            Gate::Comparator { .. } => 2,                      // in0, in1
            _ => unreachable!("dispatch bug: not an arithmetic gate"),
        }
    }

    pub(super) fn input_width_arithmetic(&self, pin: usize) -> u8 {
        match self {
            Gate::Adder { bits } | Gate::Subtractor { bits } => {
                if pin < 2 {
                    *bits
                } else {
                    1
                }
            }
            Gate::Comparator { bits, .. } => *bits,
            _ => unreachable!("dispatch bug: not an arithmetic gate with inputs"),
        }
    }

    pub(super) fn output_width_arithmetic(&self, pin: usize) -> u8 {
        match self {
            Gate::Adder { bits } | Gate::Subtractor { bits } => {
                if pin == 0 {
                    *bits
                } else {
                    1
                }
            }
            Gate::Comparator { .. } => 1, // gt, eq, lt are always single bits
            _ => unreachable!("dispatch bug: not an arithmetic gate with outputs"),
        }
    }

    pub(super) fn eval_arithmetic(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::Adder { bits } => {
                let c_in = bit_at(&inputs[2], 0);
                let (sum, c_out) = ripple_carry_add(&inputs[0], &inputs[1], c_in, *bits);
                vec![sum, vec![c_out]]
            }
            // `Subtractor.propagate`: a - b - b_in == a + ~b + ~b_in,
            // reusing the very same adder rather than a separate borrow
            // chain. Its own `b_in` coercion (`Unknown` -> `Zero`) happens
            // *before* the `.not()`, not left to `ripple_carry_add`'s
            // internal one — order matters here: `Bit::not(Unknown)` is
            // `Error` (single-bit `Value.not`'s own rule, `plugin_abi`),
            // which would wrongly poison the whole subtraction if the
            // coercion ran only after negating instead of before.
            Gate::Subtractor { bits } => {
                let b_in = bit_at(&inputs[2], 0);
                let b_in = if b_in == Bit::Unknown { Bit::Zero } else { b_in };
                let (diff, c_out) = ripple_carry_add(&inputs[0], &inputs[1].iter().map(|b| b.not()).collect::<Signal>(), b_in.not(), *bits);
                vec![diff, vec![c_out.not()]]
            }
            Gate::Comparator { bits, signed } => {
                // MSB down to LSB — the first (highest) bit where the
                // operands differ decides gt/lt; equal all the way down
                // means `eq`. `signed`: at the MSB *only*, swap which
                // operand is "bigger" for a 0/1 mismatch — two's-complement
                // sign bit inverts the usual reading (MSB=1 means
                // *smaller*, not bigger) — mirrors `Comparator.propagate`'s
                // own swap-at-the-top-bit trick verbatim rather than
                // reimplementing signed comparison from scratch.
                let (mut gt, mut eq, mut lt) = (Bit::Zero, Bit::One, Bit::Zero);
                for i in (0..*bits as usize).rev() {
                    let mut ab = bit_at(&inputs[0], i);
                    let mut bb = bit_at(&inputs[1], i);
                    if i == *bits as usize - 1 && ab != bb && *signed {
                        std::mem::swap(&mut ab, &mut bb);
                    }
                    if ab == Bit::Error || bb == Bit::Error {
                        gt = Bit::Error;
                        eq = Bit::Error;
                        lt = Bit::Error;
                        break;
                    } else if ab == Bit::Unknown || bb == Bit::Unknown {
                        gt = Bit::Unknown;
                        eq = Bit::Unknown;
                        lt = Bit::Unknown;
                        break;
                    } else if ab != bb {
                        eq = Bit::Zero;
                        if ab == Bit::One {
                            gt = Bit::One;
                        } else {
                            lt = Bit::One;
                        }
                        break;
                    }
                }
                vec![vec![gt], vec![eq], vec![lt]]
            }
            _ => unreachable!("dispatch bug: not an arithmetic gate"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use plugin_abi::Component;

    fn bits(bits: &[u8]) -> Signal {
        bits.iter().map(|&b| if b == 1 { Bit::One } else { Bit::Zero }).collect()
    }

    #[test]
    fn adds_two_defined_numbers_with_correct_carry_out() {
        let mut g = Gate::Adder { bits: 4 };
        // 12 (1100) + 5 (0101) = 17, which doesn't fit in 4 bits: sum=1
        // (0001), carry_out=1. LSB-first throughout.
        let out = g.eval(&[bits(&[0, 0, 1, 1]), bits(&[1, 0, 1, 0]), vec![Bit::Zero]]);
        assert_eq!(out, vec![bits(&[1, 0, 0, 0]), vec![Bit::One]]);
    }

    #[test]
    fn floating_carry_in_defaults_to_zero_not_unknown() {
        let mut g = Gate::Adder { bits: 2 };
        // 1 + 1 with C_IN left floating (Unknown) -> treated as 0, not
        // poisoning the whole result to Unknown.
        let out = g.eval(&[bits(&[1, 0]), bits(&[1, 0]), vec![Bit::Unknown]]);
        assert_eq!(out, vec![bits(&[0, 1]), vec![Bit::Zero]]);
    }

    #[test]
    fn an_error_bit_poisons_every_higher_bit_and_the_carry_out() {
        let mut g = Gate::Adder { bits: 4 };
        let out = g.eval(&[vec![Bit::Zero, Bit::Error, Bit::Zero, Bit::Zero], bits(&[0, 0, 0, 0]), vec![Bit::Zero]]);
        assert_eq!(out, vec![vec![Bit::Zero, Bit::Error, Bit::Error, Bit::Error], vec![Bit::Error]]);
    }

    #[test]
    fn width_thirty_two_matches_the_narrower_width_path_bit_for_bit() {
        // The exact boundary that would trip up a native-int port (`1u32 <<
        // 32`): all-ones + all-ones + 1 should still ripple a carry out of
        // the top bit like any other width, with no special-casing needed
        // here (unlike the Java fast path, which needs one).
        let mut g = Gate::Adder { bits: 32 };
        let ones = vec![Bit::One; 32];
        let out = g.eval(&[ones.clone(), ones, vec![Bit::One]]);
        // (2^32-1)*2+1 mod 2^32 = 2^32-1, i.e. every bit still One.
        assert_eq!(out[0], vec![Bit::One; 32]);
        assert_eq!(out[1], vec![Bit::One], "carry out of the top bit");
    }

    #[test]
    fn subtracts_via_twos_complement_reusing_the_adder() {
        let mut g = Gate::Subtractor { bits: 4 };
        // 5 (0101) - 3 (0011) = 2 (0010); borrow_in floating -> no initial
        // borrow, borrow_out false (no underflow).
        let out = g.eval(&[bits(&[1, 0, 1, 0]), bits(&[1, 1, 0, 0]), vec![Bit::Unknown]]);
        assert_eq!(out, vec![bits(&[0, 1, 0, 0]), vec![Bit::Zero]]);
    }

    #[test]
    fn subtractor_borrow_out_signals_underflow() {
        let mut g = Gate::Subtractor { bits: 4 };
        // 3 (0011) - 5 (0101): underflows a 4-bit unsigned range -> borrow
        // out asserted, wrapped result 3-5+16=14 (1110).
        let out = g.eval(&[bits(&[1, 1, 0, 0]), bits(&[1, 0, 1, 0]), vec![Bit::Unknown]]);
        assert_eq!(out, vec![bits(&[0, 1, 1, 1]), vec![Bit::One]]);
    }

    #[test]
    fn comparator_unsigned_orders_by_raw_magnitude() {
        let mut g = Gate::Comparator { bits: 4, signed: false };
        // 0b1000 (8 unsigned) vs 0b0001 (1) -> a > b under unsigned mode
        // even though a's MSB is set.
        let out = g.eval(&[bits(&[0, 0, 0, 1]), bits(&[1, 0, 0, 0])]);
        assert_eq!(out, vec![vec![Bit::One], vec![Bit::Zero], vec![Bit::Zero]], "gt, eq, lt");
    }

    #[test]
    fn comparator_signed_treats_the_msb_as_the_sign_bit() {
        let mut g = Gate::Comparator { bits: 4, signed: true };
        // Same bit patterns as above, but signed: 0b1000 = -8, 0b0001 = 1,
        // so a < b now.
        let out = g.eval(&[bits(&[0, 0, 0, 1]), bits(&[1, 0, 0, 0])]);
        assert_eq!(out, vec![vec![Bit::Zero], vec![Bit::Zero], vec![Bit::One]], "gt, eq, lt");
    }

    #[test]
    fn comparator_reports_equal_when_every_bit_matches() {
        let mut g = Gate::Comparator { bits: 4, signed: true };
        let out = g.eval(&[bits(&[1, 0, 1, 0]), bits(&[1, 0, 1, 0])]);
        assert_eq!(out, vec![vec![Bit::Zero], vec![Bit::One], vec![Bit::Zero]]);
    }
}
