//! Adder/Subtractor/Comparator/Multiplier/Divider (`std/arith/Adder.java`/
//! `Subtractor.java`/`Comparator.java`/`Multiplier.java`/`Divider.java`).
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
//!
//! `Multiplier.computeProduct`/`Divider.computeResult`, unlike `Adder`,
//! *do* have real, verified width-32 bugs in their fast paths — see PLAN.md
//! for the full numeric trace. Both cast a signed-representable-as-negative
//! Java `int` to `long` *without* masking (`(long) a.toIntValue()`,
//! `(long) upper.toIntValue() << w`), so a width-32 operand whose top bit
//! is set gets *sign*-extended instead of zero-extended, silently
//! corrupting the result. `eval_arithmetic` below reimplements both with
//! plain `u32 -> u64` (lossless, zero-extending by construction — Rust has
//! no signed/unsigned ambiguity to fall into here) rather than replicating
//! the bug for "fidelity" — a `.circ`-compatible *behavior* model doesn't
//! mean preserving a host-language representation accident that never had
//! any hardware meaning to begin with.

use super::{bit_at, signal_to_u32_if_defined, u32_to_signal, zeros, Gate};
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

/// Shared by `Multiplier`'s `c_in` and `Divider`'s `upper`: both are
/// *width*-wide carry-chain inputs (unlike `Adder`/`Subtractor`'s
/// single-bit `c_in`/`b_in`), and Logisim only treats a *wholly*-floating
/// one as "unconnected -> zero" (`Value.isUnknown()`'s own definition:
/// every bit `Unknown`, none `Error` — verified in `Value.java`). A
/// *partially*-defined word is left alone, which then fails the
/// fully-defined fast-path check in `eval_arithmetic` and falls through to
/// the conservative Error/Unknown handling — there's no in-between
/// "half-computed" case the way `Multiplier.computeProduct`'s own Java
/// fallback attempts (see PLAN.md for why that finer-grained heuristic
/// wasn't worth replicating bit-for-bit).
fn coerce_if_wholly_unknown(signal: &Signal, bits: u8) -> Signal {
    if signal.iter().all(|&b| b == Bit::Unknown) {
        zeros(bits)
    } else {
        signal.clone()
    }
}

fn has_error(signal: &Signal) -> bool {
    signal.contains(&Bit::Error)
}

fn mask64(bits: u8) -> u64 {
    (1u64 << bits) - 1
}

impl Gate {
    pub(super) fn input_count_arithmetic(&self) -> usize {
        match self {
            Gate::Adder { .. } | Gate::Subtractor { .. } | Gate::Multiplier { .. } | Gate::Divider { .. } => 3, // in0, in1, carry/borrow-in/upper
            Gate::Comparator { .. } => 2,                                                                       // in0, in1
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
            // `Multiplier`'s c_in / `Divider`'s upper are *width*-wide,
            // unlike `Adder`/`Subtractor`'s single-bit carry — every pin is
            // uniformly `bits` wide.
            Gate::Comparator { bits, .. } | Gate::Multiplier { bits } | Gate::Divider { bits } => *bits,
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
            Gate::Multiplier { bits } | Gate::Divider { bits } => *bits, // both outputs are width-wide
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
            // `Multiplier.computeProduct`: `in0 * in1 + c_in`, split into a
            // `bits`-wide `sum`/low word and a `bits`-wide `c_out`/high
            // word — `u32 -> u64` is a lossless zero-extension (unlike
            // Java's fast path, see this module's doc comment), so no
            // masking trick is needed to stay correct at width 32; `mask64`
            // just slices out each `bits`-wide half of the up-to-64-bit
            // product; `bits <= 32` (enforced at compile time) keeps that
            // product `< 2^64`, never overflowing `u64`.
            Gate::Multiplier { bits } => {
                let c_in = coerce_if_wholly_unknown(&inputs[2], *bits);
                let (a, b) = (&inputs[0], &inputs[1]);
                match (signal_to_u32_if_defined(a, *bits), signal_to_u32_if_defined(b, *bits), signal_to_u32_if_defined(&c_in, *bits)) {
                    (Some(av), Some(bv), Some(cv)) => {
                        let product = (av as u64) * (bv as u64) + (cv as u64);
                        let mask = mask64(*bits);
                        vec![u32_to_signal((product & mask) as u32, *bits), u32_to_signal(((product >> *bits) & mask) as u32, *bits)]
                    }
                    _ => {
                        let out = if has_error(a) || has_error(b) || has_error(&c_in) { Bit::Error } else { Bit::Unknown };
                        vec![vec![out; *bits as usize], vec![out; *bits as usize]]
                    }
                }
            }
            // `Divider.computeResult`: `(upper:in0)` (a `2*bits`-wide
            // dividend) divided by `in1`, giving a `bits`-wide quotient and
            // remainder. `den == 0` divides by `1` instead — not an error —
            // matching `Divider.java`'s own choice exactly, unusual as it
            // is. All-`u64` unsigned throughout, so (unlike `Divider.
            // computeResult`'s verified bug, this module's doc comment)
            // there's no negative-remainder correction to get right: an
            // unsigned division is never negative to begin with.
            Gate::Divider { bits } => {
                let upper = coerce_if_wholly_unknown(&inputs[2], *bits);
                let (a, b) = (&inputs[0], &inputs[1]);
                match (signal_to_u32_if_defined(a, *bits), signal_to_u32_if_defined(b, *bits), signal_to_u32_if_defined(&upper, *bits)) {
                    (Some(av), Some(bv), Some(uv)) => {
                        let num = ((uv as u64) << *bits) | (av as u64);
                        let den = if bv == 0 { 1u64 } else { bv as u64 };
                        let mask = mask64(*bits);
                        vec![u32_to_signal(((num / den) & mask) as u32, *bits), u32_to_signal(((num % den) & mask) as u32, *bits)]
                    }
                    _ => {
                        let out = if has_error(a) || has_error(b) || has_error(&upper) { Bit::Error } else { Bit::Unknown };
                        vec![vec![out; *bits as usize], vec![out; *bits as usize]]
                    }
                }
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

    #[test]
    fn multiplies_two_defined_numbers_splitting_low_and_high_words() {
        let mut g = Gate::Multiplier { bits: 4 };
        // 12 * 5 = 60 = 0x3C: low nibble 0xC (1100), high nibble 0x3 (0011).
        let out = g.eval(&[bits(&[0, 0, 1, 1]), bits(&[1, 0, 1, 0]), vec![Bit::Zero; 4]]);
        assert_eq!(out, vec![bits(&[0, 0, 1, 1]), bits(&[1, 1, 0, 0])]);
    }

    #[test]
    fn multiplier_floating_carry_in_defaults_to_zero() {
        let mut g = Gate::Multiplier { bits: 4 };
        let out = g.eval(&[bits(&[0, 0, 1, 1]), bits(&[1, 0, 1, 0]), vec![Bit::Unknown; 4]]);
        assert_eq!(out, vec![bits(&[0, 0, 1, 1]), bits(&[1, 1, 0, 0])], "wholly-floating c_in acts as zero, same as a wired-in zero");
    }

    #[test]
    fn multiplier_partially_floating_carry_in_is_not_coerced_and_yields_unknown() {
        let mut g = Gate::Multiplier { bits: 4 };
        // Only *wholly* unknown gets coerced (`Value.isUnknown()`'s own
        // rule) — one determined bit among the rest keeps it "not fully
        // defined", falling to the Unknown/Error fallback rather than being
        // silently treated as zero.
        let out = g.eval(&[bits(&[0, 0, 1, 1]), bits(&[1, 0, 1, 0]), vec![Bit::Zero, Bit::Unknown, Bit::Zero, Bit::Zero]]);
        assert_eq!(out, vec![vec![Bit::Unknown; 4], vec![Bit::Unknown; 4]]);
    }

    #[test]
    fn multiplier_at_width_32_does_not_sign_extend_a_top_bit_set_operand() {
        // The exact scenario `Multiplier.computeProduct`'s Java fast path
        // gets wrong (PLAN.md): a = 2^31 (top bit set), b = 2. True product
        // = 2^32 exactly, so the low word is all-zero and the high word is
        // exactly 1 — Java's sign-extending cast instead produces
        // 0xFFFFFFFF for the high word. `u32 -> u64` here is a lossless
        // zero-extension, so this must come out right without any special
        // casing.
        let mut g = Gate::Multiplier { bits: 32 };
        let mut a = vec![Bit::Zero; 32];
        a[31] = Bit::One; // 2^31
        let mut b = vec![Bit::Zero; 32];
        b[1] = Bit::One; // 2
        let out = g.eval(&[a, b, vec![Bit::Zero; 32]]);
        assert_eq!(out[0], vec![Bit::Zero; 32], "low word: 2^32 mod 2^32 = 0");
        let mut expected_high = vec![Bit::Zero; 32];
        expected_high[0] = Bit::One;
        assert_eq!(out[1], expected_high, "high word: 2^32 / 2^32 = 1, not Java's 0xFFFFFFFF");
    }

    #[test]
    fn multiplier_error_bit_poisons_both_outputs() {
        let mut g = Gate::Multiplier { bits: 4 };
        let out = g.eval(&[vec![Bit::Zero, Bit::Error, Bit::Zero, Bit::Zero], bits(&[1, 0, 1, 0]), vec![Bit::Zero; 4]]);
        assert_eq!(out, vec![vec![Bit::Error; 4], vec![Bit::Error; 4]]);
    }

    #[test]
    fn divides_with_a_nonzero_remainder() {
        let mut g = Gate::Divider { bits: 4 };
        // 13 / 4 = 3 remainder 1.
        let out = g.eval(&[bits(&[1, 0, 1, 1]), bits(&[0, 0, 1, 0]), vec![Bit::Zero; 4]]);
        assert_eq!(out, vec![bits(&[1, 1, 0, 0]), bits(&[1, 0, 0, 0])], "quotient=3, remainder=1");
    }

    #[test]
    fn divider_division_by_zero_acts_as_division_by_one_not_an_error() {
        let mut g = Gate::Divider { bits: 4 };
        // `Divider.java`'s own unusual choice: b=0 is silently treated as
        // b=1 (quotient = dividend, remainder = 0), not flagged as `Error`.
        let out = g.eval(&[bits(&[1, 0, 1, 0]), vec![Bit::Zero; 4], vec![Bit::Zero; 4]]);
        assert_eq!(out, vec![bits(&[1, 0, 1, 0]), vec![Bit::Zero; 4]]);
    }

    #[test]
    fn divider_floating_upper_half_defaults_to_zero() {
        let mut g = Gate::Divider { bits: 4 };
        let out = g.eval(&[bits(&[1, 0, 1, 1]), bits(&[0, 0, 1, 0]), vec![Bit::Unknown; 4]]);
        assert_eq!(out, vec![bits(&[1, 1, 0, 0]), bits(&[1, 0, 0, 0])], "wholly-floating upper acts as zero");
    }

    #[test]
    fn divider_at_width_32_treats_the_dividend_as_unsigned_across_the_boundary() {
        // The exact scenario `Divider.computeResult`'s Java fast path gets
        // wrong (PLAN.md): `upper` = 2^31 (top bit set) turns the true
        // unsigned dividend `2^63 + a` into something Java's *signed* `long`
        // division misreads entirely (its own negative-remainder
        // "correction" doesn't fully compensate). Unsigned `u64` here needs
        // no correction at all: `num = upper*2^32 + a`, `num / den`, `num %
        // den` are already correct as plain unsigned division.
        let mut g = Gate::Divider { bits: 32 };
        let mut upper = vec![Bit::Zero; 32];
        upper[31] = Bit::One; // 2^31
        let mut a = vec![Bit::Zero; 32];
        a[0] = Bit::One; // dividend low word = 1, so num = 2^63 + 1
        let mut b = vec![Bit::Zero; 32];
        b[1] = Bit::One; // divisor = 2
        let out = g.eval(&[a, b, upper]);
        // (2^63 + 1) / 2 = 2^62 (remainder 1) — 2^62's bit pattern in the
        // low 32 bits is all-zero (2^62 = 1 << 62, well above bit 31).
        assert_eq!(out[0], vec![Bit::Zero; 32], "quotient's low 32 bits: 2^62 has no bits below bit 32 set");
        let mut expected_rem = vec![Bit::Zero; 32];
        expected_rem[0] = Bit::One;
        assert_eq!(out[1], expected_rem, "remainder = 1");
    }

    #[test]
    fn divider_error_bit_poisons_both_outputs() {
        let mut g = Gate::Divider { bits: 4 };
        let out = g.eval(&[vec![Bit::Zero, Bit::Error, Bit::Zero, Bit::Zero], bits(&[0, 0, 1, 0]), vec![Bit::Zero; 4]]);
        assert_eq!(out, vec![vec![Bit::Error; 4], vec![Bit::Error; 4]]);
    }
}
