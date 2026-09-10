//! Built-in gates — PLAN.md §10 Phase 2, now covering the default Logisim
//! gate set: And/Or/Not/Nand/Nor/Xor/Xnor/Buffer/Constant plus the earlier
//! InputPin/OutputPin/PullResistor, all with configurable bit width (1..=32
//! — `logisim-port`'s own `Value.MAX_WIDTH`, verified in `Value.java`) and,
//! for the multi-input gates, configurable arity (2..=32, `GateAttributes.
//! MAX_INPUTS`). Splitters, memory, muxes etc. still come later.
//!
//! `Gate` implements `plugin_abi::Component` directly, via `match` — same
//! contract as WASM plugins (§5), but dispatched statically (an enum, no
//! `dyn`/vtable), which is exactly the "built-ins pay no boundary cost"
//! decision from §3.
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

use plugin_abi::{ActionError, Bit, Component, ReadoutError, Signal, Value};

#[derive(Debug, Clone, PartialEq)]
pub enum Gate {
    And { bits: u8, inputs: usize },
    Or { bits: u8, inputs: usize },
    Not { bits: u8 },
    /// `Value.java`'s Nand/Nor aren't separate ops — every `*Gate.java` in
    /// `std/gates` that negates is `computeAnd`/`computeOr` followed by
    /// `.not()`; same here, see `eval`.
    Nand { bits: u8, inputs: usize },
    Nor { bits: u8, inputs: usize },
    /// Default ("exactly one true input", `GateFunctions.computeExactlyOne`)
    /// — real Logisim's `GateAttributes.ATTR_XOR` also offers a chain-xor
    /// ("odd parity") mode for arity > 2; both modes agree for the common
    /// 2-input case, so that attribute isn't modeled yet (deferred, not
    /// forgotten — see PLAN.md §14).
    Xor { bits: u8, inputs: usize },
    Xnor { bits: u8, inputs: usize },
    /// Identity, just with the same propagation delay as a real gate
    /// (`gates/Buffer.java`) — not a wire, doesn't disappear at compile time.
    Buffer { bits: u8 },
    /// A source with a value fixed at instantiation (`std/wiring/
    /// Constant.java`'s `value`/`width` attrs), never simulated state.
    Constant { bits: u8, value: u32 },
    /// A source: value only changes via `invoke("set", ...)` (and
    /// `on`/`off`/`toggle` when `bits == 1`), never via `eval` — nothing
    /// upstream drives it.
    InputPin { bits: u8, value: Signal },
    /// A sink: one input, no fanout of its own; `eval` just mirrors the
    /// input so `read("get")` can report it.
    OutputPin { bits: u8, value: Signal },
    /// A weak source — pulls its point to `to` only when nothing else
    /// drives it. Deliberately *not* just another entry in the normal
    /// `combine` fold: it must lose to any real driver (even a lone
    /// well-defined one) and must not paper over a real short circuit.
    /// That two-phase rule lives in `sim.rs::gather_inputs`, matched
    /// against `logisim-port`'s `CircuitWires.pullValue` — this variant
    /// only carries the configured target (`to` can be `Error`: Logisim's
    /// own "X" pull option, for "must not be left floating"). Always
    /// 1-bit for now: real Logisim gives it `BitWidth.UNKNOWN` and
    /// auto-matches whatever net it's on, which needs per-net width
    /// inference we don't have yet (deferred, PLAN.md §14) — a pull
    /// resistor on a wide bus should be modeled per-bit externally for now.
    PullResistor { to: Bit },
}

/// One input pin's bit `i`, or `Unknown` if that pin is unconnected/narrower
/// than expected — matches `Value.createUnknown`: a floating/undersized
/// input reads as indeterminate, not silently `Zero`.
fn bit_at(signal: &Signal, i: usize) -> Bit {
    signal.get(i).copied().unwrap_or(Bit::Unknown)
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

fn zeros(bits: u8) -> Signal {
    vec![Bit::Zero; bits as usize]
}

impl Gate {
    /// Propagation delay in ticks. Only things that can be *downstream* of
    /// another gate in the same instant need a nonzero delay — see
    /// PLAN.md §3's parallel-batch note in `crates/engine/src/sim.rs` for
    /// why that matters, not just for realism.
    pub fn delay(&self) -> u64 {
        match self {
            Gate::And { .. }
            | Gate::Or { .. }
            | Gate::Not { .. }
            | Gate::Nand { .. }
            | Gate::Nor { .. }
            | Gate::Xor { .. }
            | Gate::Xnor { .. }
            | Gate::Buffer { .. } => 1,
            Gate::InputPin { .. } | Gate::OutputPin { .. } | Gate::PullResistor { .. } | Gate::Constant { .. } => 0,
        }
    }
}

impl Component for Gate {
    fn init(&mut self) {
        match self {
            Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } => *value = zeros(*bits),
            // Not runtime state to reset — fixed at instantiation (an
            // attribute in Logisim terms, not simulated state).
            Gate::And { .. }
            | Gate::Or { .. }
            | Gate::Not { .. }
            | Gate::Nand { .. }
            | Gate::Nor { .. }
            | Gate::Xor { .. }
            | Gate::Xnor { .. }
            | Gate::Buffer { .. }
            | Gate::Constant { .. }
            | Gate::PullResistor { .. } => {}
        }
    }

    fn input_count(&self) -> usize {
        match self {
            Gate::And { inputs, .. }
            | Gate::Or { inputs, .. }
            | Gate::Nand { inputs, .. }
            | Gate::Nor { inputs, .. }
            | Gate::Xor { inputs, .. }
            | Gate::Xnor { inputs, .. } => *inputs,
            Gate::Not { .. } | Gate::Buffer { .. } | Gate::OutputPin { .. } => 1,
            // Real Logisim's `propagate` is a no-op for these too — none of
            // them ever react to anything, they're all sources.
            Gate::InputPin { .. } | Gate::PullResistor { .. } | Gate::Constant { .. } => 0,
        }
    }

    fn output_count(&self) -> usize {
        match self {
            Gate::OutputPin { .. } => 0,
            _ => 1,
        }
    }

    fn eval(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::And { bits, .. } => vec![fold_bits(inputs, *bits, Bit::and)],
            Gate::Or { bits, .. } => vec![fold_bits(inputs, *bits, Bit::or)],
            Gate::Not { bits } => vec![(0..*bits as usize).map(|i| bit_at(&inputs[0], i).not()).collect()],
            Gate::Nand { bits, .. } => vec![not_bits(fold_bits(inputs, *bits, Bit::and))],
            Gate::Nor { bits, .. } => vec![not_bits(fold_bits(inputs, *bits, Bit::or))],
            Gate::Xor { bits, .. } => vec![fold_xor_one(inputs, *bits)],
            Gate::Xnor { bits, .. } => vec![not_bits(fold_xor_one(inputs, *bits))],
            Gate::Buffer { bits } => vec![(0..*bits as usize).map(|i| bit_at(&inputs[0], i)).collect()],
            Gate::Constant { bits, value } => {
                vec![(0..*bits as usize).map(|i| if (*value >> i) & 1 != 0 { Bit::One } else { Bit::Zero }).collect()]
            }
            Gate::InputPin { value, .. } => vec![value.clone()],
            Gate::OutputPin { bits, value } => {
                *value = (0..*bits as usize).map(|i| bit_at(&inputs[0], i)).collect();
                Vec::new()
            }
            Gate::PullResistor { to } => vec![vec![*to]],
        }
    }

    fn serialize_state(&self) -> Vec<u8> {
        match self {
            Gate::InputPin { value, .. } | Gate::OutputPin { value, .. } => value.iter().copied().map(bit_to_byte).collect(),
            // Configuration, not runtime state — nothing to persist.
            Gate::And { .. }
            | Gate::Or { .. }
            | Gate::Not { .. }
            | Gate::Nand { .. }
            | Gate::Nor { .. }
            | Gate::Xor { .. }
            | Gate::Xnor { .. }
            | Gate::Buffer { .. }
            | Gate::Constant { .. }
            | Gate::PullResistor { .. } => Vec::new(),
        }
    }

    fn deserialize_state(&mut self, state: &[u8]) {
        if let Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } = self {
            *value = (0..*bits as usize).map(|i| state.get(i).copied().map(byte_to_bit).unwrap_or(Bit::Zero)).collect();
        }
    }

    fn actions(&self) -> Vec<&'static str> {
        match self {
            Gate::InputPin { bits, .. } if *bits == 1 => vec!["set", "on", "off", "toggle"],
            Gate::InputPin { .. } => vec!["set"],
            _ => Vec::new(),
        }
    }

    fn invoke(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        let Gate::InputPin { bits, value } = self else {
            return Err(ActionError::UnknownAction(name.to_string()));
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
            (Gate::InputPin { value, .. } | Gate::OutputPin { value, .. }, "get") => Ok(Value::Bits(value.clone())),
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
    fn constant_ignores_inputs_and_masks_to_its_own_width() {
        let mut c = Gate::Constant { bits: 4, value: 0b1_1010 }; // 0x1A, only low 4 bits matter
        assert_eq!(c.eval(&[]), vec![vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One]]); // 0b1010 LSB-first
        assert_eq!(c.input_count(), 0);
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
