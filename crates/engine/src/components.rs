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
    /// A square-wave source — but *not* its own independent timer. Real
    /// Logisim has exactly one global tick counter for the whole
    /// simulation (`Propagator.ticks`); every `Clock` instance is a pure
    /// function of that shared counter and its own `high`/`low` period
    /// (`Clock.tick`, verified in `logisim-port`) — not a thing that runs
    /// on its own clock. `eval` here never recomputes `sending` itself
    /// (mirrors `Clock.propagate`, which just re-emits the cached value);
    /// only `Gate::tick` — driven once per `Simulation::tick()` call, for
    /// every clock in the design at once — advances it. `clicks` mirrors
    /// `ClockState.clicks`: a manual Poke-toggle (`invoke("toggle", ...)`)
    /// flips `sending` immediately *and* permanently shifts the phase
    /// parity for every later tick.
    Clock { high: u64, low: u64, clicks: u64, sending: Bit },
    /// Edge/level-triggered storage (`std/memory/Register.java`): latches
    /// `d` into `value` when `trigger` fires on `ck` and `en` isn't
    /// exactly `Zero` (note: *not* "exactly `One`" — an undriven/`Unknown`
    /// enable still latches, matching `state.getPort(EN) != Value.FALSE`
    /// verbatim, not the "safer-looking" reading), unless `clr` is `One`
    /// (asynchronous clear, wins outright, doesn't need a clock edge). A
    /// `d` that isn't fully defined (any `Unknown`/`Error` bit) is ignored
    /// wholesale that cycle — `value` holds its previous contents, it
    /// never latches a partially-known word (`in.isFullyDefined()`).
    /// Output is always fully defined (`value` is a plain integer, never
    /// `Unknown`/`Error` itself).
    Register { bits: u8, trigger: Trigger, value: u32, last_clock: Bit },
}

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

/// Little-endian (bit 0 = LSB, same convention as `ctest::bits_to_u64` and
/// `InputPin`'s own `"set"` action) — `None` if any bit isn't exactly
/// `Zero`/`One`, mirroring `Value.isFullyDefined`/`toIntValue`'s pairing in
/// `Register.propagate`: a not-fully-defined `d` is ignored outright, not
/// partially latched.
fn signal_to_u32_if_defined(signal: &Signal, bits: u8) -> Option<u32> {
    let mut value = 0u32;
    for i in 0..bits as usize {
        match bit_at(signal, i) {
            Bit::One => value |= 1 << i,
            Bit::Zero => {}
            Bit::Unknown | Bit::Error => return None,
        }
    }
    Some(value)
}

fn u32_to_signal(value: u32, bits: u8) -> Signal {
    (0..bits as usize).map(|i| if (value >> i) & 1 != 0 { Bit::One } else { Bit::Zero }).collect()
}

impl Gate {
    /// Propagation delay in ticks. Only things that can be *downstream* of
    /// another gate in the same instant need a nonzero delay — see
    /// PLAN.md §3's parallel-batch note in `crates/engine/src/sim.rs` for
    /// why that matters, not just for realism. `Register`'s `8` (not `1`)
    /// mirrors `Register.DELAY` verbatim — real Logisim gives it a longer
    /// settle than a plain gate; harmless here either way since the
    /// parallel-batch invariant only needs delay `>= 1`, not any specific
    /// value.
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
            Gate::Register { .. } => 8,
            Gate::InputPin { .. }
            | Gate::OutputPin { .. }
            | Gate::PullResistor { .. }
            | Gate::Constant { .. }
            | Gate::Clock { .. } => 0,
        }
    }

    /// Advances a `Clock`'s cached output to reflect the *shared* tick
    /// counter (see the `Gate::Clock` doc comment) — a no-op, returning
    /// `false`, for every other gate kind. Returns whether the cached
    /// value actually changed, so `Simulation::tick` only needs to
    /// reschedule the clocks that did (though rescheduling unconditionally
    /// would also be harmless — `step()` already no-ops on an unchanged
    /// output).
    pub fn tick(&mut self, global_tick: u64) -> bool {
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

impl Component for Gate {
    fn init(&mut self) {
        match self {
            Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } => *value = zeros(*bits),
            Gate::Clock { clicks, sending, .. } => {
                *clicks = 0;
                *sending = Bit::Zero; // matches `ClockState.sending`'s initial `Value.FALSE`
            }
            Gate::Register { value, last_clock, .. } => {
                *value = 0;
                *last_clock = Bit::Zero; // matches memory `ClockState`'s initial `Value.FALSE`
            }
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
            // d, ck, clr, en — this fixed order is what `compile.rs`'s
            // register geometry and `eval` below both key off of.
            Gate::Register { .. } => 4,
            // Real Logisim's `propagate` is a no-op for these too — none of
            // them ever react to anything, they're all sources.
            Gate::InputPin { .. } | Gate::PullResistor { .. } | Gate::Constant { .. } | Gate::Clock { .. } => 0,
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
            Gate::Constant { bits, value } => vec![u32_to_signal(*value, *bits)],
            Gate::InputPin { value, .. } => vec![value.clone()],
            Gate::OutputPin { bits, value } => {
                *value = (0..*bits as usize).map(|i| bit_at(&inputs[0], i)).collect();
                Vec::new()
            }
            Gate::PullResistor { to } => vec![vec![*to]],
            // Never recomputed from `inputs` — only `Gate::tick` (driven by
            // the shared tick counter) changes `sending`; `eval` just
            // re-emits the cached value, mirroring `Clock.propagate`.
            Gate::Clock { sending, .. } => vec![vec![*sending]],
            Gate::Register { bits, trigger, value, last_clock } => {
                let d = &inputs[0];
                let ck = bit_at(&inputs[1], 0);
                let clr = bit_at(&inputs[2], 0);
                let en = bit_at(&inputs[3], 0);

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
        }
    }

    fn serialize_state(&self) -> Vec<u8> {
        match self {
            Gate::InputPin { value, .. } | Gate::OutputPin { value, .. } => value.iter().copied().map(bit_to_byte).collect(),
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
        match self {
            Gate::InputPin { bits, value } | Gate::OutputPin { bits, value } => {
                *value = (0..*bits as usize).map(|i| state.get(i).copied().map(byte_to_bit).unwrap_or(Bit::Zero)).collect();
            }
            Gate::Clock { clicks, sending, .. } => {
                *clicks = state.get(0..8).and_then(|b| b.try_into().ok()).map(u64::from_le_bytes).unwrap_or(0);
                *sending = state.get(8).copied().map(byte_to_bit).unwrap_or(Bit::Zero);
            }
            Gate::Register { value, last_clock, .. } => {
                *value = state.get(0..4).and_then(|b| b.try_into().ok()).map(u32::from_le_bytes).unwrap_or(0);
                *last_clock = state.get(4).copied().map(byte_to_bit).unwrap_or(Bit::Zero);
            }
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

    fn actions(&self) -> Vec<&'static str> {
        match self {
            Gate::InputPin { bits, .. } if *bits == 1 => vec!["set", "on", "off", "toggle"],
            Gate::InputPin { .. } => vec!["set"],
            // Manual Poke-toggle (`ClockPoker`) — pulses the clock once,
            // independent of `Simulation::tick`, and permanently shifts its
            // phase parity (see `Gate::Clock`'s doc comment).
            Gate::Clock { .. } => vec!["toggle"],
            // Poke-tool-style direct write, bypassing the clock/enable
            // logic entirely — useful for `.ctest` to seed a register's
            // initial contents without stepping a clock edge.
            Gate::Register { .. } => vec!["set"],
            _ => Vec::new(),
        }
    }

    fn invoke(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        match self {
            Gate::InputPin { bits, value } => match (name, arg) {
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
            },
            // `ClockPoker.mouseReleased`, verbatim: flip `sending` *and*
            // bump `clicks` together — the parity shift is what keeps
            // future `tick()` calls consistent with this manual flip.
            Gate::Clock { clicks, sending, .. } if name == "toggle" => {
                *sending = sending.not();
                *clicks += 1;
                Ok(())
            }
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
            _ => Err(ActionError::UnknownAction(name.to_string())),
        }
    }

    fn readouts(&self) -> Vec<&'static str> {
        match self {
            Gate::InputPin { .. } | Gate::OutputPin { .. } | Gate::Clock { .. } | Gate::Register { .. } => vec!["get"],
            _ => Vec::new(),
        }
    }

    fn read(&self, name: &str) -> Result<Value, ReadoutError> {
        match (self, name) {
            // Bits, not Bool: collapsing to true/false would silently hide
            // Unknown/Error, which a `.ctest` assertion should be able to
            // see (PLAN.md §7).
            (Gate::InputPin { value, .. } | Gate::OutputPin { value, .. }, "get") => Ok(Value::Bits(value.clone())),
            (Gate::Clock { sending, .. }, "get") => Ok(Value::Bits(vec![*sending])),
            (Gate::Register { bits, value, .. }, "get") => Ok(Value::Bits(u32_to_signal(*value, *bits))),
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
