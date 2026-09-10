//! Built-in gates — PLAN.md §10 Phase 2, now covering the default Logisim
//! gate set: And/Or/Not/Nand/Nor/Xor/Xnor/Buffer/Constant plus the earlier
//! InputPin/OutputPin/PullResistor, plus the clocked pair Clock/Register,
//! all with configurable bit width (1..=32 — `logisim-port`'s own
//! `Value.MAX_WIDTH`, verified in `Value.java`) and, for the multi-input
//! gates, configurable arity (2..=32, `GateAttributes.MAX_INPUTS`).
//! Splitters, muxes, RAM etc. still come later.
//!
//! `Gate` implements `plugin_abi::Component` directly, via `match` — same
//! contract as WASM plugins (§5), but dispatched statically (an enum, no
//! `dyn`/vtable), which is exactly the "built-ins pay no boundary cost"
//! decision from §3.
//!
//! One flat `enum Gate` (not a wrapper-of-categories enum) on purpose: it's
//! what `netlist.rs`/`compile.rs`/`sim.rs` already construct and
//! pattern-match directly (`Gate::PullResistor { .. }`, `Gate::Clock { .. }`,
//! ...) — changing that shape would ripple through all three. Instead, only
//! the *implementation* is split by category: each `Component` method here
//! is a thin dispatch into a per-category inherent method
//! (`eval_logic`/`eval_wiring`/`eval_memory`, ...) defined in this module's
//! `logic`/`wiring`/`memory` submodules — legal in Rust (inherent `impl`
//! blocks for a type aren't required to live in the file that defines the
//! type, only in the same crate), and it's what actually gets each
//! category's logic and tests into their own file.

mod arithmetic;
mod logic;
mod memory;
mod plexers;
mod wiring;

pub use memory::Trigger;

use plugin_abi::{ActionError, Bit, Component, ReadoutError, Signal, Value};

#[derive(Debug, Clone, PartialEq)]
pub enum Gate {
    And { bits: u8, inputs: usize },
    Or { bits: u8, inputs: usize },
    Not { bits: u8 },
    /// `Value.java`'s Nand/Nor aren't separate ops — every `*Gate.java` in
    /// `std/gates` that negates is `computeAnd`/`computeOr` followed by
    /// `.not()`; same here, see `logic::eval_logic`.
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
    /// `std/plexers/Multiplexer.java`: routes one of `2^select_bits` data
    /// inputs (input order: data lines, then select, then `enable` if
    /// `has_enable`) to the single output, chosen by `select`. Stateless —
    /// see `plexers::eval_plexers` for the enable/select decision tree
    /// (shared with `Demux`, sverено line-by-line against the Java source).
    Mux { bits: u8, select_bits: u8, has_enable: bool, disabled_zero: bool },
    /// `std/plexers/Demultiplexer.java`: routes the single data input
    /// (input order: select, then `enable` if `has_enable`, then data) to
    /// one of `2^select_bits` outputs; every *other* output gets `tristate
    /// ? Unknown : Zero` (Demux's own extra attribute, absent on Mux —
    /// there's only one output there, nothing to default).
    Demux { bits: u8, select_bits: u8, has_enable: bool, disabled_zero: bool, tristate: bool },
    /// `std/arith/Adder.java`: `in0 + in1 + c_in`, bit-serial (see
    /// `arithmetic::ripple_carry_add` for why — not a native-int port of
    /// the Java fast path). Input order: `in0`, `in1`, `c_in`; output
    /// order: `sum`, `c_out`.
    Adder { bits: u8 },
    /// `std/arith/Subtractor.java`: `in0 - in1 - b_in`, built by feeding
    /// `Adder`'s own bit-serial adder `in0 + ~in1 + ~b_in` — not a
    /// separately reimplemented borrow chain, same as the Java source.
    /// Input order: `in0`, `in1`, `b_in`; output order: `diff`, `b_out`.
    Subtractor { bits: u8 },
    /// `std/arith/Comparator.java`: three single-bit outputs (`gt`, `eq`,
    /// `lt`) from an MSB-down bitwise comparison; `signed` (`mode` attr,
    /// `"twosComplement"`/`"unsigned"`) only changes how the *top* bit's
    /// mismatch is read — see `arithmetic::eval_arithmetic`.
    Comparator { bits: u8, signed: bool },
    /// `std/arith/Multiplier.java`: `in0 * in1 + c_in`, split into a
    /// `bits`-wide low word (`sum`) and `bits`-wide high word (`c_out`) —
    /// **not** a native-int port of `computeProduct`'s fast path, which has
    /// a verified width-32 sign-extension bug (PLAN.md,
    /// `arithmetic::eval_arithmetic`'s doc). Input order: `in0`, `in1`,
    /// `c_in` (all `bits`-wide, unlike `Adder`'s single-bit carry); output
    /// order: `sum`, `c_out`.
    Multiplier { bits: u8 },
    /// `std/arith/Divider.java`: `(upper:in0) / in1`, `(upper:in0) % in1` —
    /// **not** a native-int port of `computeResult`'s fast path, which has
    /// its own verified width-32 bug (`(long) upper.toIntValue() << w`
    /// sign-extends what should be an unsigned dividend). Input order:
    /// `in0`, `in1`, `upper` (all `bits`-wide); output order: `out`, `rem`.
    Divider { bits: u8 },
}

/// One input pin's bit `i`, or `Unknown` if that pin is unconnected/narrower
/// than expected — matches `Value.createUnknown`: a floating/undersized
/// input reads as indeterminate, not silently `Zero`. Shared by all three
/// categories, so it lives here rather than in any one of them.
fn bit_at(signal: &Signal, i: usize) -> Bit {
    signal.get(i).copied().unwrap_or(Bit::Unknown)
}

fn zeros(bits: u8) -> Signal {
    vec![Bit::Zero; bits as usize]
}

/// Little-endian (bit 0 = LSB, same convention as `ctest::bits_to_u64` and
/// `InputPin`'s own `"set"` action) — `None` if any bit isn't exactly
/// `Zero`/`One`, mirroring `Value.isFullyDefined`/`toIntValue`'s pairing in
/// `Register.propagate`: a not-fully-defined `d` is ignored outright, not
/// partially latched. Shared by `wiring` (`Constant`... actually just
/// `u32_to_signal`) and `memory` (`Register`), so it lives here.
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

impl Gate {
    /// Propagation delay in ticks. Only things that can be *downstream* of
    /// another gate in the same instant need a nonzero delay — see
    /// PLAN.md §3's parallel-batch note in `crates/engine/src/sim.rs` for
    /// why that matters, not just for realism. `Register`'s `8` (not `1`)
    /// mirrors `Register.DELAY` verbatim — real Logisim gives it a longer
    /// settle than a plain gate; harmless here either way since the
    /// parallel-batch invariant only needs delay `>= 1`, not any specific
    /// value. Spans all three categories with three different constants,
    /// so unlike the `Component` methods below it isn't worth delegating —
    /// there's no nontrivial per-category logic to hide.
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
            // `Plexers.DELAY`, verified in `Plexers.java`.
            Gate::Mux { .. } | Gate::Demux { .. } => 3,
            Gate::InputPin { .. }
            | Gate::OutputPin { .. }
            | Gate::PullResistor { .. }
            | Gate::Constant { .. }
            | Gate::Clock { .. } => 0,
            // `(width + 2) * Adder.PER_DELAY` (`PER_DELAY = 1`), verified in
            // `Adder.java`/`Comparator.java` — wider adders/comparators
            // settle slower, same as real ripple-carry hardware would.
            Gate::Adder { bits } | Gate::Comparator { bits, .. } => *bits as u64 + 2,
            // `(width + 4) * Adder.PER_DELAY`, verified in
            // `Subtractor.java` — a couple extra ticks over `Adder` for the
            // two `not()`s bracketing the shared adder.
            Gate::Subtractor { bits } => *bits as u64 + 4,
            // `width * (width + 2) * PER_DELAY`, verified in
            // `Multiplier.java`/`Divider.java` — quadratic in width,
            // matching how much slower real multiply/divide hardware is
            // than a plain adder.
            Gate::Multiplier { bits } | Gate::Divider { bits } => *bits as u64 * (*bits as u64 + 2),
        }
    }

    /// Advances a `Clock`'s cached output to reflect the *shared* tick
    /// counter — a no-op, returning `false`, for every other gate kind.
    /// See `memory::tick_memory` for the real logic and `Gate::Clock`'s doc
    /// comment for why this exists separately from `eval` at all.
    pub fn tick(&mut self, global_tick: u64) -> bool {
        self.tick_memory(global_tick)
    }

    /// Declared width of input pin `pin` — not part of `Component` (the
    /// generic plugin ABI has no notion of per-pin width), only needed by
    /// `netlist.rs`'s `Builder`, which already works with the concrete
    /// `Gate` type directly. Needed because `Splitter` (`compile/
    /// splitter.rs`) wires individual *bits* of a pin, not just whole pins
    /// — `sim.rs::gather_inputs` must know a pin's true width even for
    /// bits nothing drives, so a floating bit reads as an explicit
    /// `Unknown` entry rather than silently shortening the assembled
    /// signal (which would, e.g., make `Register`'s `D.isFullyDefined()`
    /// check see a shorter-than-real word and wrongly call it fully
    /// defined).
    pub fn input_width(&self, pin: usize) -> u8 {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.input_width_memory(pin),
            Gate::Constant { .. } | Gate::InputPin { .. } | Gate::OutputPin { .. } | Gate::PullResistor { .. } => self.input_width_wiring(pin),
            Gate::Mux { .. } | Gate::Demux { .. } => self.input_width_plexers(pin),
            Gate::Adder { .. } | Gate::Subtractor { .. } | Gate::Comparator { .. } | Gate::Multiplier { .. } | Gate::Divider { .. } => self.input_width_arithmetic(pin),
            _ => self.input_width_logic(pin),
        }
    }

    /// Declared width of output pin `pin` — see `input_width`'s doc for why
    /// this exists; used the same way, to size `Builder::add_gate`'s
    /// `fanout` entries.
    pub fn output_width(&self, pin: usize) -> u8 {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.output_width_memory(pin),
            Gate::Constant { .. } | Gate::InputPin { .. } | Gate::OutputPin { .. } | Gate::PullResistor { .. } => self.output_width_wiring(pin),
            Gate::Mux { .. } | Gate::Demux { .. } => self.output_width_plexers(pin),
            Gate::Adder { .. } | Gate::Subtractor { .. } | Gate::Comparator { .. } | Gate::Multiplier { .. } | Gate::Divider { .. } => self.output_width_arithmetic(pin),
            _ => self.output_width_logic(pin),
        }
    }
}

impl Component for Gate {
    fn init(&mut self) {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.init_memory(),
            Gate::Constant { .. } | Gate::InputPin { .. } | Gate::OutputPin { .. } | Gate::PullResistor { .. } => self.init_wiring(),
            // Logic gates carry no runtime state to reset — fixed at
            // instantiation (an attribute in Logisim terms), never
            // simulated state.
            _ => {}
        }
    }

    fn input_count(&self) -> usize {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.input_count_memory(),
            Gate::Constant { .. } | Gate::InputPin { .. } | Gate::OutputPin { .. } | Gate::PullResistor { .. } => self.input_count_wiring(),
            Gate::Mux { .. } | Gate::Demux { .. } => self.input_count_plexers(),
            Gate::Adder { .. } | Gate::Subtractor { .. } | Gate::Comparator { .. } | Gate::Multiplier { .. } | Gate::Divider { .. } => self.input_count_arithmetic(),
            _ => self.input_count_logic(),
        }
    }

    fn output_count(&self) -> usize {
        match self {
            Gate::OutputPin { .. } => 0,
            // The only gate kind with more than one output pin — everything
            // else (including `Mux`) is exactly 1.
            Gate::Demux { select_bits, .. } => 1usize << select_bits,
            Gate::Adder { .. } | Gate::Subtractor { .. } | Gate::Multiplier { .. } | Gate::Divider { .. } => 2, // sum/diff, carry/borrow-out
            Gate::Comparator { .. } => 3,                      // gt, eq, lt
            _ => 1,
        }
    }

    fn eval(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.eval_memory(inputs),
            Gate::Constant { .. } | Gate::InputPin { .. } | Gate::OutputPin { .. } | Gate::PullResistor { .. } => self.eval_wiring(inputs),
            Gate::Mux { .. } | Gate::Demux { .. } => self.eval_plexers(inputs),
            Gate::Adder { .. } | Gate::Subtractor { .. } | Gate::Comparator { .. } | Gate::Multiplier { .. } | Gate::Divider { .. } => self.eval_arithmetic(inputs),
            _ => self.eval_logic(inputs),
        }
    }

    fn serialize_state(&self) -> Vec<u8> {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.serialize_memory(),
            Gate::InputPin { .. } | Gate::OutputPin { .. } => self.serialize_wiring(),
            // Configuration, not runtime state — nothing to persist.
            _ => Vec::new(),
        }
    }

    fn deserialize_state(&mut self, state: &[u8]) {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.deserialize_memory(state),
            Gate::InputPin { .. } | Gate::OutputPin { .. } => self.deserialize_wiring(state),
            _ => {}
        }
    }

    fn actions(&self) -> Vec<&'static str> {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.actions_memory(),
            Gate::InputPin { .. } => self.actions_wiring(),
            _ => Vec::new(),
        }
    }

    fn invoke(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.invoke_memory(name, arg),
            Gate::InputPin { .. } => self.invoke_wiring(name, arg),
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
        match self {
            Gate::Clock { .. } | Gate::Register { .. } => self.read_memory(name),
            Gate::InputPin { .. } | Gate::OutputPin { .. } => self.read_wiring(name),
            _ => Err(ReadoutError::UnknownReadout(name.to_string())),
        }
    }
}
