//! Clocked/stateful components: `Clock`/`Register`/`Rom`/`Ram`. See
//! PLAN.md's "`Clock`/`Register` — один общий счётчик тактов" section for
//! the full model (one shared tick counter, not independent timers) and
//! the real bug it surfaced in `sim.rs`'s initial priming order.

use super::{bit_at, bit_to_byte, byte_to_bit, mask32, signal_to_u32_if_defined, u32_to_signal, zeros, Gate};
use plugin_abi::{ActionError, Bit, ReadoutError, Signal, Value};

/// `Ram.ATTR_BUS`'s three option strings (`"combined"`/`"asynch"`/
/// `"separate"`) — see `Gate::Ram`'s doc comment for what each one means
/// for the write path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RamBus {
    Combined,
    Asynch,
    Separate,
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

impl Gate {
    pub(super) fn init_memory(&mut self) {
        match self {
            Gate::Clock { clicks, sending, .. } => {
                *clicks = 0;
                *sending = Bit::Zero; // matches `ClockState.sending`'s initial `Value.FALSE`
            }
            Gate::Register { value, last_clock, .. } => {
                *value = 0;
                *last_clock = Bit::Zero; // matches memory `ClockState`'s initial `Value.FALSE`
            }
            // `contents` is fixed configuration (an attribute in Logisim
            // terms, loaded from `"contents"` at compile time), not
            // simulated state — same treatment as `Constant`'s `value`
            // (`wiring::init_wiring`'s doc comment) — only `held_data`
            // (the currently-driven output) resets.
            Gate::Rom { data_bits, held_data, .. } => *held_data = zeros(*data_bits),
            // Unlike `Rom`, `contents` genuinely is simulated state here
            // (writable memory) — resets to all-zero, matching a real
            // power-on/fresh-simulation `Register`-style reset.
            Gate::Ram { data_bits, contents, last_clock, held_data, .. } => {
                contents.fill(0);
                *last_clock = Bit::Zero;
                *held_data = zeros(*data_bits);
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn input_count_memory(&self) -> usize {
        match self {
            // d, ck, clr, en — this fixed order is what `compile.rs`'s
            // register geometry and `eval_memory` below both key off of.
            Gate::Register { .. } => 4,
            Gate::Clock { .. } => 0,
            Gate::Rom { .. } => 2, // addr, cs
            // addr, cs, oe, clr, clk, we, din — `Separate` alone gets a
            // dedicated write pair; `clk` is always reserved (even for
            // `Asynch`, which never reads it) so every non-`Separate` mode
            // shares one shape rather than branching pin *counts* on top
            // of pin *meanings*.
            Gate::Ram { bus: RamBus::Separate, .. } => 7,
            // addr, cs, oe, clr, clk, data_in — `Combined`/`Asynch` share
            // this shape; `data_in` senses the bidirectional bus itself
            // (there's no separate write pin without `Separate`).
            Gate::Ram { .. } => 6,
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    /// d, ck, clr, en — same fixed order as `input_count_memory`; only `d`
    /// is `bits` wide, the three control pins are always single-bit.
    pub(super) fn input_width_memory(&self, pin: usize) -> u8 {
        match self {
            Gate::Register { bits, .. } => if pin == 0 { *bits } else { 1 },
            Gate::Rom { addr_bits, .. } => {
                if pin == 0 {
                    *addr_bits
                } else {
                    1 // cs
                }
            }
            Gate::Ram { addr_bits, data_bits, bus, .. } => match pin {
                0 => *addr_bits,
                1..=4 => 1, // cs, oe, clr, clk
                5 if *bus == RamBus::Separate => 1, // we
                5 => *data_bits,                    // data_in (combined/asynch)
                _ => *data_bits,                    // din (separate, pin 6)
            },
            _ => unreachable!("dispatch bug: not a memory gate with inputs"),
        }
    }

    pub(super) fn output_width_memory(&self, _pin: usize) -> u8 {
        match self {
            Gate::Clock { .. } => 1,
            Gate::Register { bits, .. } => *bits,
            Gate::Rom { data_bits, .. } | Gate::Ram { data_bits, .. } => *data_bits,
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn eval_memory(&mut self, inputs: &[Signal]) -> Vec<Signal> {
        match self {
            // Never recomputed from `inputs` — only `Gate::tick` (driven by
            // the shared tick counter) changes `sending`; `eval` just
            // re-emits the cached value, mirroring `Clock.propagate`.
            Gate::Clock { sending, .. } => vec![vec![*sending]],
            Gate::Register { bits, trigger, value, last_clock } => {
                let d = &inputs[0];
                let ck = super::bit_at(&inputs[1], 0);
                let clr = super::bit_at(&inputs[2], 0);
                let en = super::bit_at(&inputs[3], 0);

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
            Gate::Rom { addr_bits, data_bits, contents, held_data } => {
                let cs = bit_at(&inputs[1], 0);
                // `!chipSelect`: actively drives `Unknown`, distinct from
                // the "hold" case below — verified in `Rom.propagate`.
                if cs == Bit::Zero {
                    *held_data = vec![Bit::Unknown; *data_bits as usize];
                    return vec![held_data.clone()];
                }
                // A not-fully-defined `addr` (while still selected) drives
                // *nothing* in Java (`return` with no `setPort` call) —
                // `held_data` simply isn't touched, so it keeps re-emitting
                // whatever it last drove.
                if let Some(addr) = signal_to_u32_if_defined(&inputs[0], *addr_bits) {
                    *held_data = u32_to_signal(contents[addr as usize], *data_bits);
                }
                vec![held_data.clone()]
            }
            Gate::Ram { addr_bits, data_bits, bus, contents, last_clock, held_data } => {
                let cs = bit_at(&inputs[1], 0);
                let oe = bit_at(&inputs[2], 0);
                let clr = bit_at(&inputs[3], 0);
                let ck = bit_at(&inputs[4], 0);
                let separate = *bus == RamBus::Separate;

                // Strict `== One` (verified — *not* `!= Zero` the way
                // `cs`/`oe`/`we` are): an undriven/`Unknown` `clr` must not
                // clear.
                let should_clear = clr == Bit::One;
                // `Asynch` writes combinationally (`triggered` unconditionally
                // `true`, `Ram.propagate`'s `asynch ||`); otherwise a real
                // rising edge, same `Trigger` mechanism as `Register`, just
                // always `Rising` (`Ram.java` hardcodes `StdAttr.TRIG_RISING`).
                let triggered = matches!(bus, RamBus::Asynch) || Trigger::Rising.fired(*last_clock, ck);
                *last_clock = ck;

                // Ordered exactly like `propagate`: the clear happens
                // before the chip-select gate, so `clr` works even while
                // `!cs`.
                if should_clear {
                    contents.fill(0);
                }

                let chip_select = cs != Bit::Zero;
                if !chip_select {
                    *held_data = vec![Bit::Unknown; *data_bits as usize];
                    return vec![held_data.clone()];
                }

                let Some(addr) = signal_to_u32_if_defined(&inputs[0], *addr_bits) else {
                    return vec![held_data.clone()]; // hold, same as `Rom`
                };

                let output_enabled = oe != Bit::Zero;
                if !should_clear && triggered {
                    let should_store = if separate { bit_at(&inputs[5], 0) != Bit::Zero } else { !output_enabled };
                    if should_store {
                        // `separate`: pin 6 (`din`); otherwise pin 5 doubles
                        // as the sensed write value off the shared bus.
                        let data_value = if separate { &inputs[6] } else { &inputs[5] };
                        // An undefined write value stores `mask32(data_bits)`
                        // (all-ones), not an error — see `Gate::Ram`'s doc
                        // comment (`MemContents.set`'s `value & mask`
                        // applied to Java's `toIntValue() == -1`).
                        let raw = signal_to_u32_if_defined(data_value, *data_bits).unwrap_or(u32::MAX);
                        contents[addr as usize] = raw & mask32(*data_bits);
                    }
                }

                *held_data =
                    if output_enabled { u32_to_signal(contents[addr as usize], *data_bits) } else { vec![Bit::Unknown; *data_bits as usize] };
                vec![held_data.clone()]
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn serialize_memory(&self) -> Vec<u8> {
        match self {
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
            Gate::Rom { contents, held_data, .. } => {
                let mut out: Vec<u8> = contents.iter().flat_map(|v| v.to_le_bytes()).collect();
                out.extend(held_data.iter().copied().map(bit_to_byte));
                out
            }
            Gate::Ram { contents, held_data, last_clock, .. } => {
                let mut out: Vec<u8> = contents.iter().flat_map(|v| v.to_le_bytes()).collect();
                out.extend(held_data.iter().copied().map(bit_to_byte));
                out.push(bit_to_byte(*last_clock));
                out
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn deserialize_memory(&mut self, state: &[u8]) {
        match self {
            Gate::Clock { clicks, sending, .. } => {
                *clicks = state.get(0..8).and_then(|b| b.try_into().ok()).map(u64::from_le_bytes).unwrap_or(0);
                *sending = state.get(8).copied().map(byte_to_bit).unwrap_or(Bit::Zero);
            }
            Gate::Register { value, last_clock, .. } => {
                *value = state.get(0..4).and_then(|b| b.try_into().ok()).map(u32::from_le_bytes).unwrap_or(0);
                *last_clock = state.get(4).copied().map(byte_to_bit).unwrap_or(Bit::Zero);
            }
            Gate::Rom { data_bits, contents, held_data, .. } => {
                for (i, cell) in contents.iter_mut().enumerate() {
                    let off = i * 4;
                    *cell = state.get(off..off + 4).and_then(|b| b.try_into().ok()).map(u32::from_le_bytes).unwrap_or(0);
                }
                let held_off = contents.len() * 4;
                *held_data = (0..*data_bits as usize).map(|i| state.get(held_off + i).copied().map(byte_to_bit).unwrap_or(Bit::Zero)).collect();
            }
            Gate::Ram { data_bits, contents, last_clock, held_data, .. } => {
                for (i, cell) in contents.iter_mut().enumerate() {
                    let off = i * 4;
                    *cell = state.get(off..off + 4).and_then(|b| b.try_into().ok()).map(u32::from_le_bytes).unwrap_or(0);
                }
                let held_off = contents.len() * 4;
                *held_data = (0..*data_bits as usize).map(|i| state.get(held_off + i).copied().map(byte_to_bit).unwrap_or(Bit::Zero)).collect();
                *last_clock = state.get(held_off + *data_bits as usize).copied().map(byte_to_bit).unwrap_or(Bit::Zero);
            }
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn actions_memory(&self) -> Vec<&'static str> {
        match self {
            // Manual Poke-toggle (`ClockPoker`) — pulses the clock once,
            // independent of `Simulation::tick`, and permanently shifts its
            // phase parity (see `Gate::Clock`'s doc comment).
            Gate::Clock { .. } => vec!["toggle"],
            // Poke-tool-style direct write, bypassing the clock/enable
            // logic entirely — useful for `.ctest` to seed a register's
            // initial contents without stepping a clock edge.
            Gate::Register { .. } => vec!["set"],
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn invoke_memory(&mut self, name: &str, arg: Option<Value>) -> Result<(), ActionError> {
        match self {
            // `ClockPoker.mouseReleased`, verbatim: flip `sending` *and*
            // bump `clicks` together — the parity shift is what keeps
            // future `tick()` calls consistent with this manual flip.
            Gate::Clock { clicks, sending, .. } if name == "toggle" => {
                *sending = sending.not();
                *clicks += 1;
                Ok(())
            }
            Gate::Clock { .. } => Err(ActionError::UnknownAction(name.to_string())),
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
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    pub(super) fn read_memory(&self, name: &str) -> Result<Value, ReadoutError> {
        match self {
            Gate::Clock { sending, .. } if name == "get" => Ok(Value::Bits(vec![*sending])),
            Gate::Register { bits, value, .. } if name == "get" => Ok(Value::Bits(u32_to_signal(*value, *bits))),
            Gate::Clock { .. } | Gate::Register { .. } => Err(ReadoutError::UnknownReadout(name.to_string())),
            _ => unreachable!("dispatch bug: not a memory gate"),
        }
    }

    /// Advances a `Clock`'s cached output to reflect the *shared* tick
    /// counter (see the `Gate::Clock` doc comment) — a no-op, returning
    /// `false`, for every other gate kind (defensively, via the early
    /// return below, rather than routed through `mod.rs`'s dispatcher —
    /// `tick` isn't a `Component` method, `Simulation` calls it on every
    /// gate index in `clock_gates`, all of which are already `Clock`, so
    /// this is belt-and-suspenders, not load-bearing). Returns whether the
    /// cached value actually changed, so `Simulation::tick` only needs to
    /// reschedule the clocks that did (though rescheduling unconditionally
    /// would also be harmless — `step()` already no-ops on an unchanged
    /// output).
    pub(super) fn tick_memory(&mut self, global_tick: u64) -> bool {
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

#[cfg(test)]
mod tests {
    use super::*;
    use plugin_abi::Component;

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

    fn zeros(bits: u8) -> Signal {
        vec![Bit::Zero; bits as usize]
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

    fn rom(addr_bits: u8, data_bits: u8, contents: Vec<u32>) -> Gate {
        Gate::Rom { addr_bits, data_bits, contents, held_data: zeros(data_bits) }
    }

    fn rom_inputs(addr: &[Bit], cs: Bit) -> Vec<Signal> {
        vec![addr.to_vec(), vec![cs]]
    }

    #[test]
    fn rom_reads_the_addressed_cell_when_selected() {
        let mut r = rom(2, 4, vec![0x1, 0xA, 0x3, 0xF]); // 4 cells, 2-bit address
        // addr=1 (LSB-first: [1,0]) -> contents[1] = 0xA = 0b1010
        let out = r.eval(&rom_inputs(&[Bit::One, Bit::Zero], Bit::One));
        assert_eq!(out, vec![vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One]]);
    }

    #[test]
    fn rom_deselected_floats_actively_not_holds() {
        let mut r = rom(2, 4, vec![0xF, 0xF, 0xF, 0xF]);
        r.eval(&rom_inputs(&[Bit::One, Bit::Zero], Bit::One)); // select + read once
        let out = r.eval(&rom_inputs(&[Bit::One, Bit::Zero], Bit::Zero)); // cs=0
        assert_eq!(out, vec![vec![Bit::Unknown; 4]], "cs=0 actively drives Unknown, verified in Rom.propagate");
    }

    #[test]
    fn rom_undefined_address_holds_the_previous_reading() {
        let mut r = rom(2, 4, vec![0x1, 0xA, 0x3, 0xF]);
        let first = r.eval(&rom_inputs(&[Bit::One, Bit::Zero], Bit::One)); // addr=1 -> 0xA
        let held = r.eval(&rom_inputs(&[Bit::One, Bit::Unknown], Bit::One)); // addr now partially undefined
        assert_eq!(held, first, "an undefined address while still selected must not drive a new (e.g. floating) value");
    }

    #[test]
    fn rom_undriven_cs_defaults_to_selected() {
        // `state.getPort(CS) != Value.FALSE` — Unknown counts as selected,
        // same convention as Register's EN/Mux's enable.
        let mut r = rom(2, 4, vec![0x1, 0xA, 0x3, 0xF]);
        let out = r.eval(&rom_inputs(&[Bit::One, Bit::Zero], Bit::Unknown));
        assert_eq!(out, vec![vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One]]);
    }

    fn ram(bus: RamBus, addr_bits: u8, data_bits: u8) -> Gate {
        Gate::Ram { addr_bits, data_bits, bus, contents: vec![0; 1usize << addr_bits], last_clock: Bit::Zero, held_data: zeros(data_bits) }
    }

    // combined/asynch: [addr, cs, oe, clr, clk, data]
    fn ram_bus_inputs(addr: &[Bit], cs: Bit, oe: Bit, clr: Bit, clk: Bit, data: &[Bit]) -> Vec<Signal> {
        vec![addr.to_vec(), vec![cs], vec![oe], vec![clr], vec![clk], data.to_vec()]
    }

    #[test]
    fn ram_combined_write_on_rising_edge_then_read_back() {
        let mut r = ram(RamBus::Combined, 2, 4);
        let addr = [Bit::One, Bit::Zero]; // addr=1
        let data = [Bit::One, Bit::One, Bit::Zero, Bit::Zero]; // 0b0011

        // Write mode: oe=0 means "not reading" -> sensed DATA line is stored
        // on the rising edge. Clock starts low first so the next eval is a
        // real edge, not an already-high level.
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &data));
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &data));

        // Now read it back: oe=1.
        let out = r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, &zeros(4)));
        assert_eq!(out, vec![data.to_vec()]);
    }

    #[test]
    fn ram_combined_write_needs_a_real_edge_not_just_a_high_level() {
        let mut r = ram(RamBus::Combined, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let data = [Bit::One; 4];
        let other = [Bit::Zero; 4];

        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &other)); // clk=0: establishes the low baseline, not yet triggered
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &data)); // clk 0->1: a real edge, writes `data`
        // clk stays high with different data on the bus: no *new* edge, must not re-write.
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &other));

        let out = r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, &zeros(4))); // read back
        assert_eq!(out, vec![data.to_vec()], "clock held high (no new edge) must not have re-triggered a write");
    }

    #[test]
    fn ram_asynch_writes_without_needing_any_clock_edge() {
        let mut r = ram(RamBus::Asynch, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let data = [Bit::Zero, Bit::One, Bit::One, Bit::Zero];

        // clk held at Zero throughout (no edge at all) — still writes,
        // since `triggered` is unconditionally true for `Asynch`.
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &data));
        let out = r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::Zero, &zeros(4)));
        assert_eq!(out, vec![data.to_vec()]);
    }

    #[test]
    fn ram_deselected_floats_actively_not_holds() {
        let mut r = ram(RamBus::Combined, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let data = [Bit::One; 4];
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &data));
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &data)); // write

        let out = r.eval(&ram_bus_inputs(&addr, Bit::Zero, Bit::One, Bit::Zero, Bit::One, &zeros(4))); // cs=0
        assert_eq!(out, vec![vec![Bit::Unknown; 4]]);
    }

    #[test]
    fn ram_undefined_address_holds_the_previous_reading() {
        let mut r = ram(RamBus::Combined, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let data = [Bit::One, Bit::Zero, Bit::One, Bit::Zero];
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &data));
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &data)); // write
        let first = r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, &zeros(4))); // read

        let bad_addr = [Bit::One, Bit::Unknown];
        let held = r.eval(&ram_bus_inputs(&bad_addr, Bit::One, Bit::One, Bit::Zero, Bit::One, &zeros(4)));
        assert_eq!(held, first);
    }

    /// `clr` is a strict `== One` check (verified in `Ram.propagate`) —
    /// unlike `cs`/`oe`/`we`'s `!= Zero`, an undriven/`Unknown` `clr` must
    /// NOT clear.
    #[test]
    fn ram_clear_is_strict_equality_unknown_does_not_clear() {
        let mut r = ram(RamBus::Combined, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let data = [Bit::One; 4];
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &data));
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &data)); // write

        // clr=Unknown: must not clear.
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Unknown, Bit::One, &zeros(4)));
        let out = r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Unknown, Bit::One, &zeros(4)));
        assert_eq!(out, vec![data.to_vec()], "Unknown clr must not have cleared the cell");
    }

    #[test]
    fn ram_clear_wins_even_while_deselected() {
        let mut r = ram(RamBus::Combined, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let data = [Bit::One; 4];
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &data));
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &data)); // write

        // cs=0 (deselected) but clr=1 — Java clears before the chip-select
        // gate, so this must still wipe the cell.
        r.eval(&ram_bus_inputs(&addr, Bit::Zero, Bit::One, Bit::One, Bit::One, &zeros(4)));
        let out = r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, &zeros(4)));
        assert_eq!(out, vec![zeros(4)], "clear must have taken effect despite !cs");
    }

    #[test]
    fn ram_write_with_undefined_data_stores_all_ones_not_an_error() {
        let mut r = ram(RamBus::Asynch, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &[Bit::One, Bit::Unknown, Bit::Zero, Bit::Zero]));
        let out = r.eval(&ram_bus_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::Zero, &zeros(4)));
        assert_eq!(out, vec![vec![Bit::One; 4]], "MemContents.set masks toIntValue()'s -1 sentinel to all-ones");
    }

    // separate: [addr, cs, oe, clr, clk, we, din]
    fn ram_separate_inputs(addr: &[Bit], cs: Bit, oe: Bit, clr: Bit, clk: Bit, we: Bit, din: &[Bit]) -> Vec<Signal> {
        vec![addr.to_vec(), vec![cs], vec![oe], vec![clr], vec![clk], vec![we], din.to_vec()]
    }

    #[test]
    fn ram_separate_bus_writes_via_din_we_not_the_data_line() {
        let mut r = ram(RamBus::Separate, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let din = [Bit::Zero, Bit::One, Bit::Zero, Bit::One];

        // oe=1 (would-be "read" mode in combined bus) but that no longer
        // blocks writes here — `we` alone decides.
        r.eval(&ram_separate_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::Zero, Bit::One, &din));
        r.eval(&ram_separate_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, Bit::One, &din));

        let out = r.eval(&ram_separate_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, Bit::Zero, &zeros(4)));
        assert_eq!(out, vec![din.to_vec()]);
    }

    #[test]
    fn ram_separate_bus_we_off_does_not_write() {
        let mut r = ram(RamBus::Separate, 2, 4);
        let addr = [Bit::One, Bit::Zero];
        let din = [Bit::One; 4];

        r.eval(&ram_separate_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, &din)); // we=0
        r.eval(&ram_separate_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, Bit::Zero, &din)); // rising edge, still we=0

        let out = r.eval(&ram_separate_inputs(&addr, Bit::One, Bit::One, Bit::Zero, Bit::One, Bit::Zero, &zeros(4)));
        assert_eq!(out, vec![zeros(4)], "we=0 must not have written");
    }
}
