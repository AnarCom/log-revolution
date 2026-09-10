//! Compiles the clocked components: Clock/Register/Rom/Ram.

use super::{source_geometry, width_attr, CompileError, Geometry};
use crate::components::{RamBus, Trigger};
use crate::file_format::{Circuit, ComponentInstance};
use crate::netlist::TemplateNode;

/// `Register`'s 4 fixed inputs, in the exact order
/// `components::memory::eval_memory` expects them (`d`, `ck`, `clr`, `en` —
/// see `Gate::input_count`'s doc): D on the west side, the three control
/// pins spread along the south edge (matches `Register.java`'s own port
/// layout in spirit, not pixel-for-pixel — nothing renders this yet).
/// Output Q on the east side.
fn register_geometry() -> Geometry {
    Geometry {
        inputs: vec![(0, 0), (1, 2), (2, 2), (3, 2)], // d, ck, clr, en
        outputs: vec![(4, 0)],
    }
}

/// `attrs["highDuration"]`/`attrs["lowDuration"]` for `core:Clock` —
/// defaulting to 1 each, same as `Clock.ATTR_HIGH`/`ATTR_LOW`'s own
/// defaults (`Integer.valueOf(1)` in the Java constructor). Must be >= 1
/// (`DurationAttribute`'s own range); no upper bound, matching Java's
/// `Integer.MAX_VALUE` ceiling closely enough that rejecting on overflow
/// into `i64` isn't worth the complexity here.
fn clock_duration_attr(circuit: &Circuit, comp: &ComponentInstance, key: &'static str) -> Result<u64, CompileError> {
    let raw = comp.attrs.get(key).and_then(|v| v.as_i64()).unwrap_or(1);
    if raw >= 1 {
        Ok(raw as u64)
    } else {
        Err(CompileError::InvalidClockDuration { circuit: circuit.name.clone(), id: comp.id.clone(), field: key, value: raw })
    }
}

/// `attrs["trigger"]` for `core:Register` — `StdAttr.TRIGGER`'s own option
/// strings, defaulting to `"rising"` (`StdAttr.TRIG_RISING`, `Register`'s
/// own default).
fn trigger_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<Trigger, CompileError> {
    let raw = comp.attrs.get("trigger").and_then(|v| v.as_str()).unwrap_or("rising");
    match raw {
        "rising" => Ok(Trigger::Rising),
        "falling" => Ok(Trigger::Falling),
        "high" => Ok(Trigger::High),
        "low" => Ok(Trigger::Low),
        other => Err(CompileError::InvalidTrigger { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

/// `attrs["addrWidth"]` for `core:Rom`/`core:Ram` — `Mem.ADDR_ATTR`'s own
/// name, range (2..=24, `Attributes.forBitWidth("addrWidth", .., 2, 24)`),
/// and default (8, both `RomAttributes`'s and `Ram.DEFAULTS`'s own
/// default `BitWidth.create(8)`) — narrower and differently-defaulted than
/// the general `width_attr`, so it isn't reused here.
fn addr_width_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<u8, CompileError> {
    let raw = comp.attrs.get("addrWidth").and_then(|v| v.as_i64()).unwrap_or(8);
    if (2..=24).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidWidth { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

/// `attrs["dataWidth"]` for `core:Rom`/`core:Ram` — `Mem.DATA_ATTR`'s own
/// name and default (8, same source as `addr_width_attr`'s); range is the
/// ordinary 1..=32 (`Attributes.forBitWidth("dataWidth", ..)`'s single-arg
/// overload, same ceiling as `width_attr`), only the *default* differs
/// (8, not 1), so this stays a separate helper rather than reusing
/// `width_attr` outright.
fn data_width_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<u8, CompileError> {
    let raw = comp.attrs.get("dataWidth").and_then(|v| v.as_i64()).unwrap_or(8);
    if (1..=32).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidWidth { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

/// `attrs["contents"]` for `core:Rom` — a JSON array of integers,
/// address-ordered: this schema's own encoding, not `.circ`'s hex-file
/// format (translating *that* is the importer's job, PLAN.md §9).
/// Missing/absent, or entries beyond `1 << addr_bits`, are simply not
/// there to read — same "blank until loaded" default `Rom` itself starts
/// at (`RomAttributes`'s own `MemContents.create`, all-zero). Each present
/// entry is masked to `data_bits`, same as a real write masks
/// (`MemContents.set`'s `value & mask`) — not rejected outright, since an
/// oversized literal here is far more likely a harmless "I wrote the full
/// 32-bit constant" than a schema bug worth failing compilation over.
fn rom_contents_attr(comp: &ComponentInstance, addr_bits: u8, data_bits: u8) -> Vec<u32> {
    let len = 1usize << addr_bits;
    let mask = if data_bits == 32 { u32::MAX } else { (1u32 << data_bits) - 1 };
    let mut contents = vec![0u32; len];
    if let Some(entries) = comp.attrs.get("contents").and_then(|v| v.as_array()) {
        for (cell, entry) in contents.iter_mut().zip(entries) {
            *cell = (entry.as_i64().unwrap_or(0) as u32) & mask;
        }
    }
    contents
}

/// `attrs["bus"]` for `core:Ram` — `Ram.ATTR_BUS`'s own three option
/// strings, defaulting to `"combined"` (`Ram.DEFAULTS`'s own
/// `BUS_COMBINED`).
fn ram_bus_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<RamBus, CompileError> {
    let raw = comp.attrs.get("bus").and_then(|v| v.as_str()).unwrap_or("combined");
    match raw {
        "combined" => Ok(RamBus::Combined),
        "asynch" => Ok(RamBus::Asynch),
        "separate" => Ok(RamBus::Separate),
        other => Err(CompileError::InvalidRamBus { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

/// `addr`, `cs` -> `data` — matches `Gate::Rom`'s expected input order.
fn rom_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0), (0, -2)], outputs: vec![(2, 0)] }
}

/// `addr`, `cs`, `oe`, `clr`, `clk`, then either `we`+`din` (`Separate`) or
/// a `data_in` that lands on the *exact same point* as `data_out` below
/// (`Combined`/`Asynch`) — a real bidirectional bus, not a special case:
/// the compiler already unions any two pins that coincide, so a node
/// declaring one of its own points as both an input and an output "just
/// works" (whoever else is on that net sees `Ram`'s driven read value;
/// `Ram`'s own `data_in` in turn senses whatever's on the net — itself
/// included, which is harmless, `combine(x, x) == x`). Matches `Gate::
/// Ram`'s expected input order (`addr`, `cs`, `oe`, `clr`, `clk`, then
/// `data`/`we`+`din`).
fn ram_geometry(bus: RamBus) -> Geometry {
    let mut inputs = vec![(0, 0), (0, -2), (0, -4), (0, -6), (0, -8)];
    if bus == RamBus::Separate {
        inputs.push((0, -10)); // we
        inputs.push((0, -12)); // din
    } else {
        inputs.push((2, 0)); // data_in, deliberately == data_out below
    }
    Geometry { inputs, outputs: vec![(2, 0)] }
}

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    let result = match type_ {
        "core:Clock" => (|| {
            let high = clock_duration_attr(circuit, comp, "highDuration")?;
            let low = clock_duration_attr(circuit, comp, "lowDuration")?;
            Ok((TemplateNode::Clock { high, low }, source_geometry()))
        })(),
        "core:Register" => (|| {
            let bits = width_attr(circuit, comp)?;
            let trigger = trigger_attr(circuit, comp)?;
            Ok((TemplateNode::Register { bits, trigger }, register_geometry()))
        })(),
        "core:Rom" => (|| {
            let addr_bits = addr_width_attr(circuit, comp)?;
            let data_bits = data_width_attr(circuit, comp)?;
            let contents = rom_contents_attr(comp, addr_bits, data_bits);
            Ok((TemplateNode::Rom { addr_bits, data_bits, contents }, rom_geometry()))
        })(),
        "core:Ram" => (|| {
            let addr_bits = addr_width_attr(circuit, comp)?;
            let data_bits = data_width_attr(circuit, comp)?;
            let bus = ram_bus_attr(circuit, comp)?;
            Ok((TemplateNode::Ram { addr_bits, data_bits, bus }, ram_geometry(bus)))
        })(),
        _ => return None,
    };
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::super::test_support::*;
    use super::*;
    use crate::compile::compile;
    use crate::file_format::{Circuit, ComponentInstance};
    use crate::netlist::flatten;
    use crate::sim::Simulation;
    use plugin_abi::{Bit, Value};
    use serde_json::json;
    use std::collections::BTreeMap;

    #[test]
    fn rejects_zero_duration_clock() {
        let mut attrs = BTreeMap::new();
        attrs.insert("highDuration".to_string(), json!(0));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("clk", "core:Clock", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidClockDuration {
                circuit: "main".to_string(),
                id: "clk".to_string(),
                field: "highDuration",
                value: 0,
            }
        );
    }

    #[test]
    fn rejects_unknown_trigger_option() {
        let mut attrs = BTreeMap::new();
        attrs.insert("trigger".to_string(), json!("on_edge"));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("r", "core:Register", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidTrigger { circuit: "main".to_string(), id: "r".to_string(), value: "on_edge".to_string() }
        );
    }

    /// A `Clock` driving a `Register`'s CK end to end through compile ->
    /// flatten -> `Simulation::tick()` -> settle — the actual point of
    /// "one shared tick counter" (PLAN.md §14): `sim.tick()` is called on
    /// the `Simulation`, never on the clock gate directly, and the
    /// register still sees the right rising edges through it.
    ///
    /// Layout (see `register_geometry`/`source_geometry` offsets):
    /// register "r" at (10,0) -> D=(10,0), CK=(11,2), CLR=(12,2), EN=(13,2),
    /// Q=(14,0). `d` (Constant, coincides with D), `clk` (Clock, coincides
    /// with CK) placed to land exactly on those points; CLR/EN left
    /// floating on purpose (Unknown — exercises the same "undriven EN
    /// still enables" default as the unit test, now through the compiler).
    #[test]
    fn clock_drives_a_register_through_simulation_tick() {
        let mut const_attrs = BTreeMap::new();
        const_attrs.insert("width".to_string(), json!(3));
        const_attrs.insert("value".to_string(), json!(0b101));
        let mut reg_attrs = BTreeMap::new();
        reg_attrs.insert("width".to_string(), json!(3));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: const_attrs, ..comp("d", "core:Constant", 10, 0) },
                ComponentInstance { attrs: reg_attrs, ..comp("r", "core:Register", 10, 0) },
                comp("clk", "core:Clock", 11, 2), // default high=low=1 -> period 2
                ComponentInstance {
                    attrs: { let mut a = BTreeMap::new(); a.insert("width".to_string(), json!(3)); a },
                    ..comp("out", "core:OutputPin", 14, 0)
                },
            ],
            wires: vec![],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Zero, Bit::Zero, Bit::Zero], "nothing latched before the first tick");

        sim.tick(); // global tick 1: 1%2=1, not < low(1) -> high phase -> clock rises 0->1
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::One, Bit::Zero, Bit::One], "rising edge latches D=0b101");

        sim.tick(); // global tick 2: 2%2=0 < 1 -> low phase -> clock falls 1->0
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::One, Bit::Zero, Bit::One], "falling edge must not relatch");
    }

    /// `addr`/`cs` -> `Rom`'s two inputs, `out` reads `data` — placed to
    /// coincide exactly with `rom_geometry`'s offsets relative to the ROM
    /// at `(0,0)`. `contents` preloaded via the attribute, verifying the
    /// whole compile -> flatten -> simulate path reads real (not just
    /// zeroed) data.
    #[test]
    fn compiles_and_simulates_a_preloaded_rom() {
        let mut w2 = BTreeMap::new();
        w2.insert("addrWidth".to_string(), json!(2));
        w2.insert("dataWidth".to_string(), json!(4));
        w2.insert("contents".to_string(), json!([0x1, 0xA, 0x3, 0xF]));
        let mut w4 = BTreeMap::new();
        w4.insert("width".to_string(), json!(4));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: { let mut a = BTreeMap::new(); a.insert("width".to_string(), json!(2)); a }, ..comp("addr", "core:InputPin", 0, 0) },
                comp("cs", "core:InputPin", 0, -2),
                ComponentInstance { attrs: w2, ..comp("rom", "core:Rom", 0, 0) },
                ComponentInstance { attrs: w4, ..comp("out", "core:OutputPin", 2, 0) },
            ],
            wires: vec![wire("w1", [0, 0], [0, 0]), wire("w2", [0, -2], [0, -2]), wire("w3", [2, 0], [2, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(1))).unwrap(); // addr = 1
        sim.invoke(1, "on", None).unwrap(); // cs = 1
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One], "contents[1] = 0xA");
    }

    /// Combined-bus `Ram`: `addr`/`cs`/`oe`/`clr`/`clk` driven directly;
    /// the write source is a `ControlledBuffer` (gated by `write_en`)
    /// feeding `ram_geometry`'s shared `(2,0)` point through a real wire —
    /// not a plain `InputPin` left permanently on the bus, which would
    /// short against `Ram`'s own read drive once `oe` goes high (a plain
    /// `InputPin` can't be made to stop driving; a real Logisim circuit
    /// gates a combined-bus write source the same way). This is the "one
    /// point, two directions" bus `ram_geometry`'s doc comment describes,
    /// now exercised through a real wire/net rather than direct `Gate::
    /// eval` calls (`components/memory.rs`'s unit tests already cover the
    /// eval logic itself; this one is about the *compiler* wiring the
    /// bidirectional point correctly end to end).
    #[test]
    fn compiles_and_simulates_a_combined_bus_ram_round_trip() {
        let mut ram_attrs = BTreeMap::new();
        ram_attrs.insert("addrWidth".to_string(), json!(2));
        ram_attrs.insert("dataWidth".to_string(), json!(4));
        let mut w4 = BTreeMap::new();
        w4.insert("width".to_string(), json!(4));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: { let mut a = BTreeMap::new(); a.insert("width".to_string(), json!(2)); a }, ..comp("addr", "core:InputPin", 0, 0) },
                comp("cs", "core:InputPin", 0, -2),
                comp("oe", "core:InputPin", 0, -4),
                comp("clr", "core:InputPin", 0, -6),
                comp("clk", "core:InputPin", 0, -8),
                ComponentInstance { attrs: ram_attrs, ..comp("ram", "core:Ram", 0, 0) },
                // data_src/write_en land exactly on cb's own input pins
                // (`controlled_buffer_geometry`: data at (0,0), enable at
                // (1,-2) relative to cb's own origin, here (50,0)).
                ComponentInstance { attrs: w4.clone(), ..comp("data_src", "core:InputPin", 50, 0) },
                comp("write_en", "core:InputPin", 51, -2),
                ComponentInstance { attrs: w4.clone(), ..comp("cb", "core:ControlledBuffer", 50, 0) },
                ComponentInstance { attrs: w4, ..comp("dout", "core:OutputPin", 2, 0) },
            ],
            // cb's output (50+2, 0) = (52,0) isn't naturally coincident
            // with ram's shared point (2,0) — bridged explicitly, unlike
            // every other pin pair here (which coincide by construction
            // and need no wire at all).
            wires: vec![wire("w_bus", [52, 0], [2, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        // Node order: addr=0, cs=1, oe=2, clr=3, clk=4, ram=5, data_src=6, write_en=7, cb=8, dout=9.

        sim.invoke(0, "set", Some(Value::Int(2))).unwrap(); // addr = 2
        sim.invoke(1, "on", None).unwrap(); // cs = 1
        sim.invoke(6, "set", Some(Value::Int(0b1010))).unwrap(); // data to write
        sim.invoke(7, "on", None).unwrap(); // write_en: cb now drives the bus
        sim.run_to_quiescence(); // clk still 0: priming, no edge yet

        sim.invoke(4, "on", None).unwrap(); // clk 0 -> 1: writes 0b1010 into contents[2]
        sim.run_to_quiescence();

        sim.invoke(7, "off", None).unwrap(); // cb floats: stop driving the bus
        sim.invoke(2, "on", None).unwrap(); // oe = 1: ram switches to read mode
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 9), vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One], "reads back what was written");
    }
}
