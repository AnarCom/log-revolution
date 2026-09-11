//! Compiles the sources/sinks/pull resistor: Constant/InputPin/OutputPin/
//! PullResistor/Ground/Power/LED/Button/BitExtender/HexDigit/Probe/
//! Transistor.
//!
//! `Ground`/`Power` aren't their own runtime node kind — both are exactly a
//! fixed-value source (`Ground.propagate`/`Power.propagate`, verified: each
//! is `state.setPort(0, Value.repeat(FALSE/TRUE, width), 1)`, the same
//! shape as `Constant.propagate`'s `setPort(0, Value.createKnown(width,
//! value), 1)`), so they compile straight to `TemplateNode::Constant` with
//! `value` fixed at `0`/all-ones instead of read from an attribute.
//!
//! `LED`/`Button` (`std/io/Led.java`/`Button.java`) aren't new node kinds
//! either: `Led.propagate` just stores whatever its one 1-bit input reads
//! (a `Logger`/painter can see it later) — exactly `OutputPin`'s own
//! `eval_wiring` behavior, fixed at 1 bit (real `LED` has no `width`
//! attribute at all). `Button.propagate` reads `state.setPort(0, val, 1)`
//! from internal "is it currently pressed" data that a `Poker` flips
//! `TRUE`/`FALSE` on press/release — exactly `InputPin { bits: 1 }`'s
//! existing `on`/`off` actions (verified: both default to `Value.FALSE`
//! absent any interaction, same as `InputPin`'s own zeroed initial value).
//!
//! `Probe` (`std/wiring/Probe.java`) is the same shape again: `propagate`
//! just remembers whatever its single input port reads, for display, no
//! side effect on the rest of the circuit — verbatim `OutputPin`'s own
//! `eval_wiring`. The one real difference from `LED` is width: Java's
//! `Probe` port is `BitWidth.UNKNOWN` and the instance grows/shrinks to
//! match whatever net it lands on (`propagate` calls `recomputeBounds`
//! when the width changes) — true per-net width inference. This engine
//! doesn't do net-width inference anywhere yet (the same known gap already
//! flagged for `PullResistor` in PLAN.md §14), so `Probe` takes an
//! explicit `"width"` attribute instead, same as `OutputPin`, rather than
//! inventing a one-off inference path for this one component.

use super::{sink_geometry, source_geometry, width_attr, CompileError, Geometry};
use crate::components::ExtendMode;
use crate::file_format::{Circuit, ComponentInstance};
use crate::netlist::TemplateNode;
use plugin_abi::Bit;

fn pull_target(circuit: &Circuit, comp: &ComponentInstance) -> Result<Bit, CompileError> {
    let raw = comp.attrs.get("pull").and_then(|v| v.as_str()).unwrap_or("0");
    match raw {
        "0" => Ok(Bit::Zero),
        "1" => Ok(Bit::One),
        "X" => Ok(Bit::Error),
        other => Err(CompileError::InvalidPullTarget {
            circuit: circuit.name.clone(),
            id: comp.id.clone(),
            value: other.to_string(),
        }),
    }
}

/// `attrs["value"]` for `core:Constant` — Logisim stores it as a hex
/// integer (`Attributes.forHexInteger`); JSON numbers cover that range
/// directly, so no separate string-parsing path is needed here.
fn value_attr(comp: &ComponentInstance) -> u32 {
    comp.attrs.get("value").and_then(|v| v.as_i64()).unwrap_or(0) as u32
}

/// All `bits` low bits set — `Power`'s fixed value. Written to avoid `1u32
/// << 32` overflow at the top of `width_attr`'s own 1..=32 range.
fn all_ones(bits: u8) -> u32 {
    if bits == 32 {
        u32::MAX
    } else {
        (1u32 << bits) - 1
    }
}

/// `attrs["in_width"]`/`attrs["out_width"]` for `core:BitExtender` —
/// `BitExtender.java`'s own attribute names (`ATTR_IN_WIDTH`/
/// `ATTR_OUT_WIDTH`) and defaults (8/16, not the usual `width_attr`
/// default of 1).
fn in_width_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<u8, CompileError> {
    let raw = comp.attrs.get("in_width").and_then(|v| v.as_i64()).unwrap_or(8);
    if (1..=32).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidWidth { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

fn out_width_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<u8, CompileError> {
    let raw = comp.attrs.get("out_width").and_then(|v| v.as_i64()).unwrap_or(16);
    if (1..=32).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidWidth { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

fn extend_mode_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<ExtendMode, CompileError> {
    let raw = comp.attrs.get("type").and_then(|v| v.as_str()).unwrap_or("zero");
    match raw {
        "zero" => Ok(ExtendMode::Zero),
        "one" => Ok(ExtendMode::One),
        "sign" => Ok(ExtendMode::Sign),
        "input" => Ok(ExtendMode::Input),
        other => Err(CompileError::InvalidExtendType { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

/// `in` dead center, optional `extend` control pin (only `mode == Input`)
/// tucked at `(0, -2)`, `out` two units east — matches `Gate::
/// BitExtender`'s expected input order (`in`, `extend?`).
fn bit_extender_geometry(has_extend_pin: bool) -> Geometry {
    let mut inputs = vec![(0, 0)];
    if has_extend_pin {
        inputs.push((0, -2));
    }
    Geometry { inputs, outputs: vec![(2, 0)] }
}

/// `digit` dead center, `dot` tucked at `(0, -2)` — matches `Gate::
/// HexDigit`'s expected input order; no outputs, a pure sink.
fn hex_digit_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0), (0, -2)], outputs: vec![] }
}

/// `attrs["type"]` for `core:Transistor` — `Transistor.java`'s own
/// `ATTR_TYPE` (`"p"`/`"n"`), returned already resolved to the `Bit` a
/// conducting gate must equal (`Value.FALSE` for P-type, `Value.TRUE` for
/// N-type, verified in `computeOutput`) rather than as a separate enum —
/// nothing else in the engine needs to distinguish P/N beyond that one
/// bit. Defaults to `"p"`, matching `TYPE_P` in the Java constructor's own
/// attribute defaults.
fn conducts_on_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<Bit, CompileError> {
    let raw = comp.attrs.get("type").and_then(|v| v.as_str()).unwrap_or("p");
    match raw {
        "p" => Ok(Bit::Zero),
        "n" => Ok(Bit::One),
        other => Err(CompileError::InvalidTransistorType { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

/// `input` dead center, `gate` tucked at `(0, -2)`, `output` two units east
/// — matches `Gate::Transistor`'s expected input order (`input`, `gate`).
fn transistor_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0), (0, -2)], outputs: vec![(2, 0)] }
}

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    Some(match type_ {
        "core:Constant" => {
            width_attr(circuit, comp).map(|bits| (TemplateNode::Constant { bits, value: value_attr(comp) }, source_geometry()))
        }
        "core:Ground" => width_attr(circuit, comp).map(|bits| (TemplateNode::Constant { bits, value: 0 }, source_geometry())),
        "core:Power" => width_attr(circuit, comp).map(|bits| (TemplateNode::Constant { bits, value: all_ones(bits) }, source_geometry())),
        "core:InputPin" => width_attr(circuit, comp).map(|bits| (TemplateNode::InputPin { bits }, source_geometry())),
        "core:OutputPin" => width_attr(circuit, comp).map(|bits| (TemplateNode::OutputPin { bits }, sink_geometry())),
        "core:PullResistor" => pull_target(circuit, comp).map(|to| (TemplateNode::PullResistor(to), source_geometry())),
        "core:Led" => Ok((TemplateNode::OutputPin { bits: 1 }, sink_geometry())),
        "core:Probe" => width_attr(circuit, comp).map(|bits| (TemplateNode::OutputPin { bits }, sink_geometry())),
        "core:Button" => Ok((TemplateNode::InputPin { bits: 1 }, source_geometry())),
        "core:BitExtender" => (|| {
            let in_bits = in_width_attr(circuit, comp)?;
            let out_bits = out_width_attr(circuit, comp)?;
            let mode = extend_mode_attr(circuit, comp)?;
            Ok((TemplateNode::BitExtender { in_bits, out_bits, mode }, bit_extender_geometry(mode == ExtendMode::Input)))
        })(),
        "core:HexDigit" => Ok((TemplateNode::HexDigit, hex_digit_geometry())),
        "core:Transistor" => (|| {
            let bits = width_attr(circuit, comp)?;
            let conducts_on = conducts_on_attr(circuit, comp)?;
            Ok((TemplateNode::Transistor { bits, conducts_on }, transistor_geometry()))
        })(),
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::super::test_support::*;
    use crate::compile::compile;
    use crate::file_format::{Circuit, ComponentInstance};
    use crate::netlist::flatten;
    use crate::netlist::TemplateNode as Node;
    use crate::sim::Simulation;
    use plugin_abi::{Bit, Value};
    use serde_json::json;
    use std::collections::BTreeMap;

    #[test]
    fn pull_resistor_reads_pull_attr_matching_logisim_options() {
        let mut attrs = BTreeMap::new();
        attrs.insert("pull".to_string(), json!("X"));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("r", "core:PullResistor", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let template = &library["main"];
        assert!(matches!(template.nodes[0], Node::PullResistor(Bit::Error)));
    }

    /// `Constant` feeding a `Buffer` feeding an `OutputPin` — mainly
    /// exercises `Constant`'s `width`/`value` attrs end to end; `Buffer`
    /// (a `logic` gate) is just the delayed-identity pass-through in the
    /// middle, not itself the point of this test.
    #[test]
    fn constant_compiles_and_simulates_its_fixed_value_through_a_buffer() {
        let mut const_attrs = BTreeMap::new();
        const_attrs.insert("width".to_string(), json!(3));
        const_attrs.insert("value".to_string(), json!(0b101));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: const_attrs, ..comp("k", "core:Constant", 0, 0) },
                ComponentInstance {
                    attrs: { let mut a = BTreeMap::new(); a.insert("width".to_string(), json!(3)); a },
                    ..comp("buf", "core:Buffer", 2, 0)
                },
                ComponentInstance {
                    attrs: { let mut a = BTreeMap::new(); a.insert("width".to_string(), json!(3)); a },
                    ..comp("out", "core:OutputPin", 4, 0)
                },
            ],
            wires: vec![wire("w1", [0, 0], [2, 0]), wire("w2", [4, 0], [4, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 2), vec![Bit::One, Bit::Zero, Bit::One]); // 0b101 LSB-first
    }

    /// `Ground`/`Power` compile straight to `TemplateNode::Constant` (see
    /// this module's doc comment) — both ends of a 3-bit bus, verified
    /// through a live simulation rather than just inspecting the compiled
    /// node.
    #[test]
    fn ground_and_power_drive_all_zero_and_all_one_at_their_configured_width() {
        let mut w3 = BTreeMap::new();
        w3.insert("width".to_string(), json!(3));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w3.clone(), ..comp("g", "core:Ground", 0, 0) },
                ComponentInstance { attrs: w3.clone(), ..comp("gout", "core:OutputPin", 3, 0) },
                ComponentInstance { attrs: w3.clone(), ..comp("p", "core:Power", 0, 10) },
                ComponentInstance { attrs: w3, ..comp("pout", "core:OutputPin", 3, 10) },
            ],
            wires: vec![wire("w1", [0, 0], [3, 0]), wire("w2", [0, 10], [3, 10])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 1), vec![Bit::Zero, Bit::Zero, Bit::Zero], "Ground");
        assert_eq!(get_bits(&sim, 3), vec![Bit::One, Bit::One, Bit::One], "Power");
    }

    /// `Power` at the full `MAX_WIDTH` boundary — guards `all_ones` against
    /// the `1u32 << 32` overflow a naive mask formula would hit there.
    #[test]
    fn power_at_width_thirty_two_does_not_overflow_the_mask() {
        let mut w32 = BTreeMap::new();
        w32.insert("width".to_string(), json!(32));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w32.clone(), ..comp("p", "core:Power", 0, 0) },
                ComponentInstance { attrs: w32, ..comp("out", "core:OutputPin", 3, 0) },
            ],
            wires: vec![wire("w1", [0, 0], [3, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 1), vec![Bit::One; 32]);
    }

    /// `LED` compiles straight to `TemplateNode::OutputPin { bits: 1 }`
    /// (see this module's doc comment) — reads its own `"get"` readout
    /// directly, same as any `OutputPin`.
    #[test]
    fn led_reflects_its_driven_input() {
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![comp("in", "core:InputPin", 0, 0), comp("led", "core:Led", 3, 0)],
            wires: vec![wire("w1", [0, 0], [3, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 1), Bit::Zero);

        sim.invoke(0, "on", None).unwrap();
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 1), Bit::One);
    }

    /// `Button` compiles straight to `TemplateNode::InputPin { bits: 1 }`
    /// (see this module's doc comment) — driven through an `OutputPin` to
    /// prove it behaves as a real source, not just inspecting the node.
    #[test]
    fn button_acts_as_a_momentary_one_bit_source() {
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![comp("btn", "core:Button", 0, 0), comp("out", "core:OutputPin", 3, 0)],
            wires: vec![wire("w1", [0, 0], [3, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 1), Bit::Zero, "unpressed defaults to Zero, same as Button.propagate's Value.FALSE default");

        sim.invoke(0, "on", None).unwrap(); // press
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 1), Bit::One);

        sim.invoke(0, "off", None).unwrap(); // release
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 1), Bit::Zero);
    }

    /// `Probe` compiles straight to `TemplateNode::OutputPin { bits }`
    /// (see this module's doc comment), width taken from its own `"width"`
    /// attribute rather than hardcoded like `Led` — exercised at 3 bits so
    /// this actually differs from `led_reflects_its_driven_input` above.
    #[test]
    fn probe_reflects_a_multi_bit_driven_input() {
        let mut w3 = BTreeMap::new();
        w3.insert("width".to_string(), json!(3));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w3.clone(), ..comp("in", "core:InputPin", 0, 0) },
                ComponentInstance { attrs: w3, ..comp("probe", "core:Probe", 3, 0) },
            ],
            wires: vec![wire("w1", [0, 0], [3, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 1), vec![Bit::Zero, Bit::Zero, Bit::Zero]);

        sim.invoke(0, "set", Some(Value::Int(0b101))).unwrap();
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 1), vec![Bit::One, Bit::Zero, Bit::One]);
    }

    /// `Transistor` compiles to `TemplateNode::Transistor { bits,
    /// conducts_on }` — a live simulation through a P-type transistor
    /// (`"type"` defaults to `"p"`, so left unset here on purpose to
    /// exercise the default), gated by an `InputPin`.
    #[test]
    fn compiles_and_simulates_a_p_type_transistor_gated_by_an_input() {
        let mut w2 = BTreeMap::new();
        w2.insert("width".to_string(), json!(2));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w2.clone(), ..comp("data", "core:InputPin", 0, 0) },
                comp("gate", "core:InputPin", 0, -2),
                ComponentInstance { attrs: w2.clone(), ..comp("t", "core:Transistor", 0, 0) },
                ComponentInstance { attrs: w2, ..comp("out", "core:OutputPin", 2, 0) },
            ],
            wires: vec![wire("w1", [0, 0], [0, 0]), wire("w2", [0, -2], [0, -2]), wire("w3", [2, 0], [2, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b10))).unwrap(); // data
        // gate defaults to Zero (InputPin's own zeroed initial value) ->
        // P-type conducts (`conducts_on == Zero`) without pressing anything.
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Zero, Bit::One]);

        sim.invoke(1, "on", None).unwrap(); // gate = One -> P-type stops conducting
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Unknown, Bit::Unknown]);
    }

    /// `in`/`extend`/`out` -> `BitExtender`'s two inputs and output —
    /// placed to coincide exactly with `bit_extender_geometry(true)`'s
    /// offsets relative to the extender at `(0,0)`.
    #[test]
    fn compiles_and_simulates_a_bit_extender_in_input_mode() {
        let mut in4 = BTreeMap::new();
        in4.insert("width".to_string(), json!(4));
        let mut ext_attrs = BTreeMap::new();
        ext_attrs.insert("in_width".to_string(), json!(4));
        ext_attrs.insert("out_width".to_string(), json!(8));
        ext_attrs.insert("type".to_string(), json!("input"));
        let mut out8 = BTreeMap::new();
        out8.insert("width".to_string(), json!(8));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: in4, ..comp("data", "core:InputPin", 0, 0) },
                comp("extend", "core:InputPin", 0, -2),
                ComponentInstance { attrs: ext_attrs, ..comp("ext", "core:BitExtender", 0, 0) },
                ComponentInstance { attrs: out8, ..comp("out", "core:OutputPin", 2, 0) },
            ],
            wires: vec![wire("w1", [0, 0], [0, 0]), wire("w2", [0, -2], [0, -2]), wire("w3", [2, 0], [2, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b0110))).unwrap(); // data
        sim.invoke(1, "on", None).unwrap(); // extend = 1
        sim.run_to_quiescence();
        assert_eq!(
            get_bits(&sim, 3),
            vec![Bit::Zero, Bit::One, Bit::One, Bit::Zero, Bit::One, Bit::One, Bit::One, Bit::One],
            "low 4 bits are `data`, high 4 filled with `extend`'s current value"
        );
    }

    /// `digit`/`dot` -> `HexDigit`'s two inputs, placed to coincide exactly
    /// with `hex_digit_geometry`'s offsets relative to the display at
    /// `(0,0)`; no `OutputPin` at all (`HexDigit` has no outputs) — reads
    /// its own `"get"` readout directly, same as `led_reflects_its_driven_
    /// input` above.
    #[test]
    fn compiles_and_simulates_a_hex_digit_display() {
        let mut w4 = BTreeMap::new();
        w4.insert("width".to_string(), json!(4));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w4, ..comp("digit", "core:InputPin", 0, 0) },
                comp("dot", "core:InputPin", 0, -2),
                comp("disp", "core:HexDigit", 0, 0),
            ],
            wires: vec![wire("w1", [0, 0], [0, 0]), wire("w2", [0, -2], [0, -2])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(8))).unwrap(); // digit = 8, every segment lit
        sim.invoke(1, "on", None).unwrap(); // dot = 1
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 2), vec![Bit::One; 8], "digit 8 + dot -> all 8 bits set");
    }
}
