//! Compiles the combinational gates: And/Or/Nand/Nor/Xor/Xnor/Not/Buffer/
//! ControlledBuffer.

use super::{unary_geometry, width_attr, CompileError, Geometry};
use crate::file_format::{Circuit, ComponentInstance};
use crate::netlist::TemplateNode;

/// N-input, single-output leaf shape (And/Or/Nand/Nor/Xor/Xnor): inputs
/// stacked symmetrically around the y axis (2 inputs -> y = -1, 1, matching
/// the original fixed 2-input geometry exactly), output at a fixed point
/// east. Nothing renders this yet, so the exact spacing is arbitrary —
/// only "doesn't coincide with anything else" matters.
fn variadic_gate_geometry(inputs: usize) -> Geometry {
    let n = inputs as i32;
    Geometry {
        inputs: (0..inputs).map(|i| (0, 2 * i as i32 - (n - 1))).collect(),
        outputs: vec![(3, 0)],
    }
}

/// `ControlledBuffer`: data input dead center (matching `unary_geometry`'s
/// own input point, so its width lines up the same way), enable tucked out
/// of the way at `(1, -2)`, output two units east — matches `Gate::
/// ControlledBuffer`'s expected input order (`data`, `enable`).
fn controlled_buffer_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0), (1, -2)], outputs: vec![(2, 0)] }
}

/// `attrs["inputs"]`, defaulting to 2 — rejected outright if outside 2..=32
/// (`GateAttributes.MAX_INPUTS`). Real Logisim's own UI default is 5, but
/// that's a UI convenience, not a correctness constraint; 2 is the more
/// sensible schema default for hand/generator-written JSON.
fn inputs_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<usize, CompileError> {
    let raw = comp.attrs.get("inputs").and_then(|v| v.as_i64()).unwrap_or(2);
    if (2..=32).contains(&raw) {
        Ok(raw as usize)
    } else {
        Err(CompileError::InvalidInputCount { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    let result = match type_ {
        "core:AndGate" | "core:OrGate" | "core:NandGate" | "core:NorGate" | "core:XorGate" | "core:XnorGate" => (|| {
            let bits = width_attr(circuit, comp)?;
            let inputs = inputs_attr(circuit, comp)?;
            let node = match type_ {
                "core:AndGate" => TemplateNode::And { bits, inputs },
                "core:OrGate" => TemplateNode::Or { bits, inputs },
                "core:NandGate" => TemplateNode::Nand { bits, inputs },
                "core:NorGate" => TemplateNode::Nor { bits, inputs },
                "core:XorGate" => TemplateNode::Xor { bits, inputs },
                "core:XnorGate" => TemplateNode::Xnor { bits, inputs },
                _ => unreachable!("matched above"),
            };
            Ok((node, variadic_gate_geometry(inputs)))
        })(),
        "core:NotGate" => width_attr(circuit, comp).map(|bits| (TemplateNode::Not { bits }, unary_geometry())),
        "core:Buffer" => width_attr(circuit, comp).map(|bits| (TemplateNode::Buffer { bits }, unary_geometry())),
        "core:ControlledBuffer" => {
            width_attr(circuit, comp).map(|bits| (TemplateNode::ControlledBuffer { bits }, controlled_buffer_geometry()))
        }
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

    /// a(0,-1) --wire--> and.in0(0,-1); b(0,1) --wire--> and.in1(0,1);
    /// and.out(3,0) --wire--> out(3,0). Coordinates are chosen to exactly
    /// match `variadic_gate_geometry(2)`'s offsets, at x=0 for a/b/and and
    /// x=3 for the output pin, so wires are zero-length "same point"
    /// connections — geometry only needs to line up, not be pretty.
    fn and_project() -> crate::file_format::ProjectFile {
        single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                comp("a", "core:InputPin", 0, -1),
                comp("b", "core:InputPin", 0, 1),
                comp("g", "core:AndGate", 0, 0),
                comp("out", "core:OutputPin", 3, 0),
            ],
            wires: vec![
                wire("w1", [0, -1], [0, -1]),
                wire("w2", [0, 1], [0, 1]),
                wire("w3", [3, 0], [3, 0]),
            ],
            annotations: vec![],
        })
    }

    #[test]
    fn compiles_and_simulates_a_simple_json_circuit() {
        let project = and_project();
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "on", None).unwrap(); // a
        sim.invoke(1, "on", None).unwrap(); // b
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::One);

        sim.invoke(1, "off", None).unwrap();
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::Zero);
    }

    /// `width: 4` on every component on a 4-bit bus, end to end through
    /// compile -> flatten -> simulate — not just unit-checking the
    /// attribute parser, the actual point of the "up to 32 bits" ceiling.
    #[test]
    fn compiles_and_simulates_a_multi_bit_bus() {
        let mut and_attrs = BTreeMap::new();
        and_attrs.insert("width".to_string(), json!(4));
        let pin_attrs = || {
            let mut a = BTreeMap::new();
            a.insert("width".to_string(), json!(4));
            a
        };
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: pin_attrs(), ..comp("a", "core:InputPin", 0, -1) },
                ComponentInstance { attrs: pin_attrs(), ..comp("b", "core:InputPin", 0, 1) },
                ComponentInstance { attrs: and_attrs, ..comp("g", "core:AndGate", 0, 0) },
                ComponentInstance { attrs: pin_attrs(), ..comp("out", "core:OutputPin", 3, 0) },
            ],
            wires: vec![
                wire("w1", [0, -1], [0, -1]),
                wire("w2", [0, 1], [0, 1]),
                wire("w3", [3, 0], [3, 0]),
            ],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b1100))).unwrap(); // a
        sim.invoke(1, "set", Some(Value::Int(0b1010))).unwrap(); // b
        sim.run_to_quiescence();
        // 1100 & 1010 = 1000, little-endian bit order (LSB first).
        assert_eq!(get_bits(&sim, 3), vec![Bit::Zero, Bit::Zero, Bit::Zero, Bit::One]);
    }

    /// `inputs: 3` on an OR gate — the variable-arity path, not just
    /// variable width.
    #[test]
    fn compiles_a_gate_with_more_than_two_inputs() {
        let mut or_attrs = BTreeMap::new();
        or_attrs.insert("inputs".to_string(), json!(3));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                comp("a", "core:InputPin", 0, -2),
                comp("b", "core:InputPin", 0, 0),
                comp("c", "core:InputPin", 0, 2),
                ComponentInstance { attrs: or_attrs, ..comp("g", "core:OrGate", 0, 0) },
                comp("out", "core:OutputPin", 3, 0),
            ],
            wires: vec![
                wire("w1", [0, -2], [0, -2]),
                wire("w2", [0, 0], [0, 0]),
                wire("w3", [0, 2], [0, 2]),
                wire("w4", [3, 0], [3, 0]),
            ],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 4), Bit::Zero, "all three inputs off");

        sim.invoke(2, "on", None).unwrap(); // c
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 4), Bit::One);
    }

    #[test]
    fn rejects_fewer_than_two_inputs_on_a_gate() {
        let mut attrs = BTreeMap::new();
        attrs.insert("inputs".to_string(), json!(1));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("g", "core:AndGate", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidInputCount { circuit: "main".to_string(), id: "g".to_string(), value: 1 }
        );
    }

    /// `data`/`enable` -> `ControlledBuffer`'s two inputs, `out` reads the
    /// output — placed to coincide exactly with `controlled_buffer_
    /// geometry`'s offsets relative to the buffer at `(0,0)`.
    #[test]
    fn compiles_and_simulates_a_controlled_buffer() {
        let mut w4 = BTreeMap::new();
        w4.insert("width".to_string(), json!(4));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w4.clone(), ..comp("data", "core:InputPin", 0, 0) },
                comp("en", "core:InputPin", 1, -2),
                ComponentInstance { attrs: w4.clone(), ..comp("cb", "core:ControlledBuffer", 0, 0) },
                ComponentInstance { attrs: w4, ..comp("out", "core:OutputPin", 2, 0) },
            ],
            wires: vec![wire("w1", [0, 0], [0, 0]), wire("w2", [1, -2], [1, -2]), wire("w3", [2, 0], [2, 0])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b1010))).unwrap(); // data
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Unknown; 4], "enable defaults to 0 (InputPin's own default) -> disabled/floating");

        sim.invoke(1, "on", None).unwrap(); // enable = 1
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One], "enabled -> data passes through");

        sim.invoke(1, "off", None).unwrap(); // enable = 0
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Unknown; 4], "disabled -> floating");
    }
}
