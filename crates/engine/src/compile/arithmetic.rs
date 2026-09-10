//! Compiles the arithmetic components: Adder/Subtractor/Comparator
//! (`std/arith/Adder.java`/`Subtractor.java`/`Comparator.java`).

use super::{width_attr, CompileError, Geometry};
use crate::file_format::{Circuit, ComponentInstance};
use crate::netlist::TemplateNode;

/// `Adder`/`Subtractor`'s 3 fixed inputs, in the exact order
/// `components::arithmetic::eval_arithmetic` expects them (`in0`, `in1`,
/// carry/borrow-in — see `Gate::input_count_arithmetic`'s doc): the two data
/// inputs on the west side, the single-bit carry/borrow-in to the north —
/// matches `Adder.java`/`Subtractor.java`'s own port layout in spirit, not
/// pixel-for-pixel (nothing renders this yet). Outputs: sum/diff east,
/// carry/borrow-out south.
fn adder_geometry() -> Geometry {
    Geometry {
        inputs: vec![(0, -1), (0, 1), (1, -2)], // in0, in1, c_in/b_in
        outputs: vec![(3, 0), (1, 2)],          // sum/diff, c_out/b_out
    }
}

/// `Comparator`'s 2 inputs (west) and 3 outputs (east: gt, eq, lt — the
/// exact order `eval_arithmetic` returns them in).
fn comparator_geometry() -> Geometry {
    Geometry {
        inputs: vec![(0, -1), (0, 1)],
        outputs: vec![(3, -1), (3, 0), (3, 1)],
    }
}

/// `attrs["mode"]` for `core:Comparator` — `Comparator.java`'s own
/// `MODE_ATTRIBUTE` option strings (`"twosComplement"`/`"unsigned"`),
/// defaulting to `"twosComplement"` (`SIGNED_OPTION`, the Java default).
fn signed_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<bool, CompileError> {
    let raw = comp.attrs.get("mode").and_then(|v| v.as_str()).unwrap_or("twosComplement");
    match raw {
        "twosComplement" => Ok(true),
        "unsigned" => Ok(false),
        other => Err(CompileError::InvalidComparatorMode { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    let result = match type_ {
        "core:Adder" => width_attr(circuit, comp).map(|bits| (TemplateNode::Adder { bits }, adder_geometry())),
        "core:Subtractor" => width_attr(circuit, comp).map(|bits| (TemplateNode::Subtractor { bits }, adder_geometry())),
        "core:Comparator" => (|| {
            let bits = width_attr(circuit, comp)?;
            let signed = signed_attr(circuit, comp)?;
            Ok((TemplateNode::Comparator { bits, signed }, comparator_geometry()))
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

    /// `a`/`b` -> Adder's `in0`/`in1`, `cin` -> `c_in`, `sum`/`cout` read
    /// the two outputs — all placed to coincide exactly with
    /// `adder_geometry`'s offsets relative to the adder at `(0,0)`.
    #[test]
    fn compiles_and_simulates_an_adder() {
        let mut w4 = BTreeMap::new();
        w4.insert("width".to_string(), json!(4));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w4.clone(), ..comp("a", "core:InputPin", 0, -1) },
                ComponentInstance { attrs: w4.clone(), ..comp("b", "core:InputPin", 0, 1) },
                comp("cin", "core:InputPin", 1, -2),
                ComponentInstance { attrs: w4.clone(), ..comp("add", "core:Adder", 0, 0) },
                ComponentInstance { attrs: w4, ..comp("sum", "core:OutputPin", 3, 0) },
                comp("cout", "core:OutputPin", 1, 2),
            ],
            wires: vec![
                wire("w1", [0, -1], [0, -1]),
                wire("w2", [0, 1], [0, 1]),
                wire("w3", [1, -2], [1, -2]),
                wire("w4", [3, 0], [3, 0]),
                wire("w5", [1, 2], [1, 2]),
            ],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b1100))).unwrap(); // a = 12
        sim.invoke(1, "set", Some(Value::Int(0b0101))).unwrap(); // b = 5
        sim.run_to_quiescence();
        // 12 + 5 = 17, wraps to 1 (0001) in 4 bits with carry out.
        assert_eq!(get_bits(&sim, 4), vec![Bit::One, Bit::Zero, Bit::Zero, Bit::Zero], "sum wraps to 1");
        assert_eq!(get_bit(&sim, 5), Bit::One, "carry out");
    }

    #[test]
    fn compiles_and_simulates_a_subtractor() {
        let mut w4 = BTreeMap::new();
        w4.insert("width".to_string(), json!(4));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w4.clone(), ..comp("a", "core:InputPin", 0, -1) },
                ComponentInstance { attrs: w4.clone(), ..comp("b", "core:InputPin", 0, 1) },
                ComponentInstance { attrs: w4.clone(), ..comp("sub", "core:Subtractor", 0, 0) },
                ComponentInstance { attrs: w4, ..comp("diff", "core:OutputPin", 3, 0) },
                comp("bout", "core:OutputPin", 1, 2),
            ],
            wires: vec![wire("w1", [0, -1], [0, -1]), wire("w2", [0, 1], [0, 1]), wire("w3", [3, 0], [3, 0]), wire("w4", [1, 2], [1, 2])],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(5))).unwrap();
        sim.invoke(1, "set", Some(Value::Int(3))).unwrap();
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Zero, Bit::One, Bit::Zero, Bit::Zero], "5 - 3 = 2");
        assert_eq!(get_bit(&sim, 4), Bit::Zero, "no underflow, no borrow out");
    }

    /// Default `mode` (no attr set at all) must behave as
    /// `"twosComplement"` — `Comparator.java`'s own default.
    #[test]
    fn comparator_defaults_to_signed_mode() {
        let mut w4 = BTreeMap::new();
        w4.insert("width".to_string(), json!(4));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w4.clone(), ..comp("a", "core:InputPin", 0, -1) },
                ComponentInstance { attrs: w4.clone(), ..comp("b", "core:InputPin", 0, 1) },
                ComponentInstance { attrs: w4, ..comp("cmp", "core:Comparator", 0, 0) },
                comp("gt", "core:OutputPin", 3, -1),
                comp("eq", "core:OutputPin", 3, 0),
                comp("lt", "core:OutputPin", 3, 1),
            ],
            wires: vec![
                wire("w1", [0, -1], [0, -1]),
                wire("w2", [0, 1], [0, 1]),
                wire("w3", [3, -1], [3, -1]),
                wire("w4", [3, 0], [3, 0]),
                wire("w5", [3, 1], [3, 1]),
            ],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b1000))).unwrap(); // -8 signed
        sim.invoke(1, "set", Some(Value::Int(0b0001))).unwrap(); // 1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::Zero, "gt");
        assert_eq!(get_bit(&sim, 4), Bit::Zero, "eq");
        assert_eq!(get_bit(&sim, 5), Bit::One, "lt: -8 < 1 under the default signed mode");
    }

    #[test]
    fn rejects_unknown_comparator_mode() {
        let mut attrs = BTreeMap::new();
        attrs.insert("mode".to_string(), json!("bogus"));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("cmp", "core:Comparator", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidComparatorMode { circuit: "main".to_string(), id: "cmp".to_string(), value: "bogus".to_string() }
        );
    }
}
