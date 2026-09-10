//! Compiles the sources/sinks/pull resistor: Constant/InputPin/OutputPin/
//! PullResistor.

use super::{sink_geometry, source_geometry, width_attr, CompileError, Geometry};
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

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    Some(match type_ {
        "core:Constant" => {
            width_attr(circuit, comp).map(|bits| (TemplateNode::Constant { bits, value: value_attr(comp) }, source_geometry()))
        }
        "core:InputPin" => width_attr(circuit, comp).map(|bits| (TemplateNode::InputPin { bits }, source_geometry())),
        "core:OutputPin" => width_attr(circuit, comp).map(|bits| (TemplateNode::OutputPin { bits }, sink_geometry())),
        "core:PullResistor" => pull_target(circuit, comp).map(|to| (TemplateNode::PullResistor(to), source_geometry())),
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
    use plugin_abi::Bit;
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
}
