//! Compiles the multiplexer/demultiplexer/decoder/priority encoder:
//! Multiplexer/Demultiplexer/Decoder/PriorityEncoder.
//!
//! Attribute keys/defaults match `Plexers.java` verbatim: `"select"`
//! (`ATTR_SELECT`, `BitWidth` 1..=5, default 1), `"enable"` (`ATTR_ENABLE`,
//! bool, default `true`), `"disabled"` (`ATTR_DISABLED`, `"Z"`/`"0"`,
//! default `"Z"` = floating), `"tristate"` (`ATTR_TRISTATE`, Demux-only,
//! bool, default `false`).

use super::{width_attr, CompileError, Geometry};
use crate::file_format::{Circuit, ComponentInstance};
use crate::netlist::TemplateNode;

fn select_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<u8, CompileError> {
    let raw = comp.attrs.get("select").and_then(|v| v.as_i64()).unwrap_or(1);
    if (1..=5).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidSelectWidth { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

fn enable_attr(comp: &ComponentInstance) -> bool {
    comp.attrs.get("enable").and_then(|v| v.as_bool()).unwrap_or(true)
}

/// Returns whether "disabled" means output `Zero` (`"0"`) rather than the
/// default floating/`Unknown` (`"Z"`).
fn disabled_zero_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<bool, CompileError> {
    let raw = comp.attrs.get("disabled").and_then(|v| v.as_str()).unwrap_or("Z");
    match raw {
        "Z" => Ok(false),
        "0" => Ok(true),
        other => Err(CompileError::InvalidDisabledOption { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

fn tristate_attr(comp: &ComponentInstance) -> bool {
    comp.attrs.get("tristate").and_then(|v| v.as_bool()).unwrap_or(false)
}

/// `n` data inputs stacked down the west edge, select (and optional
/// enable) along the south edge, output on the east — arbitrary, matches
/// `Gate::Mux`'s expected input order (data.., select, enable?); nothing
/// renders this yet, only "distinct points" matters.
fn mux_geometry(n: usize, has_enable: bool) -> Geometry {
    let mut inputs: Vec<(i32, i32)> = (0..n).map(|i| (0, 2 * i as i32)).collect();
    inputs.push((1, -1)); // select
    if has_enable {
        inputs.push((1, -2)); // enable
    }
    Geometry { inputs, outputs: vec![(3, 0)] }
}

/// Mirror of `mux_geometry`: select (and optional enable) then the single
/// data input along the west edge, `n` outputs down the east — matches
/// `Gate::Demux`'s expected input order (select, enable?, data).
fn demux_geometry(n: usize, has_enable: bool) -> Geometry {
    let mut inputs = vec![(0, -1)]; // select
    if has_enable {
        inputs.push((0, -2)); // enable
    }
    inputs.push((0, 0)); // data
    let outputs = (0..n).map(|i| (3, 2 * i as i32)).collect();
    Geometry { inputs, outputs }
}

/// Same as `demux_geometry` minus the data input — matches `Gate::
/// Decoder`'s expected input order (select, enable?).
fn decoder_geometry(n: usize, has_enable: bool) -> Geometry {
    let mut inputs = vec![(0, -1)]; // select
    if has_enable {
        inputs.push((0, -2)); // enable
    }
    let outputs = (0..n).map(|i| (3, 2 * i as i32)).collect();
    Geometry { inputs, outputs }
}

/// `n` data lines stacked down the west edge, `enable_in` just past them —
/// matches `Gate::PriorityEncoder`'s expected input order (data.., enable);
/// the three outputs (`out`, `enable_out`, `group_signal`) on the east.
fn priority_encoder_geometry(n: usize) -> Geometry {
    let mut inputs: Vec<(i32, i32)> = (0..n).map(|i| (0, 2 * i as i32)).collect();
    inputs.push((0, 2 * n as i32)); // enable_in
    Geometry { inputs, outputs: vec![(3, 0), (3, 2), (3, 4)] }
}

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    let result = match type_ {
        "core:Multiplexer" => (|| {
            let bits = width_attr(circuit, comp)?;
            let select_bits = select_attr(circuit, comp)?;
            let has_enable = enable_attr(comp);
            let disabled_zero = disabled_zero_attr(circuit, comp)?;
            let n = 1usize << select_bits;
            Ok((TemplateNode::Mux { bits, select_bits, has_enable, disabled_zero }, mux_geometry(n, has_enable)))
        })(),
        "core:Demultiplexer" => (|| {
            let bits = width_attr(circuit, comp)?;
            let select_bits = select_attr(circuit, comp)?;
            let has_enable = enable_attr(comp);
            let disabled_zero = disabled_zero_attr(circuit, comp)?;
            let tristate = tristate_attr(comp);
            let n = 1usize << select_bits;
            Ok((TemplateNode::Demux { bits, select_bits, has_enable, disabled_zero, tristate }, demux_geometry(n, has_enable)))
        })(),
        "core:Decoder" => (|| {
            let select_bits = select_attr(circuit, comp)?;
            let has_enable = enable_attr(comp);
            let disabled_zero = disabled_zero_attr(circuit, comp)?;
            let tristate = tristate_attr(comp);
            let n = 1usize << select_bits;
            Ok((TemplateNode::Decoder { select_bits, has_enable, disabled_zero, tristate }, decoder_geometry(n, has_enable)))
        })(),
        "core:PriorityEncoder" => (|| {
            let select_bits = select_attr(circuit, comp)?;
            let disabled_zero = disabled_zero_attr(circuit, comp)?;
            let n = 1usize << select_bits;
            Ok((TemplateNode::PriorityEncoder { select_bits, disabled_zero }, priority_encoder_geometry(n)))
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
    use plugin_abi::Bit;
    use serde_json::json;
    use std::collections::BTreeMap;

    /// `a`/`b` -> Mux data inputs, `sel` -> select, output -> `out`. All
    /// placed to coincide exactly with `mux_geometry(2, true)`'s offsets
    /// relative to the mux at (0,0); enable left floating (Unknown -> still
    /// active, per `mux_undriven_enable_still_selects...` in
    /// `components/plexers.rs`).
    #[test]
    fn compiles_and_simulates_a_multiplexer() {
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                comp("a", "core:InputPin", 0, 0),
                comp("b", "core:InputPin", 0, 2),
                comp("mux", "core:Multiplexer", 0, 0),
                comp("sel", "core:InputPin", 1, -1),
                comp("out", "core:OutputPin", 3, 0),
            ],
            wires: vec![
                wire("w1", [0, 0], [0, 0]),
                wire("w2", [0, 2], [0, 2]),
                wire("w3", [1, -1], [1, -1]),
                wire("w4", [3, 0], [3, 0]),
            ],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "on", None).unwrap(); // a = 1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 4), Bit::One, "select=0 (default) -> a");

        sim.invoke(3, "on", None).unwrap(); // sel = 1 (gates: a=0, b=1, mux=2, sel=3, out=4)
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 4), Bit::Zero, "select=1 -> b (still 0)");
    }

    /// Same layout idea as the mux test, but for `demux_geometry`: `d` ->
    /// data, `sel` -> select, outputs `o0`/`o1`.
    #[test]
    fn compiles_and_simulates_a_demultiplexer() {
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                comp("sel", "core:InputPin", 0, -1),
                comp("d", "core:InputPin", 0, 0),
                comp("dmx", "core:Demultiplexer", 0, 0),
                comp("o0", "core:OutputPin", 3, 0),
                comp("o1", "core:OutputPin", 3, 2),
            ],
            wires: vec![
                wire("w1", [0, -1], [0, -1]),
                wire("w2", [0, 0], [0, 0]),
                wire("w3", [3, 0], [3, 0]),
                wire("w4", [3, 2], [3, 2]),
            ],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(1, "on", None).unwrap(); // d = 1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::One, "select=0 (default) -> o0 gets data");
        assert_eq!(get_bit(&sim, 4), Bit::Zero, "o1 idles at Zero (default disabled_zero option)");

        sim.invoke(0, "on", None).unwrap(); // sel = 1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 3), Bit::Zero, "o0 now idles");
        assert_eq!(get_bit(&sim, 4), Bit::One, "o1 gets data");
    }

    #[test]
    fn rejects_select_width_above_five() {
        let mut attrs = BTreeMap::new();
        attrs.insert("select".to_string(), json!(6));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("mux", "core:Multiplexer", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidSelectWidth { circuit: "main".to_string(), id: "mux".to_string(), value: 6 }
        );
    }

    #[test]
    fn rejects_unknown_disabled_option() {
        let mut attrs = BTreeMap::new();
        attrs.insert("disabled".to_string(), json!("float"));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("mux", "core:Multiplexer", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidDisabledOption { circuit: "main".to_string(), id: "mux".to_string(), value: "float".to_string() }
        );
    }

    #[test]
    fn multiplexer_without_enable_attribute_compiles_with_fewer_pins() {
        let mut attrs = BTreeMap::new();
        attrs.insert("enable".to_string(), json!(false));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                comp("a", "core:InputPin", 0, 0),
                comp("b", "core:InputPin", 0, 2),
                ComponentInstance { attrs, ..comp("mux", "core:Multiplexer", 0, 0) },
                comp("sel", "core:InputPin", 1, -1),
                comp("out", "core:OutputPin", 3, 0),
            ],
            wires: vec![
                wire("w1", [0, 0], [0, 0]),
                wire("w2", [0, 2], [0, 2]),
                wire("w3", [1, -1], [1, -1]),
                wire("w4", [3, 0], [3, 0]),
            ],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "on", None).unwrap(); // a = 1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 4), Bit::One, "no enable pin at all -> always active, still selects a");
    }

    /// `sel` -> Decoder's select, outputs `o0`/`o1` — placed to coincide
    /// exactly with `decoder_geometry(2, false)`'s offsets relative to the
    /// decoder at `(0,0)`. No `enable` attribute given (defaults to
    /// `true`), left unwired: an unconnected 1-bit `enable` reads `Unknown`
    /// -> still active (`enable_status`), so this also exercises that path
    /// end to end, not just via a directly-driven `InputPin` like the mux/
    /// demux tests above.
    #[test]
    fn compiles_and_simulates_a_decoder() {
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                comp("sel", "core:InputPin", 0, -1),
                comp("dec", "core:Decoder", 0, 0),
                comp("o0", "core:OutputPin", 3, 0),
                comp("o1", "core:OutputPin", 3, 2),
            ],
            wires: vec![wire("w1", [0, -1], [0, -1]), wire("w2", [3, 0], [3, 0]), wire("w3", [3, 2], [3, 2])],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 2), Bit::One, "select=0 (default) -> o0 asserted");
        assert_eq!(get_bit(&sim, 3), Bit::Zero, "o1 idles at Zero (default disabled/tristate options)");

        sim.invoke(0, "on", None).unwrap(); // sel = 1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 2), Bit::Zero, "o0 now idles");
        assert_eq!(get_bit(&sim, 3), Bit::One, "o1 asserted");
    }

    /// `i0`/`i1` -> PriorityEncoder's two data lines, `en` -> enable_in,
    /// `out`/`eout`/`gs` read the three outputs — placed to coincide
    /// exactly with `priority_encoder_geometry(2)`'s offsets relative to
    /// the encoder at `(0,0)`.
    #[test]
    fn compiles_and_simulates_a_priority_encoder() {
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                comp("i0", "core:InputPin", 0, 0),
                comp("i1", "core:InputPin", 0, 2),
                comp("en", "core:InputPin", 0, 4),
                comp("pe", "core:PriorityEncoder", 0, 0),
                comp("out", "core:OutputPin", 3, 0),
                comp("eout", "core:OutputPin", 3, 2),
                comp("gs", "core:OutputPin", 3, 4),
            ],
            wires: vec![
                wire("w1", [0, 0], [0, 0]),
                wire("w2", [0, 2], [0, 2]),
                wire("w3", [0, 4], [0, 4]),
                wire("w4", [3, 0], [3, 0]),
                wire("w5", [3, 2], [3, 2]),
                wire("w6", [3, 4], [3, 4]),
            ],
            annotations: vec![],
        });
        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(2, "on", None).unwrap(); // enable
        sim.invoke(1, "on", None).unwrap(); // i1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 4), Bit::One, "out=1");
        assert_eq!(get_bit(&sim, 5), Bit::Zero, "enable_out=0 (found something)");
        assert_eq!(get_bit(&sim, 6), Bit::One, "group_signal=1");
    }
}
