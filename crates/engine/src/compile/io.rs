//! Compiles the IO components: `DotMatrix`/`Tty`/`Keyboard`.

use super::{CompileError, Geometry};
use crate::components::{DotMatrixInput, Trigger};
use crate::file_format::{Circuit, ComponentInstance};
use crate::netlist::TemplateNode;

/// `attrs["matrixcols"]`/`attrs["matrixrows"]` for `core:DotMatrix`,
/// ranged 1..=32 (`DotMatrix.ATTR_MATRIX_COLS`/`ATTR_MATRIX_ROWS`'s own
/// range, `1, Value.MAX_WIDTH`), defaulting to `DotMatrix`'s own
/// constructor defaults (5 cols, 7 rows).
fn dot_matrix_size(circuit: &Circuit, comp: &ComponentInstance, key: &'static str, default: i64) -> Result<u8, CompileError> {
    let raw = comp.attrs.get(key).and_then(|v| v.as_i64()).unwrap_or(default);
    if (1..=32).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidDotMatrixSize { circuit: circuit.name.clone(), id: comp.id.clone(), field: key, value: raw })
    }
}

/// `attrs["inputtype"]` for `core:DotMatrix` — `DotMatrix.
/// ATTR_INPUT_TYPE`'s own three option strings, defaulting to its own
/// default (`INPUT_COLUMN`).
fn dot_matrix_input(circuit: &Circuit, comp: &ComponentInstance) -> Result<DotMatrixInput, CompileError> {
    let raw = comp.attrs.get("inputtype").and_then(|v| v.as_str()).unwrap_or("column");
    match raw {
        "column" => Ok(DotMatrixInput::Column),
        "row" => Ok(DotMatrixInput::Row),
        "select" => Ok(DotMatrixInput::Select),
        other => Err(CompileError::InvalidDotMatrixInput { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

/// `DotMatrix.updatePorts`'s `INPUT_COLUMN`/`INPUT_ROW` shape: `n` inputs
/// (one per column/row), stacked down the left edge, each `width`-wide —
/// no outputs (`DotMatrix` is a pure sink).
fn dot_matrix_line_geometry(n: u8) -> Geometry {
    Geometry { inputs: (0..n as i32).map(|i| (0, 2 * i)).collect(), outputs: vec![] }
}

/// `DotMatrix.updatePorts`'s general (`rows>1 && cols>1`) `INPUT_SELECT`
/// shape: `col_data` then `row_select` — see `Gate::DotMatrix`'s doc
/// comment for why the degenerate single-row/col case (a single port in
/// Java) isn't replicated.
fn dot_matrix_select_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0), (0, 2)], outputs: vec![] }
}

/// `attrs["cols"]`/`attrs["rows"]` for `core:Tty`, ranged like `Tty.
/// ATTR_COLUMNS`/`ATTR_ROWS` (1..=120 cols, 1..=48 rows), defaulting to
/// `Tty`'s own constructor defaults (32 cols, 8 rows).
fn tty_size(circuit: &Circuit, comp: &ComponentInstance, key: &'static str, max: i64, default: i64) -> Result<u8, CompileError> {
    let raw = comp.attrs.get(key).and_then(|v| v.as_i64()).unwrap_or(default);
    if (1..=max).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidTtyDimension { circuit: circuit.name.clone(), id: comp.id.clone(), field: key, value: raw })
    }
}

/// `Tty`'s fixed port shape: `clear`, `ck`, `we` (1-bit each), `in`
/// (7-bit) — matches `Gate::Tty`'s expected input order; no outputs.
fn tty_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0), (1, 0), (2, 0), (3, 0)], outputs: vec![] }
}

/// `attrs["buflen"]` for `core:Keyboard`, ranged 1..=256 (`Keyboard.
/// ATTR_BUFFER`'s own range), defaulting to its own default (32).
fn keyboard_capacity(circuit: &Circuit, comp: &ComponentInstance) -> Result<u16, CompileError> {
    let raw = comp.attrs.get("buflen").and_then(|v| v.as_i64()).unwrap_or(32);
    if (1..=256).contains(&raw) {
        Ok(raw as u16)
    } else {
        Err(CompileError::InvalidKeyboardBufferLength { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

/// `Keyboard`'s fixed port shape: `clear`, `ck`, `re` (1-bit each), then
/// `avl` (1-bit output), `out` (7-bit output) — matches `Gate::Keyboard`'s
/// expected input/output order.
fn keyboard_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0), (1, 0), (2, 0)], outputs: vec![(3, 0), (4, 0)] }
}

/// `attrs["trigger"]` for `core:Tty`/`core:Keyboard`/`core:Random` —
/// `StdAttr.EDGE_TRIGGER`'s own two option strings only (`"rising"`/
/// `"falling"`), unlike `Register`/`Ram`'s 4-option `StdAttr.TRIGGER` (see
/// `super::memory::trigger_attr`) — these three components' own
/// `propagate` only ever distinguishes falling from "anything else", so
/// `High`/`Low` aren't offered here at all, not just defaulted away.
/// `pub(super)` since `compile::memory` (`core:Random`) reuses this too.
pub(super) fn edge_trigger_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<Trigger, CompileError> {
    let raw = comp.attrs.get("trigger").and_then(|v| v.as_str()).unwrap_or("rising");
    match raw {
        "rising" => Ok(Trigger::Rising),
        "falling" => Ok(Trigger::Falling),
        other => Err(CompileError::InvalidTrigger { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    let result = match type_ {
        "core:DotMatrix" => (|| {
            let cols = dot_matrix_size(circuit, comp, "matrixcols", 5)?;
            let rows = dot_matrix_size(circuit, comp, "matrixrows", 7)?;
            let input = dot_matrix_input(circuit, comp)?;
            let geometry = match input {
                DotMatrixInput::Column => dot_matrix_line_geometry(cols),
                DotMatrixInput::Row => dot_matrix_line_geometry(rows),
                DotMatrixInput::Select => dot_matrix_select_geometry(),
            };
            Ok((TemplateNode::DotMatrix { rows, cols, input }, geometry))
        })(),
        "core:Tty" => (|| {
            let cols = tty_size(circuit, comp, "cols", 120, 32)?;
            let rows = tty_size(circuit, comp, "rows", 48, 8)?;
            let trigger = edge_trigger_attr(circuit, comp)?;
            Ok((TemplateNode::Tty { cols, rows, trigger }, tty_geometry()))
        })(),
        "core:Keyboard" => (|| {
            let capacity = keyboard_capacity(circuit, comp)?;
            let trigger = edge_trigger_attr(circuit, comp)?;
            Ok((TemplateNode::Keyboard { capacity, trigger }, keyboard_geometry()))
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

    /// `sw0`/`sw1`/`sw2` (`InputPin`s, 7 bits each) drive a 7x3
    /// column-mode `DotMatrix`'s three column inputs; `read_db`-style
    /// direct `sim.read` (via the `get` readout, `DotMatrix` has no output
    /// pins to wire an `OutputPin` to) confirms the whole compile ->
    /// flatten -> simulate path, not just `Gate::eval` in isolation.
    #[test]
    fn compiles_and_simulates_a_column_mode_dot_matrix() {
        let mut attrs = BTreeMap::new();
        attrs.insert("matrixrows".to_string(), json!(7));
        attrs.insert("matrixcols".to_string(), json!(3));
        attrs.insert("inputtype".to_string(), json!("column"));
        let w7 = |id: &str, x: i32, y: i32| {
            let mut a = BTreeMap::new();
            a.insert("width".to_string(), json!(7));
            ComponentInstance { attrs: a, ..comp(id, "core:InputPin", x, y) }
        };

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                w7("c0", 0, 0),
                w7("c1", 0, 2),
                w7("c2", 0, 4),
                ComponentInstance { attrs, ..comp("m", "core:DotMatrix", 0, 0) },
            ],
            wires: vec![wire("w0", [0, 0], [0, 0]), wire("w1", [0, 2], [0, 2]), wire("w2", [0, 4], [0, 4])],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        // Node order: c0=0, c1=1, c2=2, m=3.
        sim.invoke(0, "set", Some(Value::Int(0b1))).unwrap(); // column 0: only LSB (bottom row) set
        sim.run_to_quiescence();

        let Value::Bits(grid) = sim.read(3, "get").unwrap() else { panic!("expected Bits") };
        assert_eq!(grid.len(), 21);
        assert_eq!(grid[6 * 3], Bit::One, "row 6 (bottom), col 0 — LSB of column 0's input");
    }

    #[test]
    fn rejects_dot_matrix_size_above_thirty_two() {
        let mut attrs = BTreeMap::new();
        attrs.insert("matrixrows".to_string(), json!(33));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("m", "core:DotMatrix", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidDotMatrixSize { circuit: "main".to_string(), id: "m".to_string(), field: "matrixrows", value: 33 }
        );
    }

    #[test]
    fn rejects_unknown_dot_matrix_input_type() {
        let mut attrs = BTreeMap::new();
        attrs.insert("inputtype".to_string(), json!("diagonal"));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("m", "core:DotMatrix", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidDotMatrixInput { circuit: "main".to_string(), id: "m".to_string(), value: "diagonal".to_string() }
        );
    }

    /// A `Tty` fed "A" through a clocked write end to end (compile ->
    /// flatten -> simulate); readout is the dynamically-named `row0` (the
    /// in-progress row, nothing committed yet).
    #[test]
    fn compiles_and_simulates_a_tty_receiving_a_character() {
        let mut w1 = BTreeMap::new();
        w1.insert("width".to_string(), json!(1));
        let mut w7 = BTreeMap::new();
        w7.insert("width".to_string(), json!(7));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w1.clone(), ..comp("clr", "core:InputPin", 0, 0) },
                ComponentInstance { attrs: w1.clone(), ..comp("ck", "core:InputPin", 1, 0) },
                ComponentInstance { attrs: w1, ..comp("we", "core:InputPin", 2, 0) },
                ComponentInstance { attrs: w7, ..comp("in", "core:InputPin", 3, 0) },
                comp("tty", "core:Tty", 0, 0),
            ],
            wires: vec![wire("w0", [0, 0], [0, 0]), wire("w1", [1, 0], [1, 0]), wire("w2", [2, 0], [2, 0]), wire("w3", [3, 0], [3, 0])],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        // Node order: clr=0, ck=1, we=2, in=3, tty=4.
        sim.invoke(2, "on", None).unwrap(); // we = 1
        sim.invoke(3, "set", Some(Value::Int('A' as i64))).unwrap();
        sim.run_to_quiescence();
        sim.invoke(1, "on", None).unwrap(); // ck: 0 -> 1, rising edge
        sim.run_to_quiescence();

        assert_eq!(sim.read(4, "row0").unwrap(), Value::Bits(vec![Bit::One, Bit::Zero, Bit::Zero, Bit::Zero, Bit::Zero, Bit::Zero, Bit::One]));
    }

    /// `Keyboard`'s `AVL`/`OUT` wired to real `OutputPin`s — the *external*
    /// input (`invoke("key", ...)`) still has to reach through
    /// `Simulation::invoke` by the compiled node's global index.
    #[test]
    fn compiles_and_simulates_a_keyboard_key_reaching_the_output_pins() {
        let mut w1 = BTreeMap::new();
        w1.insert("width".to_string(), json!(1));
        let mut w7 = BTreeMap::new();
        w7.insert("width".to_string(), json!(7));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w1.clone(), ..comp("clr", "core:InputPin", 0, 0) },
                ComponentInstance { attrs: w1.clone(), ..comp("ck", "core:InputPin", 1, 0) },
                ComponentInstance { attrs: w1.clone(), ..comp("re", "core:InputPin", 2, 0) },
                comp("kb", "core:Keyboard", 0, 0),
                ComponentInstance { attrs: w1, ..comp("avl", "core:OutputPin", 3, 0) },
                ComponentInstance { attrs: w7, ..comp("out", "core:OutputPin", 4, 0) },
            ],
            wires: vec![wire("w0", [0, 0], [0, 0]), wire("w1", [1, 0], [1, 0]), wire("w2", [2, 0], [2, 0])],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        // Node order: clr=0, ck=1, re=2, kb=3, avl=4, out=5.
        sim.invoke(3, "key", Some(Value::Int('Z' as i64))).unwrap();
        sim.run_to_quiescence();

        assert_eq!(sim.read(4, "get").unwrap(), Value::Bits(vec![Bit::One]));
        assert_eq!(
            sim.read(5, "get").unwrap(),
            // 'Z' = 0x5A = 0b1011010; LSB-first (bit0..bit6) = 0,1,0,1,1,0,1.
            Value::Bits(vec![Bit::Zero, Bit::One, Bit::Zero, Bit::One, Bit::One, Bit::Zero, Bit::One])
        );
    }
}
