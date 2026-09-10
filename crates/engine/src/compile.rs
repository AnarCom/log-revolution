//! Compiles a `file_format::ProjectFile` into a `netlist::Netlist` — the
//! missing link between "a `.json` file on disk" and "something
//! `sim::Simulation` can actually run" (PLAN.md §10 Phase 2.7 groundwork).
//!
//! Wires connect by coordinate, same as `.circ` (PLAN.md §9's wire-segment
//! decision): a component's pins have fixed positions relative to its own
//! `x`/`y` (rotated by `facing`), and two points electrically connect iff
//! they coincide. A union-find over every pin position and wire endpoint
//! turns that into groups ("nets"); each net's output-direction pins
//! become drivers of every input-direction pin in the same net — which is
//! just `CircuitTemplate.connections` as an all-pairs product, nothing the
//! rest of the engine doesn't already handle (short circuits included).

use crate::file_format::{Circuit, ComponentInstance, Facing, ProjectFile};
use crate::netlist::{CircuitTemplate, PinRef, TemplateNode};
use plugin_abi::Bit;
use std::collections::HashMap;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CompileError {
    UnknownComponentType { circuit: String, id: String, type_: String },
    UnknownSubcircuit { circuit: String, id: String, referenced: String },
    InvalidPullTarget { circuit: String, id: String, value: String },
    DuplicateComponentId { circuit: String, id: String },
}

type Point = (i32, i32);

/// A component's pins, relative to its own origin, at `Facing::East`
/// (rotated per-instance for other facings). Deliberately small/arbitrary
/// — this is our own schema, not `.circ`'s exact pixel geometry; matching
/// that is the importer's job (PLAN.md §9), not this compiler's.
struct Geometry {
    inputs: Vec<Point>,
    outputs: Vec<Point>,
}

fn leaf_geometry(type_: &str) -> Option<Geometry> {
    match type_ {
        "core:AndGate" | "core:OrGate" => Some(Geometry {
            inputs: vec![(0, -1), (0, 1)],
            outputs: vec![(3, 0)],
        }),
        "core:NotGate" => Some(Geometry {
            inputs: vec![(0, 0)],
            outputs: vec![(2, 0)],
        }),
        "core:InputPin" | "core:PullResistor" => Some(Geometry {
            inputs: vec![],
            outputs: vec![(0, 0)],
        }),
        "core:OutputPin" => Some(Geometry {
            inputs: vec![(0, 0)],
            outputs: vec![],
        }),
        _ => None,
    }
}

/// A subcircuit instance's port count varies per reference, so its
/// footprint is computed rather than looked up: input ports stacked down
/// the left edge, output ports down the right, on a fixed-width box.
/// Arbitrary — nothing renders this yet, only connectivity matters.
const SUBCIRCUIT_WIDTH: i32 = 6;

fn subcircuit_geometry(input_ports: usize, output_ports: usize) -> Geometry {
    Geometry {
        inputs: (0..input_ports).map(|i| (0, 2 * i as i32)).collect(),
        outputs: (0..output_ports).map(|i| (SUBCIRCUIT_WIDTH, 2 * i as i32)).collect(),
    }
}

/// Rotates a `Facing::East`-relative offset for the other three facings.
/// East is the identity; the rest are 90°-step rotations, applied
/// consistently (not matched to any particular on-screen convention, since
/// nothing renders this yet).
fn rotate(offset: Point, facing: Facing) -> Point {
    let (dx, dy) = offset;
    match facing {
        Facing::East => (dx, dy),
        Facing::South => (-dy, dx),
        Facing::West => (-dx, -dy),
        Facing::North => (dy, -dx),
    }
}

/// Union-find over pin/wire-endpoint coordinates.
struct Dsu {
    parent: HashMap<Point, Point>,
}

impl Dsu {
    fn new() -> Self {
        Dsu { parent: HashMap::new() }
    }

    fn find(&mut self, p: Point) -> Point {
        let parent = *self.parent.entry(p).or_insert(p);
        if parent == p {
            p
        } else {
            let root = self.find(parent);
            self.parent.insert(p, root);
            root
        }
    }

    fn union(&mut self, a: Point, b: Point) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent.insert(ra, rb);
        }
    }
}

/// A circuit's ports, in the order they're exposed when it's used as a
/// subcircuit: `InputPin`/`OutputPin` components sorted by `id` — simple
/// and deterministic since this is our own format (not `.circ` import,
/// which has its own pin-ordering rules to match separately later).
fn port_component_ids<'a>(circuit: &'a Circuit, type_: &str) -> Vec<&'a str> {
    let mut ids: Vec<&str> = circuit
        .components
        .iter()
        .filter(|c| c.type_ == type_)
        .map(|c| c.id.as_str())
        .collect();
    ids.sort_unstable();
    ids
}

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

/// Compiles every circuit in `project` into a `CircuitTemplate` library,
/// keyed by circuit name — ready for `netlist::flatten(&project.main_circuit, ..)`.
pub fn compile(project: &ProjectFile) -> Result<HashMap<String, CircuitTemplate>, CompileError> {
    // Needed up front: a subcircuit instance's pin geometry depends on how
    // many ports the *referenced* circuit exposes, and circuits can
    // reference each other regardless of order in the file.
    let port_counts: HashMap<String, (usize, usize)> = project
        .circuits
        .iter()
        .map(|c| {
            let n_in = port_component_ids(c, "core:InputPin").len();
            let n_out = port_component_ids(c, "core:OutputPin").len();
            (c.name.clone(), (n_in, n_out))
        })
        .collect();

    let mut library = HashMap::new();
    for circuit in &project.circuits {
        library.insert(circuit.name.clone(), compile_circuit(circuit, &port_counts)?);
    }
    Ok(library)
}

fn compile_circuit(
    circuit: &Circuit,
    port_counts: &HashMap<String, (usize, usize)>,
) -> Result<CircuitTemplate, CompileError> {
    let mut id_to_index: HashMap<&str, usize> = HashMap::new();
    let mut nodes = Vec::with_capacity(circuit.components.len());
    let mut input_points: Vec<Vec<Point>> = Vec::with_capacity(circuit.components.len());
    let mut output_points: Vec<Vec<Point>> = Vec::with_capacity(circuit.components.len());
    // Only relevant for Subcircuit nodes: netlist.rs numbers a subcircuit
    // instance's pins as one flat space (inputs 0..n_in, outputs
    // n_in..n_in+n_out — see `netlist::TemplateNode::Subcircuit`'s doc),
    // unlike a leaf gate's independently-0-based input/output pins. Track
    // the offset per node so output PinRefs can be shifted to match.
    let mut output_pin_offset: Vec<usize> = Vec::with_capacity(circuit.components.len());

    for comp in &circuit.components {
        if id_to_index.contains_key(comp.id.as_str()) {
            return Err(CompileError::DuplicateComponentId {
                circuit: circuit.name.clone(),
                id: comp.id.clone(),
            });
        }
        let idx = nodes.len();
        id_to_index.insert(comp.id.as_str(), idx);

        let (node, geom) = if let Some(sub_name) = comp.type_.strip_prefix("core:circuit/") {
            let &(n_in, n_out) = port_counts.get(sub_name).ok_or_else(|| CompileError::UnknownSubcircuit {
                circuit: circuit.name.clone(),
                id: comp.id.clone(),
                referenced: sub_name.to_string(),
            })?;
            output_pin_offset.push(n_in);
            (TemplateNode::Subcircuit(sub_name.to_string()), subcircuit_geometry(n_in, n_out))
        } else if let Some(geom) = leaf_geometry(&comp.type_) {
            output_pin_offset.push(0);
            let node = match comp.type_.as_str() {
                "core:AndGate" => TemplateNode::And,
                "core:OrGate" => TemplateNode::Or,
                "core:NotGate" => TemplateNode::Not,
                "core:InputPin" => TemplateNode::InputPin,
                "core:OutputPin" => TemplateNode::OutputPin,
                "core:PullResistor" => TemplateNode::PullResistor(pull_target(circuit, comp)?),
                _ => unreachable!("leaf_geometry and this match must stay in sync"),
            };
            (node, geom)
        } else {
            return Err(CompileError::UnknownComponentType {
                circuit: circuit.name.clone(),
                id: comp.id.clone(),
                type_: comp.type_.clone(),
            });
        };

        let abs = |offsets: &[Point]| -> Vec<Point> {
            offsets
                .iter()
                .map(|&o| {
                    let (rx, ry) = rotate(o, comp.facing);
                    (comp.x + rx, comp.y + ry)
                })
                .collect()
        };
        input_points.push(abs(&geom.inputs));
        output_points.push(abs(&geom.outputs));
        nodes.push(node);
    }

    // Union every pin position and wire endpoint that coincides, then
    // snapshot each pin's net root once (avoids re-walking the DSU later).
    let mut dsu = Dsu::new();
    for pts in input_points.iter().chain(output_points.iter()) {
        for &p in pts {
            dsu.find(p);
        }
    }
    for wire in &circuit.wires {
        dsu.union((wire.from[0], wire.from[1]), (wire.to[0], wire.to[1]));
    }

    let mut nets: HashMap<Point, (Vec<PinRef>, Vec<PinRef>)> = HashMap::new();
    let mut output_pin_root: Vec<Vec<Point>> = Vec::with_capacity(output_points.len());
    for (node_idx, pts) in output_points.iter().enumerate() {
        let mut roots = Vec::with_capacity(pts.len());
        for (pin_idx, &p) in pts.iter().enumerate() {
            let root = dsu.find(p);
            let pin_ref = (node_idx, pin_idx + output_pin_offset[node_idx]);
            nets.entry(root).or_default().0.push(pin_ref);
            roots.push(root);
        }
        output_pin_root.push(roots);
    }
    let mut input_pin_root: Vec<Vec<Point>> = Vec::with_capacity(input_points.len());
    for (node_idx, pts) in input_points.iter().enumerate() {
        let mut roots = Vec::with_capacity(pts.len());
        for (pin_idx, &p) in pts.iter().enumerate() {
            let root = dsu.find(p);
            nets.entry(root).or_default().1.push((node_idx, pin_idx));
            roots.push(root);
        }
        input_pin_root.push(roots);
    }

    let mut connections = Vec::new();
    for (outs, ins) in nets.values() {
        for &src in outs {
            for &dst in ins {
                connections.push((src, dst));
            }
        }
    }

    let input_ports = port_component_ids(circuit, "core:InputPin")
        .into_iter()
        .map(|id| {
            let node_idx = id_to_index[id];
            let root = output_pin_root[node_idx][0]; // InputPin has exactly one output pin
            nets[&root].1.clone() // that net's input-direction pins = what this port feeds
        })
        .collect();
    let output_ports = port_component_ids(circuit, "core:OutputPin")
        .into_iter()
        .map(|id| {
            let node_idx = id_to_index[id];
            let root = input_pin_root[node_idx][0]; // OutputPin has exactly one input pin
            nets[&root].0.clone() // that net's output-direction pins = what drives this port
        })
        .collect();

    // Every InputPin/OutputPin used above to *declare* a port is also a
    // real, materialized gate — mark it so `netlist::expand` can skip its
    // own internal wiring when this template is used as a nested
    // subcircuit instance (see `port_marker_nodes`'s doc comment for why:
    // otherwise it becomes a second, conflicting driver alongside the
    // parent's real one).
    let port_marker_nodes = port_component_ids(circuit, "core:InputPin")
        .into_iter()
        .chain(port_component_ids(circuit, "core:OutputPin"))
        .map(|id| id_to_index[id])
        .collect();

    Ok(CircuitTemplate {
        name: circuit.name.clone(),
        nodes,
        connections,
        input_ports,
        output_ports,
        port_marker_nodes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::file_format::{Circuit, ComponentInstance, ProjectFile, WireSegment};
    use crate::netlist::flatten;
    use crate::sim::Simulation;
    use plugin_abi::Value;
    use serde_json::json;
    use std::collections::BTreeMap;

    fn comp(id: &str, type_: &str, x: i32, y: i32) -> ComponentInstance {
        ComponentInstance {
            id: id.to_string(),
            type_: type_.to_string(),
            x,
            y,
            facing: Facing::East,
            attrs: BTreeMap::new(),
        }
    }

    fn wire(id: &str, from: [i32; 2], to: [i32; 2]) -> WireSegment {
        WireSegment { id: id.to_string(), from, to }
    }

    fn get_bit(sim: &Simulation, gate: usize) -> Bit {
        match sim.read(gate, "get").unwrap() {
            Value::Bits(bits) => bits[0],
            other => panic!("unexpected readout {other:?}"),
        }
    }

    /// a(0,-1) --wire--> and.in0(0,-1); b(0,1) --wire--> and.in1(0,1);
    /// and.out(3,0) --wire--> out(3,0). Coordinates are chosen to exactly
    /// match `leaf_geometry`'s AndGate offsets, at x=0 for a/b/and and
    /// x=3 for the output pin, so wires are zero-length "same point"
    /// connections — geometry only needs to line up, not be pretty.
    fn and_project() -> ProjectFile {
        ProjectFile {
            schema_version: 1,
            main_circuit: "main".to_string(),
            circuits: vec![Circuit {
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
            }],
        }
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

    #[test]
    fn compiles_a_subcircuit_reference_with_matching_port_count() {
        let and_sub = Circuit {
            name: "and2".to_string(),
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
        };
        // Instance footprint per `subcircuit_geometry`: 2 inputs on the
        // left (y=0,2), 1 output on the right (x=SUBCIRCUIT_WIDTH, y=0).
        let main = Circuit {
            name: "main".to_string(),
            components: vec![
                comp("a1", "core:InputPin", 10, 0),
                comp("b1", "core:InputPin", 10, 2),
                comp("inst", "core:circuit/and2", 10, 0),
                comp("out1", "core:OutputPin", 10 + SUBCIRCUIT_WIDTH, 0),
            ],
            wires: vec![
                wire("w1", [10, 0], [10, 0]),
                wire("w2", [10, 2], [10, 2]),
                wire("w3", [10 + SUBCIRCUIT_WIDTH, 0], [10 + SUBCIRCUIT_WIDTH, 0]),
            ],
            annotations: vec![],
        };
        let project = ProjectFile {
            schema_version: 1,
            main_circuit: "main".to_string(),
            circuits: vec![and_sub, main],
        };

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        // Elaboration order: main's a1,b1 -> 0,1; expanding `inst` recurses
        // into and2's own leaves in and2's component order (a,b,g,out) ->
        // 2..6; then main's own out1 -> 6.
        assert_eq!(netlist.gates.len(), 7);
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "on", None).unwrap(); // a1
        sim.invoke(1, "on", None).unwrap(); // b1
        sim.run_to_quiescence();
        assert_eq!(get_bit(&sim, 6), Bit::One);
    }

    #[test]
    fn rejects_unknown_component_type() {
        let project = ProjectFile {
            schema_version: 1,
            main_circuit: "main".to_string(),
            circuits: vec![Circuit {
                name: "main".to_string(),
                components: vec![comp("x", "core:DoesNotExist", 0, 0)],
                wires: vec![],
                annotations: vec![],
            }],
        };
        let err = compile(&project).unwrap_err();
        assert_eq!(
            err,
            CompileError::UnknownComponentType {
                circuit: "main".to_string(),
                id: "x".to_string(),
                type_: "core:DoesNotExist".to_string(),
            }
        );
    }

    #[test]
    fn rejects_duplicate_component_ids() {
        let project = ProjectFile {
            schema_version: 1,
            main_circuit: "main".to_string(),
            circuits: vec![Circuit {
                name: "main".to_string(),
                components: vec![
                    comp("dup", "core:InputPin", 0, 0),
                    comp("dup", "core:OutputPin", 5, 0),
                ],
                wires: vec![],
                annotations: vec![],
            }],
        };
        let err = compile(&project).unwrap_err();
        assert_eq!(
            err,
            CompileError::DuplicateComponentId {
                circuit: "main".to_string(),
                id: "dup".to_string(),
            }
        );
    }

    #[test]
    fn pull_resistor_reads_pull_attr_matching_logisim_options() {
        let mut attrs = BTreeMap::new();
        attrs.insert("pull".to_string(), json!("X"));
        let project = ProjectFile {
            schema_version: 1,
            main_circuit: "main".to_string(),
            circuits: vec![Circuit {
                name: "main".to_string(),
                components: vec![ComponentInstance {
                    attrs,
                    ..comp("r", "core:PullResistor", 0, 0)
                }],
                wires: vec![],
                annotations: vec![],
            }],
        };
        let library = compile(&project).unwrap();
        let template = &library["main"];
        assert!(matches!(template.nodes[0], TemplateNode::PullResistor(Bit::Error)));
    }
}
