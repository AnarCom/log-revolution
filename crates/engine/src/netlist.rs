//! Netlist compilation: turns a hierarchical description (circuits that may
//! instantiate other circuits) into one flat `Netlist` — the "Netlist IR"
//! from PLAN.md §9, an ephemeral compiled artifact, never saved.
//!
//! Subcircuits are fully elaborated (each instance expanded into its own
//! copies of the internal gates) rather than kept as a runtime tree, like
//! original Logisim's `Circuit`/`CircuitState`. Trade-off, stated plainly:
//! this can blow up in size for deeply-nested, heavily-reused subcircuits,
//! but it keeps the runtime representation a flat array (PLAN.md §3's
//! data-oriented design) and makes "which gates are independent" a static,
//! precomputed property instead of something re-derived through a
//! recursive tree on every step.
//!
//! A subcircuit's *ports* are wire-elided during flattening — an instance's
//! input port fans out directly to whatever it feeds internally, its
//! output port is read directly from whatever drives it internally. No
//! extra gate is materialized for the boundary itself, so it costs nothing
//! at simulation time. `InputPin`/`OutputPin` nodes are a separate,
//! optional thing: named leaf pins a `.ctest` script or the UI can address,
//! usable whether or not the circuit they're in is ever used as a
//! subcircuit.

use crate::components::Gate;
use plugin_abi::Bit;
use std::collections::HashMap;

pub type PinRef = (usize, usize); // (node index, pin index)

#[derive(Debug, Clone)]
pub enum TemplateNode {
    And,
    Or,
    Not,
    InputPin,
    OutputPin,
    /// Reference to another `CircuitTemplate` by name, resolved during
    /// `flatten`. As a connection endpoint, its pins are numbered
    /// `0..input_ports.len()` for inputs then `input_ports.len()..` for
    /// outputs — the instance's own external interface, not its internals.
    Subcircuit(String),
}

#[derive(Debug, Clone, Default)]
pub struct CircuitTemplate {
    pub name: String,
    pub nodes: Vec<TemplateNode>,
    /// (source node, output pin) -> (dest node, input pin).
    pub connections: Vec<(PinRef, PinRef)>,
    /// Per input port: every internal pin it feeds (a port can fan out;
    /// an empty list means "declared but unused internally").
    pub input_ports: Vec<Vec<PinRef>>,
    /// Per output port: the single internal pin it reads from.
    pub output_ports: Vec<PinRef>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetlistError {
    UnknownSubcircuit(String),
    CircularSubcircuit(Vec<String>),
}

#[derive(Debug, Default)]
pub struct Netlist {
    pub gates: Vec<Gate>,
    /// [gate][input pin] -> every driving (gate, output pin) — **not** at
    /// most one. Logisim allows multiple outputs tied to the same point
    /// (PLAN.md §9's tri-state/short-circuit requirement); an empty list
    /// means genuinely unconnected (floating). Resolving more than one
    /// entry is `Bit::combine`'s job (`sim.rs`), not this module's.
    pub input_sources: Vec<Vec<Vec<PinRef>>>,
    /// [gate][output pin] -> consumers driven by it. The reverse index of
    /// `input_sources`, precomputed once so `sim.rs` never has to search.
    pub fanout: Vec<Vec<Vec<PinRef>>>,
}

struct Builder {
    gates: Vec<Gate>,
    input_sources: Vec<Vec<Vec<PinRef>>>,
    fanout: Vec<Vec<Vec<PinRef>>>,
}

impl Builder {
    fn add_gate(&mut self, gate: Gate) -> usize {
        use plugin_abi::Component;
        let idx = self.gates.len();
        self.input_sources.push(vec![Vec::new(); gate.input_count()]);
        self.fanout.push(vec![Vec::new(); gate.output_count()]);
        self.gates.push(gate);
        idx
    }

    /// Adds `src` as *another* driver of `dst` — doesn't replace whatever
    /// was already connected there (see `Netlist::input_sources`).
    fn connect(&mut self, src: PinRef, dst: PinRef) {
        self.input_sources[dst.0][dst.1].push(src);
        self.fanout[src.0][src.1].push(dst);
    }
}

/// A subcircuit instance's resolved boundary, in global (already-flattened)
/// gate indices — what a parent needs to wire directly through it.
struct ChildPorts {
    input_port_targets: Vec<Vec<PinRef>>,
    output_port_sources: Vec<PinRef>,
}

/// Elaborates `entry` (looked up in `library` by name) into a flat
/// `Netlist`, recursively inlining every subcircuit instance.
pub fn flatten(
    entry: &str,
    library: &HashMap<String, CircuitTemplate>,
) -> Result<Netlist, NetlistError> {
    let mut builder = Builder {
        gates: Vec::new(),
        input_sources: Vec::new(),
        fanout: Vec::new(),
    };
    let mut stack = Vec::new();
    expand(entry, library, &mut builder, &mut stack)?;
    Ok(Netlist {
        gates: builder.gates,
        input_sources: builder.input_sources,
        fanout: builder.fanout,
    })
}

fn expand(
    name: &str,
    library: &HashMap<String, CircuitTemplate>,
    builder: &mut Builder,
    stack: &mut Vec<String>,
) -> Result<ChildPorts, NetlistError> {
    if stack.iter().any(|n| n == name) {
        stack.push(name.to_string());
        return Err(NetlistError::CircularSubcircuit(stack.clone()));
    }
    let template = library
        .get(name)
        .ok_or_else(|| NetlistError::UnknownSubcircuit(name.to_string()))?;
    stack.push(name.to_string());

    // Leaf nodes materialize directly; Subcircuit nodes recurse and are
    // remembered only as their resolved boundary (no gate of their own).
    let mut local_to_global: HashMap<usize, usize> = HashMap::new();
    let mut child_ports: HashMap<usize, ChildPorts> = HashMap::new();

    for (local_idx, node) in template.nodes.iter().enumerate() {
        match node {
            TemplateNode::And => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::And));
            }
            TemplateNode::Or => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Or));
            }
            TemplateNode::Not => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Not));
            }
            TemplateNode::InputPin => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::InputPin { value: Bit::Zero }));
            }
            TemplateNode::OutputPin => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::OutputPin { value: Bit::Zero }));
            }
            TemplateNode::Subcircuit(sub_name) => {
                let ports = expand(sub_name, library, builder, stack)?;
                child_ports.insert(local_idx, ports);
            }
        }
    }

    // A local PinRef used as a *source*: leaf gate's own pin, or (for a
    // Subcircuit node) the single global source behind that output port.
    let resolve_source = |local: PinRef| -> PinRef {
        let (node, pin) = local;
        match local_to_global.get(&node) {
            Some(&g) => (g, pin),
            None => {
                let ports = &child_ports[&node];
                let out_idx = pin - ports.input_port_targets.len();
                ports.output_port_sources[out_idx]
            }
        }
    };
    // A local PinRef used as a *destination*: leaf gate's own pin, or (for
    // a Subcircuit node) every global destination that input port fans out
    // to internally.
    let resolve_destinations = |local: PinRef| -> Vec<PinRef> {
        let (node, pin) = local;
        match local_to_global.get(&node) {
            Some(&g) => vec![(g, pin)],
            None => child_ports[&node].input_port_targets[pin].clone(),
        }
    };

    for &(src, dst) in &template.connections {
        let resolved_src = resolve_source(src);
        for target in resolve_destinations(dst) {
            builder.connect(resolved_src, target);
        }
    }

    let input_port_targets = template
        .input_ports
        .iter()
        .map(|refs| refs.iter().flat_map(|&p| resolve_destinations(p)).collect())
        .collect();
    let output_port_sources = template.output_ports.iter().map(|&p| resolve_source(p)).collect();

    stack.pop();
    Ok(ChildPorts {
        input_port_targets,
        output_port_sources,
    })
}
