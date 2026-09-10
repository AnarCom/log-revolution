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
    And { bits: u8, inputs: usize },
    Or { bits: u8, inputs: usize },
    Not { bits: u8 },
    Nand { bits: u8, inputs: usize },
    Nor { bits: u8, inputs: usize },
    Xor { bits: u8, inputs: usize },
    Xnor { bits: u8, inputs: usize },
    Buffer { bits: u8 },
    Constant { bits: u8, value: u32 },
    InputPin { bits: u8 },
    OutputPin { bits: u8 },
    /// A weak source pulling its point to `Bit` when nothing else drives
    /// it — see `Gate::PullResistor` for why it's wired in like a normal
    /// driver but resolved specially.
    PullResistor(Bit),
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
    /// Per output port: every internal pin driving it — plural, not one,
    /// symmetric with `input_ports`. A port can be undriven (empty — reads
    /// `Unknown` outside, same as any other point) or driven by more than
    /// one source (a real conflict inside the subcircuit should still show
    /// up as `Error` outside it, not get silently resolved at the
    /// boundary).
    pub output_ports: Vec<Vec<PinRef>>,
    /// Local node indices that exist purely to *declare* a port (an
    /// `InputPin`/`OutputPin` whose net was used to compute `input_ports`/
    /// `output_ports`) — not an independent extra requirement, a
    /// consequence of ports being expressed by reusing ordinary leaf nodes
    /// (PLAN.md §9) rather than a dedicated port-only node kind. See
    /// `expand`'s use of this field for why it matters.
    pub port_marker_nodes: Vec<usize>,
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
    output_port_sources: Vec<Vec<PinRef>>,
}

/// Elaborates `entry` (looked up in `library` by name) into a flat
/// `Netlist`, recursively inlining every subcircuit instance.
pub fn flatten(
    entry: &str,
    library: &HashMap<String, CircuitTemplate>,
) -> Result<Netlist, NetlistError> {
    flatten_with_entry_map(entry, library).map(|(netlist, _)| netlist)
}

/// Same as `flatten`, but also returns the entry circuit's own local node
/// index -> global gate index mapping. Only meaningful for the entry
/// itself (nested subcircuit instances have no single global index — each
/// instance gets its own copy, see the module doc) — needed to address
/// top-level components by id (`.ctest`'s `component[id]`, PLAN.md §7;
/// `engine-cli`'s headless report).
pub fn flatten_with_entry_map(
    entry: &str,
    library: &HashMap<String, CircuitTemplate>,
) -> Result<(Netlist, HashMap<usize, usize>), NetlistError> {
    let mut builder = Builder {
        gates: Vec::new(),
        input_sources: Vec::new(),
        fanout: Vec::new(),
    };
    let mut stack = Vec::new();
    let (_, entry_map) = expand(entry, library, &mut builder, &mut stack, true)?;
    Ok((
        Netlist {
            gates: builder.gates,
            input_sources: builder.input_sources,
            fanout: builder.fanout,
        },
        entry_map,
    ))
}

fn expand(
    name: &str,
    library: &HashMap<String, CircuitTemplate>,
    builder: &mut Builder,
    stack: &mut Vec<String>,
    is_entry: bool,
) -> Result<(ChildPorts, HashMap<usize, usize>), NetlistError> {
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
            TemplateNode::And { bits, inputs } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::And { bits: *bits, inputs: *inputs }));
            }
            TemplateNode::Or { bits, inputs } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Or { bits: *bits, inputs: *inputs }));
            }
            TemplateNode::Not { bits } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Not { bits: *bits }));
            }
            TemplateNode::Nand { bits, inputs } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Nand { bits: *bits, inputs: *inputs }));
            }
            TemplateNode::Nor { bits, inputs } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Nor { bits: *bits, inputs: *inputs }));
            }
            TemplateNode::Xor { bits, inputs } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Xor { bits: *bits, inputs: *inputs }));
            }
            TemplateNode::Xnor { bits, inputs } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Xnor { bits: *bits, inputs: *inputs }));
            }
            TemplateNode::Buffer { bits } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Buffer { bits: *bits }));
            }
            TemplateNode::Constant { bits, value } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Constant { bits: *bits, value: *value }));
            }
            TemplateNode::InputPin { bits } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::InputPin { bits: *bits, value: vec![Bit::Zero; *bits as usize] }));
            }
            TemplateNode::OutputPin { bits } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::OutputPin { bits: *bits, value: vec![Bit::Zero; *bits as usize] }));
            }
            TemplateNode::PullResistor(to) => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::PullResistor { to: *to }));
            }
            TemplateNode::Subcircuit(sub_name) => {
                let (ports, _) = expand(sub_name, library, builder, stack, false)?;
                child_ports.insert(local_idx, ports);
            }
        }
    }

    // A local PinRef used as a *source*: leaf gate's own pin (one), or (for
    // a Subcircuit node) every global source behind that output port —
    // plural, same reasoning as `resolve_destinations` below.
    let resolve_sources = |local: PinRef| -> Vec<PinRef> {
        let (node, pin) = local;
        match local_to_global.get(&node) {
            Some(&g) => vec![(g, pin)],
            None => {
                let ports = &child_ports[&node];
                let out_idx = pin - ports.input_port_targets.len();
                ports.output_port_sources[out_idx].clone()
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

    // Nested (this circuit is a *subcircuit instance*, not the flatten
    // entry): a port's real value comes from whatever the parent wires
    // into it (via `resolve_sources`/`resolve_destinations` above, using
    // `input_ports`/`output_ports`) — so the marker node's *own* internal
    // wiring must be skipped here, or the marker's default state (e.g. an
    // `InputPin` sitting at `Bit::Zero`, never invoked) becomes a second,
    // conflicting driver on the exact same point as the real one. At the
    // top level there's no external driver to conflict with, so the
    // marker wires normally and is simply what a `.ctest`/UI addresses.
    let skip = |n: usize| !is_entry && template.port_marker_nodes.contains(&n);
    for &(src, dst) in &template.connections {
        if skip(src.0) || skip(dst.0) {
            continue;
        }
        for resolved_src in resolve_sources(src) {
            for target in resolve_destinations(dst) {
                builder.connect(resolved_src, target);
            }
        }
    }

    let input_port_targets = template
        .input_ports
        .iter()
        .map(|refs| refs.iter().flat_map(|&p| resolve_destinations(p)).collect())
        .collect();
    let output_port_sources = template
        .output_ports
        .iter()
        .map(|refs| refs.iter().flat_map(|&p| resolve_sources(p)).collect())
        .collect();

    stack.pop();
    Ok((
        ChildPorts {
            input_port_targets,
            output_port_sources,
        },
        local_to_global,
    ))
}
