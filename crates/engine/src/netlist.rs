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

use crate::components::{Gate, Trigger};
use plugin_abi::Bit;
use std::collections::HashMap;

pub type PinRef = (usize, usize); // (node index, pin index)
/// (node index, pin index, bit index) — the unit of connectivity.
/// Everything used to connect whole pins to whole pins (implicitly, bit `i`
/// of every source aligned with bit `i` of every destination); `Splitter`
/// (`compile/splitter.rs`) breaks that assumption — it fuses specific bits
/// of its combined pin to specific bits of each fanout pin, arbitrarily
/// remapped — so every connection is bit-addressed now, not just the ones a
/// `Splitter` actually touches (one uniform representation, not two).
pub type BitRef = (usize, usize, u8);

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
    /// See `Gate::Clock`'s doc comment: `high`/`low` are the period
    /// attributes, not an independent timer — actual advancement happens
    /// only through `Simulation::tick`.
    Clock { high: u64, low: u64 },
    Register { bits: u8, trigger: Trigger },
    /// See `Gate::Mux`: input order is data lines, then select, then
    /// `enable` if `has_enable`.
    Mux { bits: u8, select_bits: u8, has_enable: bool, disabled_zero: bool },
    /// See `Gate::Demux`: input order is select, then `enable` if
    /// `has_enable`, then the one data line; `2^select_bits` outputs.
    Demux { bits: u8, select_bits: u8, has_enable: bool, disabled_zero: bool, tristate: bool },
    /// See `Gate::Adder`: inputs `in0`, `in1`, `c_in`; outputs `sum`, `c_out`.
    Adder { bits: u8 },
    /// See `Gate::Subtractor`: inputs `in0`, `in1`, `b_in`; outputs `diff`,
    /// `b_out`.
    Subtractor { bits: u8 },
    /// See `Gate::Comparator`: inputs `in0`, `in1`; outputs `gt`, `eq`, `lt`.
    Comparator { bits: u8, signed: bool },
    /// Reference to another `CircuitTemplate` by name, resolved during
    /// `flatten`. As a connection endpoint, its pins are numbered
    /// `0..input_ports.len()` for inputs then `input_ports.len()..` for
    /// outputs — the instance's own external interface, not its internals.
    Subcircuit(String),
}

impl TemplateNode {
    /// Declared width of input pin `pin` — mirrors `Gate::input_width`
    /// exactly (same fields, same pin ordering; see each category's own
    /// `input_width_*` for the per-kind reasoning), needed by
    /// `compile::compile_circuit` to size a pin's bit lanes *before* any
    /// `Gate` exists yet. Panics for `Subcircuit`, which has no static
    /// width of its own — `compile_circuit` never calls this for one,
    /// using the port-width table it threads through instead.
    pub(crate) fn input_width(&self, pin: usize) -> u8 {
        match self {
            TemplateNode::And { bits, .. }
            | TemplateNode::Or { bits, .. }
            | TemplateNode::Not { bits }
            | TemplateNode::Nand { bits, .. }
            | TemplateNode::Nor { bits, .. }
            | TemplateNode::Xor { bits, .. }
            | TemplateNode::Xnor { bits, .. }
            | TemplateNode::Buffer { bits } => *bits,
            TemplateNode::OutputPin { bits } => *bits,
            TemplateNode::Register { bits, .. } => {
                if pin == 0 {
                    *bits
                } else {
                    1
                }
            }
            TemplateNode::Mux { bits, select_bits, .. } => {
                let n = 1usize << select_bits;
                if pin < n {
                    *bits
                } else if pin == n {
                    *select_bits
                } else {
                    1
                }
            }
            TemplateNode::Demux { bits, select_bits, has_enable, .. } => {
                if pin == 0 {
                    *select_bits
                } else if *has_enable && pin == 1 {
                    1
                } else {
                    *bits
                }
            }
            TemplateNode::Adder { bits } | TemplateNode::Subtractor { bits } => {
                if pin < 2 {
                    *bits
                } else {
                    1
                }
            }
            TemplateNode::Comparator { bits, .. } => *bits,
            TemplateNode::Constant { .. }
            | TemplateNode::InputPin { .. }
            | TemplateNode::PullResistor(_)
            | TemplateNode::Clock { .. } => unreachable!("no input pins on this node kind"),
            TemplateNode::Subcircuit(_) => unreachable!("Subcircuit width is resolved via the port-width table"),
        }
    }

    /// Declared width of output pin `pin` — see `input_width`'s doc.
    pub(crate) fn output_width(&self, pin: usize) -> u8 {
        match self {
            TemplateNode::And { bits, .. }
            | TemplateNode::Or { bits, .. }
            | TemplateNode::Not { bits }
            | TemplateNode::Nand { bits, .. }
            | TemplateNode::Nor { bits, .. }
            | TemplateNode::Xor { bits, .. }
            | TemplateNode::Xnor { bits, .. }
            | TemplateNode::Buffer { bits } => *bits,
            TemplateNode::Constant { bits, .. } => *bits,
            TemplateNode::InputPin { bits } => *bits,
            TemplateNode::PullResistor(_) => 1,
            TemplateNode::Clock { .. } => 1,
            TemplateNode::Register { bits, .. } => *bits,
            TemplateNode::Mux { bits, .. } | TemplateNode::Demux { bits, .. } => *bits,
            TemplateNode::Adder { bits } | TemplateNode::Subtractor { bits } => {
                if pin == 0 {
                    *bits
                } else {
                    1
                }
            }
            TemplateNode::Comparator { .. } => 1,
            TemplateNode::OutputPin { .. } => unreachable!("OutputPin has no output pins"),
            TemplateNode::Subcircuit(_) => unreachable!("Subcircuit width is resolved via the port-width table"),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct CircuitTemplate {
    pub name: String,
    pub nodes: Vec<TemplateNode>,
    /// (source bit) -> (dest bit). Bit-addressed even where no `Splitter`
    /// is involved (see `BitRef`'s doc) — a uniform representation rather
    /// than a whole-pin fast path plus a bit-addressed special case.
    pub connections: Vec<(BitRef, BitRef)>,
    /// Per input port, per bit of that port: every internal bit it feeds (a
    /// bit can fan out; an empty list means "declared but unused
    /// internally", or unreachable through a `Splitter` that left it
    /// unconnected).
    pub input_ports: Vec<Vec<Vec<BitRef>>>,
    /// Per output port, per bit of that port: every internal bit driving
    /// it — plural, not one, symmetric with `input_ports`. A bit can be
    /// undriven (empty — reads `Unknown` outside, same as any other point)
    /// or driven by more than one source (a real conflict inside the
    /// subcircuit should still show up as `Error` outside it, not get
    /// silently resolved at the boundary).
    pub output_ports: Vec<Vec<Vec<BitRef>>>,
    /// Local node indices that exist purely to *declare* a port (an
    /// `InputPin`/`OutputPin` whose net was used to compute `input_ports`/
    /// `output_ports`) — not an independent extra requirement, a
    /// consequence of ports being expressed by reusing ordinary leaf nodes
    /// (PLAN.md §9) rather than a dedicated port-only node kind. See
    /// `expand`'s use of this field for why it matters.
    pub port_marker_nodes: Vec<usize>,
    /// Every *real* (non-`Splitter`) component's JSON `id` -> its local
    /// node index in `nodes`. Needed now because that index is no longer
    /// just "position in the source `Circuit.components` array" — a
    /// `Splitter` (`compile/splitter.rs`) consumes a slot in that array but
    /// never becomes a node, so positions after one are off by however many
    /// splitters precede them. Callers that address components by id
    /// (`.ctest`, `engine-cli`'s report) must go through this map rather
    /// than re-deriving the index themselves.
    pub component_index: HashMap<String, usize>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NetlistError {
    UnknownSubcircuit(String),
    CircularSubcircuit(Vec<String>),
}

#[derive(Debug, Default)]
pub struct Netlist {
    pub gates: Vec<Gate>,
    /// [gate][input pin][bit] -> every driving bit — **not** at most one.
    /// Logisim allows multiple outputs tied to the same point (PLAN.md §9's
    /// tri-state/short-circuit requirement); an empty list means genuinely
    /// unconnected (floating). Resolving more than one entry is
    /// `Bit::combine`'s job (`sim.rs`), not this module's. Pre-sized to
    /// each pin's true declared width (`Gate::input_width`) at
    /// `Builder::add_gate` time, so a bit with zero drivers still has a
    /// (empty) slot — `sim.rs::gather_inputs` needs that to assemble a
    /// correctly-*sized* signal, not just a correctly-valued one.
    pub input_sources: Vec<Vec<Vec<Vec<BitRef>>>>,
    /// [gate][output pin][bit] -> consumers driven by it. The reverse index
    /// of `input_sources`, precomputed once so `sim.rs` never has to
    /// search.
    pub fanout: Vec<Vec<Vec<Vec<BitRef>>>>,
}

struct Builder {
    gates: Vec<Gate>,
    input_sources: Vec<Vec<Vec<Vec<BitRef>>>>,
    fanout: Vec<Vec<Vec<Vec<BitRef>>>>,
}

impl Builder {
    fn add_gate(&mut self, gate: Gate) -> usize {
        use plugin_abi::Component;
        let idx = self.gates.len();
        let in_sources = (0..gate.input_count()).map(|p| vec![Vec::new(); gate.input_width(p) as usize]).collect();
        let out_fanout = (0..gate.output_count()).map(|p| vec![Vec::new(); gate.output_width(p) as usize]).collect();
        self.input_sources.push(in_sources);
        self.fanout.push(out_fanout);
        self.gates.push(gate);
        idx
    }

    /// Adds `src` as *another* driver of `dst` — doesn't replace whatever
    /// was already connected there (see `Netlist::input_sources`).
    fn connect(&mut self, src: BitRef, dst: BitRef) {
        self.input_sources[dst.0][dst.1][dst.2 as usize].push(src);
        self.fanout[src.0][src.1][src.2 as usize].push(dst);
    }
}

/// A subcircuit instance's resolved boundary, in global (already-flattened)
/// gate/bit indices — what a parent needs to wire directly through it.
struct ChildPorts {
    input_port_targets: Vec<Vec<Vec<BitRef>>>,
    output_port_sources: Vec<Vec<Vec<BitRef>>>,
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
            TemplateNode::Clock { high, low } => {
                local_to_global.insert(
                    local_idx,
                    builder.add_gate(Gate::Clock { high: *high, low: *low, clicks: 0, sending: Bit::Zero }),
                );
            }
            TemplateNode::Register { bits, trigger } => {
                local_to_global.insert(
                    local_idx,
                    builder.add_gate(Gate::Register { bits: *bits, trigger: *trigger, value: 0, last_clock: Bit::Zero }),
                );
            }
            TemplateNode::Mux { bits, select_bits, has_enable, disabled_zero } => {
                local_to_global.insert(
                    local_idx,
                    builder.add_gate(Gate::Mux {
                        bits: *bits,
                        select_bits: *select_bits,
                        has_enable: *has_enable,
                        disabled_zero: *disabled_zero,
                    }),
                );
            }
            TemplateNode::Demux { bits, select_bits, has_enable, disabled_zero, tristate } => {
                local_to_global.insert(
                    local_idx,
                    builder.add_gate(Gate::Demux {
                        bits: *bits,
                        select_bits: *select_bits,
                        has_enable: *has_enable,
                        disabled_zero: *disabled_zero,
                        tristate: *tristate,
                    }),
                );
            }
            TemplateNode::Adder { bits } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Adder { bits: *bits }));
            }
            TemplateNode::Subtractor { bits } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Subtractor { bits: *bits }));
            }
            TemplateNode::Comparator { bits, signed } => {
                local_to_global.insert(local_idx, builder.add_gate(Gate::Comparator { bits: *bits, signed: *signed }));
            }
            TemplateNode::Subcircuit(sub_name) => {
                let (ports, _) = expand(sub_name, library, builder, stack, false)?;
                child_ports.insert(local_idx, ports);
            }
        }
    }

    // A local BitRef used as a *source*: leaf gate's own bit (one), or (for
    // a Subcircuit node) every global source behind that output port's
    // bit — plural, same reasoning as `resolve_destinations` below.
    let resolve_sources = |local: BitRef| -> Vec<BitRef> {
        let (node, pin, bit) = local;
        match local_to_global.get(&node) {
            Some(&g) => vec![(g, pin, bit)],
            None => {
                let ports = &child_ports[&node];
                let out_idx = pin - ports.input_port_targets.len();
                ports.output_port_sources[out_idx][bit as usize].clone()
            }
        }
    };
    // A local BitRef used as a *destination*: leaf gate's own bit, or (for
    // a Subcircuit node) every global destination that input port's bit
    // fans out to internally.
    let resolve_destinations = |local: BitRef| -> Vec<BitRef> {
        let (node, pin, bit) = local;
        match local_to_global.get(&node) {
            Some(&g) => vec![(g, pin, bit)],
            None => child_ports[&node].input_port_targets[pin][bit as usize].clone(),
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
        .map(|per_bit| per_bit.iter().map(|refs| refs.iter().flat_map(|&p| resolve_destinations(p)).collect()).collect())
        .collect();
    let output_port_sources = template
        .output_ports
        .iter()
        .map(|per_bit| per_bit.iter().map(|refs| refs.iter().flat_map(|&p| resolve_sources(p)).collect()).collect())
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
