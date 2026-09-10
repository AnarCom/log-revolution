use super::*;
use crate::netlist::{flatten, CircuitTemplate, TemplateNode};
use plugin_abi::Value;
use std::collections::HashMap;

fn set(sim: &mut Simulation, gate: usize, on: bool) {
    sim.invoke(gate, if on { "on" } else { "off" }, None).unwrap();
}

fn get_bit(sim: &Simulation, gate: usize) -> Bit {
    match sim.read(gate, "get").unwrap() {
        Value::Bits(bits) => bits[0],
        other => panic!("unexpected readout {other:?}"),
    }
}

/// a=1, b=1 -> InputPin -> InputPin -> And -> OutputPin, straight line.
/// Gate indices match node order 1:1 (no subcircuits in this template).
fn and_circuit() -> CircuitTemplate {
    CircuitTemplate {
        name: "and_circuit".to_string(),
        nodes: vec![
            TemplateNode::InputPin { bits: 1 },  // 0: a
            TemplateNode::InputPin { bits: 1 },  // 1: b
            TemplateNode::And { bits: 1, inputs: 2 },       // 2
            TemplateNode::OutputPin { bits: 1 }, // 3: out
        ],
        connections: vec![((0, 0, 0), (2, 0, 0)), ((1, 0, 0), (2, 1, 0)), ((2, 0, 0), (3, 0, 0))],
        input_ports: Vec::new(),
        output_ports: Vec::new(),
        port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
    }
}

#[test]
fn simple_and_circuit_propagates() {
    let mut library = HashMap::new();
    library.insert("main".to_string(), and_circuit());
    let netlist = flatten("main", &library).unwrap();
    let mut sim = Simulation::new(netlist);

    set(&mut sim, 0, true);
    set(&mut sim, 1, true);
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 3), Bit::One);

    set(&mut sim, 1, false);
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 3), Bit::Zero);
}

/// A reusable 2-input AND with no leaf Pin nodes of its own — its ports
/// point straight at the And gate, proving ports don't need a
/// materialized boundary gate (netlist.rs's wire-elision claim).
fn and2_subcircuit() -> CircuitTemplate {
    CircuitTemplate {
        name: "and2".to_string(),
        nodes: vec![TemplateNode::And { bits: 1, inputs: 2 }], // 0
        connections: Vec::new(),
        input_ports: vec![vec![vec![(0, 0, 0)]], vec![vec![(0, 1, 0)]]],
        output_ports: vec![vec![vec![(0, 0, 0)]]],
        port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
    }
}

/// Two independent instances of `and2`, fed different inputs, to prove
/// flattening gives each instance its own gates (not shared state).
fn two_instances_circuit() -> CircuitTemplate {
    CircuitTemplate {
        name: "main".to_string(),
        nodes: vec![
            TemplateNode::InputPin { bits: 1 },             // 0: a1
            TemplateNode::InputPin { bits: 1 },             // 1: b1
            TemplateNode::InputPin { bits: 1 },             // 2: a2
            TemplateNode::InputPin { bits: 1 },             // 3: b2
            TemplateNode::Subcircuit("and2".to_string()), // 4: inst1
            TemplateNode::Subcircuit("and2".to_string()), // 5: inst2
            TemplateNode::OutputPin { bits: 1 },            // 6: out1
            TemplateNode::OutputPin { bits: 1 },            // 7: out2
        ],
        connections: vec![
            ((0, 0, 0), (4, 0, 0)),
            ((1, 0, 0), (4, 1, 0)),
            ((4, 2, 0), (6, 0, 0)), // inst1's sole output port is pin index 2 (after the 2 input ports)
            ((2, 0, 0), (5, 0, 0)),
            ((3, 0, 0), (5, 1, 0)),
            ((5, 2, 0), (7, 0, 0)),
        ],
        input_ports: Vec::new(),
        output_ports: Vec::new(),
        port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
    }
}

#[test]
fn subcircuit_instances_are_independent() {
    let mut library = HashMap::new();
    library.insert("and2".to_string(), and2_subcircuit());
    library.insert("main".to_string(), two_instances_circuit());
    let netlist = flatten("main", &library).unwrap();
    // Elaboration allocates leaves in node order, recursing into each
    // Subcircuit node as it's reached: a1,b1,a2,b2 -> 0..4, inst1's And
    // -> 4, inst2's And -> 5, out1,out2 -> 6,7.
    assert_eq!(netlist.gates.len(), 8);

    let mut sim = Simulation::new(netlist);
    set(&mut sim, 0, true); // a1
    set(&mut sim, 1, true); // b1
    set(&mut sim, 2, true); // a2
    set(&mut sim, 3, false); // b2
    sim.run_to_quiescence();

    assert_eq!(get_bit(&sim, 6), Bit::One); // out1 = a1 & b1 = 1
    assert_eq!(get_bit(&sim, 7), Bit::Zero); // out2 = a2 & b2 = 0
}

/// Two drivers on the same point: agreement passes through, conflict
/// is `Error` — PLAN.md §9's short-circuit/tri-state requirement.
fn shared_point_circuit() -> CircuitTemplate {
    CircuitTemplate {
        name: "main".to_string(),
        nodes: vec![
            TemplateNode::InputPin { bits: 1 },  // 0: a
            TemplateNode::InputPin { bits: 1 },  // 1: b
            TemplateNode::OutputPin { bits: 1 }, // 2: out — fed by both a and b
        ],
        connections: vec![((0, 0, 0), (2, 0, 0)), ((1, 0, 0), (2, 0, 0))],
        input_ports: Vec::new(),
        output_ports: Vec::new(),
        port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
    }
}

#[test]
fn multiple_drivers_combine_and_detect_short_circuit() {
    let mut library = HashMap::new();
    library.insert("main".to_string(), shared_point_circuit());
    let netlist = flatten("main", &library).unwrap();
    let mut sim = Simulation::new(netlist);

    set(&mut sim, 0, true);
    set(&mut sim, 1, true);
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 2), Bit::One, "agreeing drivers pass through");

    set(&mut sim, 1, false);
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 2), Bit::Error, "disagreeing drivers short-circuit");
}

/// Two entirely disjoint AND chains in one netlist — nothing in one
/// depends on the other, so they land in the same batch every step
/// and get evaluated in the same `par_iter_mut` pass (sim.rs module
/// doc). Correctness here is what "independent branches parallelize
/// safely" actually cashes out to for a single-threaded test.
fn two_disjoint_chains() -> CircuitTemplate {
    CircuitTemplate {
        name: "main".to_string(),
        nodes: vec![
            TemplateNode::InputPin { bits: 1 },  // 0: a1
            TemplateNode::InputPin { bits: 1 },  // 1: b1
            TemplateNode::And { bits: 1, inputs: 2 },       // 2
            TemplateNode::OutputPin { bits: 1 }, // 3: out1
            TemplateNode::InputPin { bits: 1 },  // 4: a2
            TemplateNode::InputPin { bits: 1 },  // 5: b2
            TemplateNode::And { bits: 1, inputs: 2 },       // 6
            TemplateNode::OutputPin { bits: 1 }, // 7: out2
        ],
        connections: vec![
            ((0, 0, 0), (2, 0, 0)),
            ((1, 0, 0), (2, 1, 0)),
            ((2, 0, 0), (3, 0, 0)),
            ((4, 0, 0), (6, 0, 0)),
            ((5, 0, 0), (6, 1, 0)),
            ((6, 0, 0), (7, 0, 0)),
        ],
        input_ports: Vec::new(),
        output_ports: Vec::new(),
        port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
    }
}

#[test]
fn independent_branches_evaluate_correctly_in_the_same_batch() {
    let mut library = HashMap::new();
    library.insert("main".to_string(), two_disjoint_chains());
    let netlist = flatten("main", &library).unwrap();
    let mut sim = Simulation::new(netlist);

    set(&mut sim, 0, true);
    set(&mut sim, 1, false);
    set(&mut sim, 4, true);
    set(&mut sim, 5, true);
    sim.run_to_quiescence();

    assert_eq!(get_bit(&sim, 3), Bit::Zero);
    assert_eq!(get_bit(&sim, 7), Bit::One);
}

/// out is fed by a PullResistor(One) alone, or by an InputPin and a
/// PullResistor(One) together — exercises `pullValue`'s three real
/// branches (`logisim-port`'s `CircuitWires.java`).
fn pulled_output_circuit() -> CircuitTemplate {
    CircuitTemplate {
        name: "main".to_string(),
        nodes: vec![
            TemplateNode::InputPin { bits: 1 },             // 0: driver (starts off)
            TemplateNode::PullResistor(Bit::One), // 1: pull-up
            TemplateNode::OutputPin { bits: 1 },            // 2: out — fed by both
        ],
        connections: vec![((0, 0, 0), (2, 0, 0)), ((1, 0, 0), (2, 0, 0))],
        input_ports: Vec::new(),
        output_ports: Vec::new(),
        port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
    }
}

#[test]
fn pull_resistor_only_wins_when_nothing_else_drives() {
    let mut library = HashMap::new();
    library.insert("main".to_string(), pulled_output_circuit());
    let netlist = flatten("main", &library).unwrap();
    let mut sim = Simulation::new(netlist);

    // Nothing real driving (InputPin defaults to Zero via `init`, but
    // it's *connected* — so this isn't "no real driver": test that
    // case with a lone pull separately, below).
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 2), Bit::Zero, "a real driver (even 0) beats the pull");

    set(&mut sim, 0, true);
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 2), Bit::One, "real driver still wins, happens to agree with pull here");
}

#[test]
fn pull_resistor_fills_in_when_truly_unconnected() {
    let mut library = HashMap::new();
    // Same shape, but nothing connects to the InputPin's output — only
    // the pull resistor drives `out`.
    library.insert(
        "main".to_string(),
        CircuitTemplate {
            name: "main".to_string(),
            nodes: vec![TemplateNode::PullResistor(Bit::One), TemplateNode::OutputPin { bits: 1 }],
            connections: vec![((0, 0, 0), (1, 0, 0))],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
        },
    );
    let netlist = flatten("main", &library).unwrap();
    let mut sim = Simulation::new(netlist);
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 1), Bit::One);
}

#[test]
fn pull_resistor_does_not_mask_a_real_short_circuit() {
    let mut library = HashMap::new();
    library.insert(
        "main".to_string(),
        CircuitTemplate {
            name: "main".to_string(),
            nodes: vec![
                TemplateNode::InputPin { bits: 1 },               // 0: a
                TemplateNode::InputPin { bits: 1 },               // 1: b
                TemplateNode::PullResistor(Bit::One), // 2
                TemplateNode::OutputPin { bits: 1 },              // 3: out
            ],
            connections: vec![((0, 0, 0), (3, 0, 0)), ((1, 0, 0), (3, 0, 0)), ((2, 0, 0), (3, 0, 0))],
            input_ports: Vec::new(),
            output_ports: Vec::new(),
            port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
        },
    );
    let netlist = flatten("main", &library).unwrap();
    let mut sim = Simulation::new(netlist);

    set(&mut sim, 0, true);
    set(&mut sim, 1, false);
    sim.run_to_quiescence();
    assert_eq!(
        get_bit(&sim, 3),
        Bit::Error,
        "a real conflict stays an error, the pull resistor doesn't paper over it"
    );
}

/// Regression test for a real bug: staggering `Simulation::new`'s
/// initial priming by dependency depth (`priming::priming_depths`)
/// instead of scheduling every gate at the same instant `t=0`. Before
/// that fix, this exact sequence — `tick()` called *before* any settle
/// has ever happened, which is exactly what a `.ctest` script does when
/// its first line is `simulate` (no leading bare settle) — corrupted
/// `Register`'s `last_clock` with a stale `Unknown` placeholder read of
/// `Clock`'s not-yet-committed output, permanently hiding the real
/// rising edge. `clock_drives_a_register_through_simulation_tick` (in
/// `compile/memory.rs`) didn't catch this because it happened to call
/// `run_to_quiescence()` once *before* the first `tick()`, which
/// incidentally let the corruption self-heal (`Unknown` -> `Zero`,
/// coincidentally the value it needed to be) before the real edge
/// occurred — this test deliberately uses the *other* (more common, via
/// `.ctest`) call order, where that lucky self-heal can't happen.
fn clock_feeds_register_circuit() -> CircuitTemplate {
    CircuitTemplate {
        name: "main".to_string(),
        nodes: vec![
            TemplateNode::Constant { bits: 1, value: 1 }, // 0: D, fixed at 1
            TemplateNode::Clock { high: 1, low: 1 },      // 1: CK, period 2
            TemplateNode::Register { bits: 1, trigger: crate::components::Trigger::Rising }, // 2
            TemplateNode::OutputPin { bits: 1 },          // 3: Q
        ],
        connections: vec![((0, 0, 0), (2, 0, 0)), ((1, 0, 0), (2, 1, 0)), ((2, 0, 0), (3, 0, 0))],
        input_ports: Vec::new(),
        output_ports: Vec::new(),
        port_marker_nodes: Vec::new(),
        component_index: HashMap::new(),
    }
}

#[test]
fn register_sees_the_first_real_clock_edge_even_without_a_leading_settle() {
    let mut library = HashMap::new();
    library.insert("main".to_string(), clock_feeds_register_circuit());
    let netlist = flatten("main", &library).unwrap();
    let mut sim = Simulation::new(netlist);

    // No `run_to_quiescence()` here on purpose — `tick()` is the very
    // first thing called, exactly like a `.ctest` script whose first
    // statement is `simulate`.
    sim.tick(); // global tick 1: high=low=1 -> period 2, 1%2=1 -> high phase -> clock rises 0->1
    sim.run_to_quiescence();
    assert_eq!(get_bit(&sim, 3), Bit::One, "the rising edge must be observed, not swallowed by a stale priming read");
}
