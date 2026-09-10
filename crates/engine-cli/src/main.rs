//! Headless runner (PLAN.md §4): loads a `.json` project file, compiles +
//! flattens it (`engine::compile`, `engine::netlist`), runs it to
//! quiescence, and reports every top-level `InputPin`/`OutputPin`'s final
//! value — a first, minimal `-tty`-style report, not yet driven by a
//! `.ctest` script (PLAN.md §7 — that's the next piece, once this exists
//! for it to run against).

use engine::compile::compile;
use engine::file_format::ProjectFile;
use engine::netlist::flatten_with_entry_map;
use engine::sim::Simulation;
use plugin_abi::{Bit, Value};
use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let Some(path) = env::args().nth(1) else {
        eprintln!("usage: engine-cli <project.json>");
        return ExitCode::FAILURE;
    };

    let text = match fs::read_to_string(&path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("error: cannot read {path}: {e}");
            return ExitCode::FAILURE;
        }
    };

    let project: ProjectFile = match serde_json::from_str(&text) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("error: invalid project file: {e}");
            return ExitCode::FAILURE;
        }
    };

    let library = match compile(&project) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("error: compile failed: {e:?}");
            return ExitCode::FAILURE;
        }
    };

    let (netlist, entry_map) = match flatten_with_entry_map(&project.main_circuit, &library) {
        Ok(n) => n,
        Err(e) => {
            eprintln!("error: flatten failed: {e:?}");
            return ExitCode::FAILURE;
        }
    };

    let Some(main_circuit) = project.circuits.iter().find(|c| c.name == project.main_circuit) else {
        eprintln!("error: main circuit {:?} not found among {} circuit(s)", project.main_circuit, project.circuits.len());
        return ExitCode::FAILURE;
    };

    let mut sim = Simulation::new(netlist);
    sim.run_to_quiescence();

    println!("circuit: {}", main_circuit.name);
    for (local_idx, comp) in main_circuit.components.iter().enumerate() {
        if comp.type_ != "core:InputPin" && comp.type_ != "core:OutputPin" {
            continue;
        }
        let Some(&gate) = entry_map.get(&local_idx) else {
            continue;
        };
        match sim.read(gate, "get") {
            Ok(Value::Bits(bits)) => {
                let value: String = bits.iter().map(|b| bit_char(*b)).collect();
                println!("  {} ({}) = {}", comp.id, comp.type_, value);
            }
            Ok(other) => println!("  {} ({}) = {other:?}", comp.id, comp.type_),
            Err(e) => eprintln!("  {} ({}): read failed: {e:?}", comp.id, comp.type_),
        }
    }

    ExitCode::SUCCESS
}

/// Same single-character convention as `logisim-port`'s own `Value.toString`
/// (`0`/`1`/`x`/`E`) — not our own invention, matches the oracle.
fn bit_char(b: Bit) -> char {
    match b {
        Bit::Zero => '0',
        Bit::One => '1',
        Bit::Unknown => 'x',
        Bit::Error => 'E',
    }
}
