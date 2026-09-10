//! Headless runner (PLAN.md §4).
//!
//! `engine-cli <project.json>` — compiles + flattens the project
//! (`engine::compile`, `engine::netlist`), runs it to quiescence, and
//! reports every top-level `InputPin`/`OutputPin`'s final value — a
//! minimal `-tty`-style report.
//!
//! `engine-cli test <project.json> <script.ctest>` — runs a `.ctest`
//! script (PLAN.md §7) against the project: `component[id] -> action`,
//! `simulate`, `assert component[id] -> readout == value`. Prints every
//! failing assertion (not just the first) and exits nonzero if any failed.

use engine::compile::{compile, CompileError};
use engine::ctest;
use engine::file_format::ProjectFile;
use engine::netlist::{flatten_with_entry_map, NetlistError};
use engine::sim::Simulation;
use plugin_abi::{Bit, Value};
use std::collections::HashMap;
use std::env;
use std::fs;
use std::process::ExitCode;

fn main() -> ExitCode {
    let args: Vec<String> = env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        Some("test") => match args.get(1..3) {
            Some([project_path, script_path]) => run_test(project_path, script_path),
            _ => {
                eprintln!("usage: engine-cli test <project.json> <script.ctest>");
                ExitCode::FAILURE
            }
        },
        Some(project_path) if args.len() == 1 => run_report(project_path),
        _ => {
            eprintln!("usage: engine-cli <project.json>");
            eprintln!("       engine-cli test <project.json> <script.ctest>");
            ExitCode::FAILURE
        }
    }
}

enum LoadError {
    Io(String, std::io::Error),
    Json(String, serde_json::Error),
    Compile(CompileError),
    Flatten(NetlistError),
    MissingMainCircuit(String, usize),
}

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            LoadError::Io(path, e) => write!(f, "cannot read {path}: {e}"),
            LoadError::Json(path, e) => write!(f, "invalid project file {path}: {e}"),
            LoadError::Compile(e) => write!(f, "compile failed: {e:?}"),
            LoadError::Flatten(e) => write!(f, "flatten failed: {e:?}"),
            LoadError::MissingMainCircuit(name, count) => {
                write!(f, "main circuit {name:?} not found among {count} circuit(s)")
            }
        }
    }
}

struct Loaded {
    project: ProjectFile,
    sim: Simulation,
    entry_map: HashMap<usize, usize>,
    /// The main circuit's own `id -> local node index` map (`CircuitTemplate::
    /// component_index`) — *not* the same as position in `Circuit::
    /// components`, since a `Splitter` consumes a slot in that raw JSON
    /// array but never becomes a node (see `component_index`'s doc).
    component_index: HashMap<String, usize>,
}

fn load(project_path: &str) -> Result<Loaded, LoadError> {
    let text = fs::read_to_string(project_path).map_err(|e| LoadError::Io(project_path.to_string(), e))?;
    let project: ProjectFile = serde_json::from_str(&text).map_err(|e| LoadError::Json(project_path.to_string(), e))?;
    let library = compile(&project).map_err(LoadError::Compile)?;
    let component_index = library.get(&project.main_circuit).map(|t| t.component_index.clone()).unwrap_or_default();
    let (netlist, entry_map) = flatten_with_entry_map(&project.main_circuit, &library).map_err(LoadError::Flatten)?;
    if !project.circuits.iter().any(|c| c.name == project.main_circuit) {
        return Err(LoadError::MissingMainCircuit(project.main_circuit.clone(), project.circuits.len()));
    }
    Ok(Loaded { project, sim: Simulation::new(netlist), entry_map, component_index })
}

/// Resolves `component[id]` via the main circuit's `component_index` (its
/// JSON `id` -> local node index, as the compiler actually numbered it —
/// see `Loaded::component_index`'s doc), then through `entry_map` to a
/// global gate index.
fn resolver<'a>(component_index: &'a HashMap<String, usize>, entry_map: &'a HashMap<usize, usize>) -> impl Fn(&str) -> Option<usize> + 'a {
    move |id: &str| {
        let &local = component_index.get(id)?;
        entry_map.get(&local).copied()
    }
}

fn run_report(project_path: &str) -> ExitCode {
    let mut loaded = match load(project_path) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::FAILURE;
        }
    };
    loaded.sim.run_to_quiescence();

    let main_circuit = loaded.project.circuits.iter().find(|c| c.name == loaded.project.main_circuit).expect("checked in load()");

    println!("circuit: {}", main_circuit.name);
    for comp in main_circuit.components.iter() {
        if comp.type_ != "core:InputPin" && comp.type_ != "core:OutputPin" {
            continue;
        }
        let Some(&local_idx) = loaded.component_index.get(&comp.id) else {
            continue;
        };
        let Some(&gate) = loaded.entry_map.get(&local_idx) else {
            continue;
        };
        match loaded.sim.read(gate, "get") {
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

fn run_test(project_path: &str, script_path: &str) -> ExitCode {
    let mut loaded = match load(project_path) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::FAILURE;
        }
    };

    let script_text = match fs::read_to_string(script_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("error: cannot read {script_path}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let script = match ctest::parse(&script_text) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("error: {script_path}: {e}");
            return ExitCode::FAILURE;
        }
    };

    let resolve = resolver(&loaded.component_index, &loaded.entry_map);
    let report = ctest::run(&script, &mut loaded.sim, resolve);

    println!("{script_path}: {} assert(s), {} failure(s)", report.asserts_run, report.failures.len());
    for failure in &report.failures {
        println!("  line {}: {}", failure.line, failure.message);
    }

    if report.passed() {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
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
