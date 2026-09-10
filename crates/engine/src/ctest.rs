//! `.ctest` — the small, closed test-script DSL from PLAN.md §7:
//!
//! ```text
//! component[sw_a] -> on
//! component[sw_b] -> set(0x05)
//! simulate
//! assert component[out] -> get == 0x08
//! ```
//!
//! Deliberately not Turing-complete (no loops, no arbitrary code) — a
//! documented safety choice (§7/§12), not a missing feature: these scripts
//! are meant to be safe to run automatically against untrusted (e.g.
//! student-submitted) circuits without plugin-level sandboxing.
//!
//! One file = one flat list of statements, executed top to bottom. A
//! failing `assert` does not stop the run — every failure is collected,
//! because the point is grading feedback ("here's everything wrong"), not
//! just a pass/fail bit.

use crate::sim::Simulation;
use plugin_abi::{Bit, Value};
use std::fmt;

#[derive(Debug, Clone, PartialEq)]
pub enum Literal {
    Hex(u64),
    Bin(u64),
    Dec(u64),
    Bool(bool),
}

impl Literal {
    /// For `component[x] -> action(<literal>)` — what `Simulation::invoke`
    /// actually receives. Hex/Bin/Dec all become `Value::Int`; the gate's
    /// own `invoke` decides what to do with it (e.g. `Gate::InputPin`
    /// treats nonzero as `Bit::One`).
    fn to_action_arg(&self) -> Value {
        match self {
            Literal::Hex(n) | Literal::Bin(n) | Literal::Dec(n) => Value::Int(*n as i64),
            Literal::Bool(b) => Value::Bool(*b),
        }
    }
}

impl fmt::Display for Literal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Literal::Hex(n) => write!(f, "0x{n:x}"),
            Literal::Bin(n) => write!(f, "0b{n:b}"),
            Literal::Dec(n) => write!(f, "{n}"),
            Literal::Bool(b) => write!(f, "{b}"),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cmp {
    Eq,
    Ne,
}

#[derive(Debug, Clone, PartialEq)]
pub enum Statement {
    /// `component[id] -> action` or `component[id] -> action(arg)`.
    Action { component: String, action: String, arg: Option<Literal> },
    /// `simulate` (settle) or `simulate <n>` — see `run`'s doc comment on
    /// what `n` means today (not much, yet).
    Simulate { ticks: Option<u32> },
    /// `assert component[id] -> readout <cmp> <literal>`.
    Assert { component: String, readout: String, cmp: Cmp, expected: Literal },
}

#[derive(Debug, Clone, PartialEq)]
pub struct ParseError {
    pub line: usize,
    pub message: String,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "line {}: {}", self.line, self.message)
    }
}

pub fn parse(source: &str) -> Result<Vec<(usize, Statement)>, ParseError> {
    let mut statements = Vec::new();
    for (idx, raw_line) in source.lines().enumerate() {
        let line_no = idx + 1;
        let line = strip_comment(raw_line).trim();
        if line.is_empty() {
            continue;
        }
        statements.push((line_no, parse_statement(line, line_no)?));
    }
    Ok(statements)
}

fn strip_comment(line: &str) -> &str {
    match line.find('#') {
        Some(i) => &line[..i],
        None => line,
    }
}

fn parse_statement(line: &str, line_no: usize) -> Result<Statement, ParseError> {
    let err = |message: &str| ParseError { line: line_no, message: message.to_string() };

    if let Some(rest) = line.strip_prefix("simulate") {
        let rest = rest.trim();
        let ticks = if rest.is_empty() {
            None
        } else {
            Some(rest.parse::<u32>().map_err(|_| err("expected an integer tick count after `simulate`"))?)
        };
        return Ok(Statement::Simulate { ticks });
    }

    if let Some(rest) = line.strip_prefix("assert") {
        let rest = rest.trim();
        let (component, readout, rest) = parse_component_arrow(rest, line_no)?;
        let rest = rest.trim();
        let (cmp, rest) = if let Some(r) = rest.strip_prefix("==") {
            (Cmp::Eq, r)
        } else if let Some(r) = rest.strip_prefix("!=") {
            (Cmp::Ne, r)
        } else {
            return Err(err("expected `==` or `!=` in assert"));
        };
        let expected = parse_literal(rest.trim(), line_no)?;
        return Ok(Statement::Assert { component, readout: readout.to_string(), cmp, expected });
    }

    let (component, action, trailer) = parse_component_arrow(line, line_no)?;
    let arg = parse_action_arg(trailer.trim(), line_no)?;
    Ok(Statement::Action { component, action: action.to_string(), arg })
}

/// Parses the common `component[<id>] -> <rest-of-line>` prefix, returning
/// the id, the identifier immediately after `->` (action or readout name),
/// and whatever trails after *that* identifier — an assert's `<cmp> <value>`,
/// or an action's optional `(<value>)`.
fn parse_component_arrow<'a>(input: &'a str, line_no: usize) -> Result<(String, &'a str, &'a str), ParseError> {
    let err = |message: &str| ParseError { line: line_no, message: message.to_string() };

    let input = input.trim();
    let input = input.strip_prefix("component").ok_or_else(|| err("expected `component[...]`"))?.trim_start();
    let input = input.strip_prefix('[').ok_or_else(|| err("expected `[` after `component`"))?;
    let close = input.find(']').ok_or_else(|| err("missing closing `]`"))?;
    let id = input[..close].trim();
    if id.is_empty() {
        return Err(err("component id must not be empty"));
    }
    let rest = input[close + 1..].trim_start();
    let rest = rest.strip_prefix("->").ok_or_else(|| err("expected `->` after `component[...]`"))?.trim_start();

    let ident_end = rest
        .char_indices()
        .find(|(_, c)| !(c.is_ascii_alphanumeric() || *c == '_'))
        .map(|(i, _)| i)
        .unwrap_or(rest.len());
    if ident_end == 0 {
        return Err(err("expected an action/readout name after `->`"));
    }
    Ok((id.to_string(), &rest[..ident_end], &rest[ident_end..]))
}

/// `trailer` is whatever followed the action name — either empty (no-arg
/// action, e.g. `on`) or `(<literal>)` (e.g. `set(0x05)`).
fn parse_action_arg(trailer: &str, line_no: usize) -> Result<Option<Literal>, ParseError> {
    if trailer.is_empty() {
        return Ok(None);
    }
    let inner = trailer.strip_prefix('(').and_then(|s| s.strip_suffix(')')).ok_or_else(|| ParseError {
        line: line_no,
        message: "expected `(<value>)` after action name, or nothing".to_string(),
    })?;
    Ok(Some(parse_literal(inner.trim(), line_no)?))
}

fn parse_literal(input: &str, line_no: usize) -> Result<Literal, ParseError> {
    let err = || ParseError { line: line_no, message: format!("invalid literal `{input}`") };
    if let Some(hex) = input.strip_prefix("0x").or_else(|| input.strip_prefix("0X")) {
        return u64::from_str_radix(hex, 16).map(Literal::Hex).map_err(|_| err());
    }
    if let Some(bin) = input.strip_prefix("0b").or_else(|| input.strip_prefix("0B")) {
        return u64::from_str_radix(bin, 2).map(Literal::Bin).map_err(|_| err());
    }
    match input {
        "on" | "true" => return Ok(Literal::Bool(true)),
        "off" | "false" => return Ok(Literal::Bool(false)),
        _ => {}
    }
    input.parse::<u64>().map(Literal::Dec).map_err(|_| err())
}

#[derive(Debug, Clone, PartialEq)]
pub struct Failure {
    pub line: usize,
    pub message: String,
}

#[derive(Debug, Default)]
pub struct RunReport {
    pub asserts_run: usize,
    pub failures: Vec<Failure>,
}

impl RunReport {
    pub fn passed(&self) -> bool {
        self.failures.is_empty()
    }
}

/// Runs a parsed script against `sim`, resolving `component[id]` through
/// `resolve`. `simulate <n>` currently just calls `run_to_quiescence()` —
/// there's no clocked component yet for "n ticks" to mean anything more
/// specific than "settle, n times" (harmless once already quiescent); the
/// open question of what `n` means with multiple independent clocks
/// (PLAN.md §14) is unresolved and deliberately not guessed at here.
pub fn run(
    script: &[(usize, Statement)],
    sim: &mut Simulation,
    resolve: impl Fn(&str) -> Option<usize>,
) -> RunReport {
    let mut report = RunReport::default();

    for (line, statement) in script {
        match statement {
            Statement::Simulate { ticks } => {
                for _ in 0..ticks.unwrap_or(1).max(1) {
                    sim.run_to_quiescence();
                }
            }
            Statement::Action { component, action, arg } => {
                let Some(gate) = resolve(component) else {
                    report.failures.push(Failure {
                        line: *line,
                        message: format!("unknown component `{component}`"),
                    });
                    continue;
                };
                if let Err(e) = sim.invoke(gate, action, arg.as_ref().map(Literal::to_action_arg)) {
                    report.failures.push(Failure {
                        line: *line,
                        message: format!("component[{component}] -> {action}: {e:?}"),
                    });
                }
            }
            Statement::Assert { component, readout, cmp, expected } => {
                report.asserts_run += 1;
                let Some(gate) = resolve(component) else {
                    report.failures.push(Failure {
                        line: *line,
                        message: format!("unknown component `{component}`"),
                    });
                    continue;
                };
                let actual = match sim.read(gate, readout) {
                    Ok(v) => v,
                    Err(e) => {
                        report.failures.push(Failure {
                            line: *line,
                            message: format!("component[{component}] -> {readout}: {e:?}"),
                        });
                        continue;
                    }
                };
                let matched = value_matches(&actual, expected);
                let ok = match cmp {
                    Cmp::Eq => matched,
                    Cmp::Ne => !matched,
                };
                if !ok {
                    let op = match cmp {
                        Cmp::Eq => "==",
                        Cmp::Ne => "!=",
                    };
                    report.failures.push(Failure {
                        line: *line,
                        message: format!(
                            "assert component[{component}] -> {readout} {op} {expected}: got {}",
                            describe(&actual)
                        ),
                    });
                }
            }
        }
    }

    report
}

fn value_matches(actual: &Value, expected: &Literal) -> bool {
    match (actual, expected) {
        (Value::Bool(b), Literal::Bool(e)) => b == e,
        (Value::Bool(b), _) => bits_from_bool(*b).is_some_and(|n| literal_as_u64(expected) == Some(n)),
        (Value::Int(i), _) => literal_as_u64(expected).is_some_and(|n| *i == n as i64),
        (Value::Bits(bits), Literal::Bool(e)) => bits.len() == 1 && (bits[0] == Bit::One) == *e,
        (Value::Bits(bits), _) => match (bits_to_u64(bits), literal_as_u64(expected)) {
            (Some(a), Some(b)) => a == b,
            _ => false,
        },
    }
}

fn literal_as_u64(lit: &Literal) -> Option<u64> {
    match lit {
        Literal::Hex(n) | Literal::Bin(n) | Literal::Dec(n) => Some(*n),
        Literal::Bool(b) => Some(*b as u64),
    }
}

fn bits_from_bool(b: bool) -> Option<u64> {
    Some(b as u64)
}

/// Little-endian: `bits[0]` is the least significant bit. `None` if any
/// bit is `Unknown`/`Error` — an indeterminate signal can't equal a
/// concrete number, and pretending otherwise would hide exactly the kind
/// of bug `Bit`'s four states exist to catch (PLAN.md §9).
fn bits_to_u64(bits: &[Bit]) -> Option<u64> {
    let mut value = 0u64;
    for (i, b) in bits.iter().enumerate() {
        match b {
            Bit::One => value |= 1 << i,
            Bit::Zero => {}
            Bit::Unknown | Bit::Error => return None,
        }
    }
    Some(value)
}

fn describe(v: &Value) -> String {
    match v {
        Value::Bool(b) => b.to_string(),
        Value::Int(i) => i.to_string(),
        Value::Bits(bits) => bits
            .iter()
            .rev()
            .map(|b| match b {
                Bit::Zero => '0',
                Bit::One => '1',
                Bit::Unknown => 'x',
                Bit::Error => 'E',
            })
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::compile::compile;
    use crate::file_format::{Circuit, ComponentInstance, Facing, ProjectFile, WireSegment};
    use crate::netlist::flatten_with_entry_map;
    use std::collections::BTreeMap;

    fn comp(id: &str, type_: &str, x: i32, y: i32) -> ComponentInstance {
        ComponentInstance { id: id.to_string(), type_: type_.to_string(), x, y, facing: Facing::East, attrs: BTreeMap::new() }
    }
    fn wire(id: &str, at: [i32; 2]) -> WireSegment {
        WireSegment { id: id.to_string(), from: at, to: at }
    }

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
                wires: vec![wire("w1", [0, -1]), wire("w2", [0, 1]), wire("w3", [3, 0])],
                annotations: vec![],
            }],
        }
    }

    fn setup() -> (Simulation, std::collections::HashMap<usize, usize>, Vec<ComponentInstance>) {
        let project = and_project();
        let library = compile(&project).unwrap();
        let (netlist, entry_map) = flatten_with_entry_map(&project.main_circuit, &library).unwrap();
        let sim = Simulation::new(netlist);
        let components = project.circuits[0].components.clone();
        (sim, entry_map, components)
    }

    fn resolver<'a>(
        entry_map: &'a std::collections::HashMap<usize, usize>,
        components: &'a [ComponentInstance],
    ) -> impl Fn(&str) -> Option<usize> + 'a {
        move |id: &str| {
            let local = components.iter().position(|c| c.id == id)?;
            entry_map.get(&local).copied()
        }
    }

    #[test]
    fn parses_the_readme_example() {
        let script = "component[sw_a] -> on\ncomponent[sw_b] -> set(0x05)\nsimulate\nassert component[out] -> get == 0x08\n";
        let parsed = parse(script).unwrap();
        assert_eq!(parsed.len(), 4);
        assert_eq!(
            parsed[0].1,
            Statement::Action { component: "sw_a".to_string(), action: "on".to_string(), arg: None }
        );
        assert_eq!(
            parsed[1].1,
            Statement::Action {
                component: "sw_b".to_string(),
                action: "set".to_string(),
                arg: Some(Literal::Hex(5))
            }
        );
        assert_eq!(parsed[2].1, Statement::Simulate { ticks: None });
        assert_eq!(
            parsed[3].1,
            Statement::Assert {
                component: "out".to_string(),
                readout: "get".to_string(),
                cmp: Cmp::Eq,
                expected: Literal::Hex(8)
            }
        );
    }

    #[test]
    fn comments_and_blank_lines_are_ignored() {
        let script = "# a comment\n\ncomponent[a] -> on  # trailing comment\n";
        let parsed = parse(script).unwrap();
        assert_eq!(parsed.len(), 1);
    }

    #[test]
    fn runs_a_passing_script_end_to_end() {
        let (mut sim, entry_map, components) = setup();
        let script = parse("component[a] -> on\ncomponent[b] -> on\nsimulate\nassert component[out] -> get == 1\n").unwrap();
        let report = run(&script, &mut sim, resolver(&entry_map, &components));
        assert!(report.passed(), "{:?}", report.failures);
        assert_eq!(report.asserts_run, 1);
    }

    #[test]
    fn collects_multiple_failures_instead_of_stopping_at_the_first() {
        let (mut sim, entry_map, components) = setup();
        let script = parse(
            "component[a] -> on\ncomponent[b] -> off\nsimulate\nassert component[out] -> get == 1\nassert component[out] -> get == 0x1\n",
        )
        .unwrap();
        let report = run(&script, &mut sim, resolver(&entry_map, &components));
        assert_eq!(report.asserts_run, 2);
        assert_eq!(report.failures.len(), 2, "{:?}", report.failures);
    }

    #[test]
    fn unknown_component_is_a_failure_not_a_panic() {
        let (mut sim, entry_map, components) = setup();
        let script = parse("assert component[nope] -> get == 0\n").unwrap();
        let report = run(&script, &mut sim, resolver(&entry_map, &components));
        assert_eq!(report.failures.len(), 1);
        assert!(report.failures[0].message.contains("unknown component"));
    }
}
