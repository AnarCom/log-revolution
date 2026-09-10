//! Compiles the `Splitter`: unlike every other component, it produces no
//! `TemplateNode`/`Gate` at all. Real Logisim's own `Splitter.java` doesn't
//! `propagate()` anything either (`; // handled by CircuitWires, nothing to
//! do`) — a splitter is pure wire fusion between specific *bits* of its
//! combined end and specific bits of each fanout end, computed once in
//! `configureComponent` (`bit_end`/`bit_thread`) and applied structurally by
//! `CircuitWires`'s "unite threads going through splitters" pass, not
//! simulated as a stateful/delayed component. So this file doesn't return a
//! `(TemplateNode, Geometry)` pair like every other `compile::*::compile` —
//! it returns a `SplitterWiring`, consumed directly by
//! `compile_circuit`'s bit-lane union-find and never becoming a netlist
//! node.
//!
//! Attribute keys match `SplitterAttributes.java` verbatim: `"incoming"`
//! (`ATTR_WIDTH`, the combined end's width, default 2 — `Attributes.
//! forBitWidth("incoming", ..)`), `"fanout"` (`ATTR_FANOUT`, 1..=32,
//! default 2 — `Attributes.forIntegerRange("fanout", .., 1, 32)`). `"bits"`
//! is *not* a real Logisim attribute name — Logisim spreads per-bit
//! assignment across individual `bit0`, `bit1`, ... attributes for its own
//! per-bit Swing combo-box UI (`SplitterAttributes.BitOutAttribute`); this
//! schema instead encodes the exact same `bit_end` array as one JSON array
//! (index = combined bit, value = 1-based fanout end, `0` = unconnected/
//! Logisim's "none"), since nothing here has that UI constraint to satisfy.
//! Omitted, it defaults to `SplitterAttributes.computeDistribution` with
//! `order = 1` (contiguous ranges, assigned to ends in order) — Logisim's
//! own default the moment a `Splitter` is placed, verified line-by-line
//! against the Java source below (`default_bit_end`).

use super::{rotate, CompileError, Point};
use crate::file_format::{Circuit, ComponentInstance};

/// A `Splitter` instance's resolved wiring, in this circuit's own local
/// (unrotated-to-global-index) coordinate space — consumed only by
/// `compile_circuit`'s union-find, one bit lane at a time.
pub(super) struct SplitterWiring {
    pub(super) combined_point: Point,
    /// Length `fanout`; index `0` is end `1` (Logisim's ends are 1-based,
    /// `0` being reserved for "no end" in `bit_end`).
    pub(super) fanout_points: Vec<Point>,
    /// Length `incoming`; `0` means that combined bit isn't wired to any
    /// fanout end at all (Logisim's "none"), else `1..=fanout`.
    pub(super) bit_end: Vec<u8>,
    /// Length `incoming`; `bit_thread[i]` is bit `i`'s position *within*
    /// whatever end `bit_end[i]` names (only meaningful where `bit_end[i] >
    /// 0` — `compile_circuit` only ever reads it there).
    pub(super) bit_thread: Vec<u8>,
}

fn fanout_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<u8, CompileError> {
    let raw = comp.attrs.get("fanout").and_then(|v| v.as_i64()).unwrap_or(2);
    if (1..=32).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidFanout { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

fn incoming_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<u8, CompileError> {
    let raw = comp.attrs.get("incoming").and_then(|v| v.as_i64()).unwrap_or(2);
    if (1..=32).contains(&raw) {
        Ok(raw as u8)
    } else {
        Err(CompileError::InvalidWidth { circuit: circuit.name.clone(), id: comp.id.clone(), value: raw })
    }
}

/// `SplitterAttributes.computeDistribution(fanout, bits, order = 1)`: when
/// `fanout >= bits`, bit `i` goes to end `i+1` alone (one bit per end; ends
/// beyond `bits` just end up width-0); otherwise contiguous ranges are
/// handed to ends in order, front-loaded with the remainder
/// (`bits % fanout` extra bits go to the first ends, one each). Verified
/// line-by-line against the Java source (the `order >= 0` branch only —
/// this schema has no UI notion of the descending `order < 0` variant,
/// which only exists to mirror a "distribute selected bits" menu command).
fn default_bit_end(fanout: u8, bits: u8) -> Vec<u8> {
    let (fanout, bits) = (fanout as usize, bits as usize);
    let mut ret = vec![0u8; bits];
    if fanout >= bits {
        for (i, slot) in ret.iter_mut().enumerate() {
            *slot = (i + 1) as u8;
        }
    } else {
        let threads_per_end = bits / fanout;
        let mut ends_with_extra = bits % fanout;
        let mut cur_end: i32 = -1;
        let mut left_in_end = 0usize;
        for slot in ret.iter_mut() {
            if left_in_end == 0 {
                cur_end += 1;
                left_in_end = threads_per_end;
                if ends_with_extra > 0 {
                    left_in_end += 1;
                    ends_with_extra -= 1;
                }
            }
            *slot = (1 + cur_end) as u8;
            left_in_end -= 1;
        }
    }
    ret
}

/// `attrs["bits"]` — this schema's own array encoding of `bit_end` (see
/// this module's doc comment). Falls back to `default_bit_end` when absent.
fn bit_end_attr(circuit: &Circuit, comp: &ComponentInstance, fanout: u8, incoming: u8) -> Result<Vec<u8>, CompileError> {
    let invalid = |reason: String| CompileError::InvalidSplitterBits { circuit: circuit.name.clone(), id: comp.id.clone(), reason };
    match comp.attrs.get("bits").and_then(|v| v.as_array()) {
        None => Ok(default_bit_end(fanout, incoming)),
        Some(arr) => {
            if arr.len() != incoming as usize {
                return Err(invalid(format!("expected {incoming} entries, got {}", arr.len())));
            }
            arr.iter()
                .map(|v| {
                    let n = v.as_i64().ok_or_else(|| invalid("entries must be integers".to_string()))?;
                    if (0..=fanout as i64).contains(&n) {
                        Ok(n as u8)
                    } else {
                        Err(invalid(format!("entry {n} outside 0..={fanout}")))
                    }
                })
                .collect()
        }
    }
}

/// `Splitter.configureComponent`'s `bit_thread` computation: bit `i`'s
/// position within its assigned end is a running per-end count over
/// increasing `i` (so `bit_thread` only ever depends on `bit_end`, nothing
/// else — same as the Java source, which derives both from `bit_end` in one
/// pass).
fn bit_thread_for(bit_end: &[u8]) -> Vec<u8> {
    let mut end_width = [0u8; 33]; // ends are 1..=32
    bit_end
        .iter()
        .map(|&thr| {
            if thr > 0 {
                let t = end_width[thr as usize];
                end_width[thr as usize] += 1;
                t
            } else {
                0
            }
        })
        .collect()
}

/// Arbitrary, mutually distinct offsets (nothing renders this yet — same
/// rationale as every other `compile::*::*_geometry`): the combined end at
/// the component's own origin, fanout ends spread out east of it.
fn fanout_offsets(fanout: u8) -> Vec<Point> {
    (0..fanout as i32).map(|k| (3, 2 * k)).collect()
}

pub(super) fn compile(circuit: &Circuit, comp: &ComponentInstance) -> Result<SplitterWiring, CompileError> {
    let fanout = fanout_attr(circuit, comp)?;
    let incoming = incoming_attr(circuit, comp)?;
    let bit_end = bit_end_attr(circuit, comp, fanout, incoming)?;
    let bit_thread = bit_thread_for(&bit_end);

    let abs = |o: Point| {
        let (rx, ry) = rotate(o, comp.facing);
        (comp.x + rx, comp.y + ry)
    };
    let combined_point = abs((0, 0));
    let fanout_points = fanout_offsets(fanout).into_iter().map(abs).collect();

    Ok(SplitterWiring { combined_point, fanout_points, bit_end, bit_thread })
}

#[cfg(test)]
mod tests {
    use super::super::test_support::*;
    use super::*;
    use crate::compile::{compile, CompileError};
    use crate::file_format::Circuit;
    use crate::netlist::flatten;
    use crate::sim::Simulation;
    use plugin_abi::{Bit, Value};
    use serde_json::json;
    use std::collections::BTreeMap;

    #[test]
    fn default_distribution_matches_logisim_one_bit_per_end_when_fanout_covers_every_bit() {
        // fanout=4, incoming=4 (fanout >= bits): bit i -> end i+1.
        assert_eq!(default_bit_end(4, 4), vec![1, 2, 3, 4]);
    }

    #[test]
    fn default_distribution_matches_logisim_contiguous_chunks_when_fanout_is_narrower() {
        // fanout=2, incoming=8: 4 bits per end, no remainder.
        assert_eq!(default_bit_end(2, 8), vec![1, 1, 1, 1, 2, 2, 2, 2]);
        // fanout=3, incoming=8: 8 = 2*3 + 2, so ends 1,2 get an extra bit.
        assert_eq!(default_bit_end(3, 8), vec![1, 1, 1, 2, 2, 2, 3, 3]);
    }

    #[test]
    fn bit_thread_counts_position_within_each_end() {
        assert_eq!(bit_thread_for(&[1, 1, 2, 1, 2]), vec![0, 1, 0, 2, 1]);
    }

    /// An 8-bit combined bus split into two 4-bit halves (default
    /// distribution: bits 0-3 -> end 1, bits 4-7 -> end 2), each half read
    /// out through its own `OutputPin` — proves the fanout ends carry the
    /// *right* bits, not just *some* bits.
    #[test]
    fn splits_a_bus_into_two_contiguous_halves() {
        let mut wide = BTreeMap::new();
        wide.insert("width".to_string(), json!(8));
        let mut half = BTreeMap::new();
        half.insert("width".to_string(), json!(4));
        let mut spl = BTreeMap::new();
        spl.insert("incoming".to_string(), json!(8));
        spl.insert("fanout".to_string(), json!(2));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: wide.clone(), ..comp("src", "core:InputPin", 0, 0) },
                ComponentInstance { attrs: spl, ..comp("spl", "core:Splitter", 0, 0) },
                ComponentInstance { attrs: half.clone(), ..comp("lo", "core:OutputPin", 3, 0) },
                ComponentInstance { attrs: half, ..comp("hi", "core:OutputPin", 3, 2) },
            ],
            // src(0,0) -> spl's combined end (0,0); spl's fanout end 1 is at
            // (3,0) (`fanout_offsets`'s first entry) -> lo(3,0); end 2 is at
            // (3,2) -> hi(3,2).
            wires: vec![wire("w1", [0, 0], [0, 0]), wire("w2", [3, 0], [3, 0]), wire("w3", [3, 2], [3, 2])],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b1011_0010))).unwrap(); // src
        sim.run_to_quiescence();
        // Little-endian bit order (LSB first): low nibble 0010 -> lo,
        // high nibble 1011 -> hi. Gate indices: src=0, `spl` materializes no
        // node at all (see this module's doc comment), so lo=1, hi=2.
        assert_eq!(get_bits(&sim, 1), vec![Bit::Zero, Bit::One, Bit::Zero, Bit::Zero], "lo = low nibble");
        assert_eq!(get_bits(&sim, 2), vec![Bit::One, Bit::One, Bit::Zero, Bit::One], "hi = high nibble");
    }

    /// The mirror of the split above — two independent nibbles wired
    /// *into* the fanout ends combine into one 8-bit word read off the
    /// combined end, proving a splitter is genuinely bidirectional wire
    /// fusion (no `Gate`, no fixed "which side drives"), not a disguised
    /// unidirectional demux.
    #[test]
    fn joins_two_nibbles_into_one_bus_through_the_combined_end() {
        let mut wide = BTreeMap::new();
        wide.insert("width".to_string(), json!(8));
        let mut half = BTreeMap::new();
        half.insert("width".to_string(), json!(4));
        let mut spl = BTreeMap::new();
        spl.insert("incoming".to_string(), json!(8));
        spl.insert("fanout".to_string(), json!(2));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: half.clone(), ..comp("lo", "core:InputPin", 3, 0) },
                ComponentInstance { attrs: half, ..comp("hi", "core:InputPin", 3, 2) },
                ComponentInstance { attrs: spl, ..comp("spl", "core:Splitter", 0, 0) },
                ComponentInstance { attrs: wide, ..comp("out", "core:OutputPin", 0, 0) },
            ],
            wires: vec![wire("w1", [3, 0], [3, 0]), wire("w2", [3, 2], [3, 2]), wire("w3", [0, 0], [0, 0])],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);

        sim.invoke(0, "set", Some(Value::Int(0b0010))).unwrap(); // lo
        sim.invoke(1, "set", Some(Value::Int(0b1011))).unwrap(); // hi
        sim.run_to_quiescence();
        // Gate indices: lo=0, hi=1, `spl` materializes no node, out=2.
        assert_eq!(
            get_bits(&sim, 2),
            vec![Bit::Zero, Bit::One, Bit::Zero, Bit::Zero, Bit::One, Bit::One, Bit::Zero, Bit::One],
            "out = hi:lo = 1011_0010"
        );
    }

    /// A bit whose `bit_end` entry is `0` ("none") is wired to nothing —
    /// reads as floating (`Unknown`), not silently zero.
    #[test]
    fn an_unconnected_bit_reads_as_floating() {
        let mut w2 = BTreeMap::new();
        w2.insert("width".to_string(), json!(2));
        let mut spl = BTreeMap::new();
        spl.insert("incoming".to_string(), json!(2));
        spl.insert("fanout".to_string(), json!(1));
        spl.insert("bits".to_string(), json!([1, 0])); // bit 0 -> end 1, bit 1 -> none

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: w2.clone(), ..comp("src", "core:InputPin", 0, 0) },
                ComponentInstance { attrs: spl, ..comp("spl", "core:Splitter", 0, 0) },
                comp("out", "core:OutputPin", 3, 0), // width 1, bit 0 only
            ],
            wires: vec![wire("w1", [0, 0], [0, 0]), wire("w2", [3, 0], [3, 0])],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.invoke(0, "set", Some(Value::Int(0b11))).unwrap();
        sim.run_to_quiescence();
        // Gate indices: src=0, `spl` materializes no node, out=1.
        assert_eq!(get_bit(&sim, 1), Bit::One, "bit 0 (bit_end=1) reaches the fanout end");
    }

    #[test]
    fn rejects_fanout_above_thirty_two() {
        let mut attrs = BTreeMap::new();
        attrs.insert("fanout".to_string(), json!(33));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("spl", "core:Splitter", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidFanout { circuit: "main".to_string(), id: "spl".to_string(), value: 33 }
        );
    }

    #[test]
    fn rejects_a_bits_array_of_the_wrong_length() {
        let mut attrs = BTreeMap::new();
        attrs.insert("incoming".to_string(), json!(4));
        attrs.insert("bits".to_string(), json!([1, 2]));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("spl", "core:Splitter", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert!(matches!(compile(&project).unwrap_err(), CompileError::InvalidSplitterBits { .. }));
    }
}
