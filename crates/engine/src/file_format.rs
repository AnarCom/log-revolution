//! Native project file format (JSON): the on-disk, visual schema —
//! positions, wires, hierarchy. Never the compiled netlist (see PLAN.md §9).
//!
//! Deliberately stops at the *envelope*: it says nothing about which
//! component types exist or what attributes each one takes — `type` and
//! `attrs` are opaque here. Enumerating the standard component library is
//! Phase 2 work (PLAN.md §10) and lives with each component's own
//! implementation, not in the file format.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Component/annotation attribute value — intentionally untyped at this
/// layer; each component implementation interprets its own `attrs`.
pub type AttrValue = serde_json::Value;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProjectFile {
    pub schema_version: u32,
    pub main_circuit: String,
    pub circuits: Vec<Circuit>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Circuit {
    pub name: String,
    #[serde(default)]
    pub components: Vec<ComponentInstance>,
    #[serde(default)]
    pub wires: Vec<WireSegment>,
    #[serde(default)]
    pub annotations: Vec<Annotation>,
}

/// One placed instance of *something* — a built-in component, a subcircuit
/// (`type: "core:circuit/<name>"`), or a plugin component
/// (`type: "plugin:<plugin-id>/<component-id>"`). The namespacing convention
/// is fixed now because it's an addressing scheme, not a library entry; the
/// set of valid `core:` type strings is not.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ComponentInstance {
    pub id: String,
    #[serde(rename = "type")]
    pub type_: String,
    pub x: i32,
    pub y: i32,
    #[serde(default)]
    pub facing: Facing,
    #[serde(default)]
    pub attrs: BTreeMap<String, AttrValue>,
}

/// Mirrors Logisim's own orientation model (N/S/E/W), not free rotation —
/// needed to reproduce `.circ` import semantics faithfully (PLAN.md §9).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Facing {
    #[default]
    East,
    West,
    North,
    South,
}

/// A single two-point segment, mirroring `.circ`'s own wire model —
/// junctions are implicit at shared endpoints, not a separate entity.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WireSegment {
    pub id: String,
    pub from: [i32; 2],
    pub to: [i32; 2],
}

/// Non-simulated editor annotations (text labels, etc.) — kept separate
/// from `components` since they never have pins and never enter the
/// netlist. `kind` stays an open string for the same reason `type` does
/// above.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Annotation {
    pub id: String,
    pub kind: String,
    pub x: i32,
    pub y: i32,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> ProjectFile {
        let mut attrs = BTreeMap::new();
        attrs.insert("bitWidth".to_string(), serde_json::json!(1));

        ProjectFile {
            schema_version: 1,
            main_circuit: "main".to_string(),
            circuits: vec![Circuit {
                name: "main".to_string(),
                components: vec![ComponentInstance {
                    id: "c1".to_string(),
                    type_: "core:AndGate".to_string(),
                    x: 100,
                    y: 200,
                    facing: Facing::East,
                    attrs,
                }],
                wires: vec![WireSegment {
                    id: "w1".to_string(),
                    from: [100, 200],
                    to: [140, 200],
                }],
                annotations: vec![Annotation {
                    id: "a1".to_string(),
                    kind: "label".to_string(),
                    x: 100,
                    y: 180,
                    text: Some("adder".to_string()),
                }],
            }],
        }
    }

    #[test]
    fn round_trips_through_json() {
        let original = sample();
        let json = serde_json::to_string_pretty(&original).unwrap();
        let parsed: ProjectFile = serde_json::from_str(&json).unwrap();
        assert_eq!(original, parsed);
    }

    #[test]
    fn defaults_are_optional_in_input() {
        let minimal = serde_json::json!({
            "schemaVersion": 1,
            "mainCircuit": "main",
            "circuits": [{ "name": "main" }]
        });
        let parsed: ProjectFile = serde_json::from_value(minimal).unwrap();
        assert!(parsed.circuits[0].components.is_empty());
        assert_eq!(parsed.circuits[0].components.first().is_none(), true);
    }
}
