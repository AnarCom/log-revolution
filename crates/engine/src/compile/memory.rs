//! Compiles the clocked components: Clock/Register.

use super::{source_geometry, width_attr, CompileError, Geometry};
use crate::components::Trigger;
use crate::file_format::{Circuit, ComponentInstance};
use crate::netlist::TemplateNode;

/// `Register`'s 4 fixed inputs, in the exact order
/// `components::memory::eval_memory` expects them (`d`, `ck`, `clr`, `en` —
/// see `Gate::input_count`'s doc): D on the west side, the three control
/// pins spread along the south edge (matches `Register.java`'s own port
/// layout in spirit, not pixel-for-pixel — nothing renders this yet).
/// Output Q on the east side.
fn register_geometry() -> Geometry {
    Geometry {
        inputs: vec![(0, 0), (1, 2), (2, 2), (3, 2)], // d, ck, clr, en
        outputs: vec![(4, 0)],
    }
}

/// `attrs["highDuration"]`/`attrs["lowDuration"]` for `core:Clock` —
/// defaulting to 1 each, same as `Clock.ATTR_HIGH`/`ATTR_LOW`'s own
/// defaults (`Integer.valueOf(1)` in the Java constructor). Must be >= 1
/// (`DurationAttribute`'s own range); no upper bound, matching Java's
/// `Integer.MAX_VALUE` ceiling closely enough that rejecting on overflow
/// into `i64` isn't worth the complexity here.
fn clock_duration_attr(circuit: &Circuit, comp: &ComponentInstance, key: &'static str) -> Result<u64, CompileError> {
    let raw = comp.attrs.get(key).and_then(|v| v.as_i64()).unwrap_or(1);
    if raw >= 1 {
        Ok(raw as u64)
    } else {
        Err(CompileError::InvalidClockDuration { circuit: circuit.name.clone(), id: comp.id.clone(), field: key, value: raw })
    }
}

/// `attrs["trigger"]` for `core:Register` — `StdAttr.TRIGGER`'s own option
/// strings, defaulting to `"rising"` (`StdAttr.TRIG_RISING`, `Register`'s
/// own default).
fn trigger_attr(circuit: &Circuit, comp: &ComponentInstance) -> Result<Trigger, CompileError> {
    let raw = comp.attrs.get("trigger").and_then(|v| v.as_str()).unwrap_or("rising");
    match raw {
        "rising" => Ok(Trigger::Rising),
        "falling" => Ok(Trigger::Falling),
        "high" => Ok(Trigger::High),
        "low" => Ok(Trigger::Low),
        other => Err(CompileError::InvalidTrigger { circuit: circuit.name.clone(), id: comp.id.clone(), value: other.to_string() }),
    }
}

pub(super) fn compile(type_: &str, circuit: &Circuit, comp: &ComponentInstance) -> Option<Result<(TemplateNode, Geometry), CompileError>> {
    let result = match type_ {
        "core:Clock" => (|| {
            let high = clock_duration_attr(circuit, comp, "highDuration")?;
            let low = clock_duration_attr(circuit, comp, "lowDuration")?;
            Ok((TemplateNode::Clock { high, low }, source_geometry()))
        })(),
        "core:Register" => (|| {
            let bits = width_attr(circuit, comp)?;
            let trigger = trigger_attr(circuit, comp)?;
            Ok((TemplateNode::Register { bits, trigger }, register_geometry()))
        })(),
        _ => return None,
    };
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::super::test_support::*;
    use super::*;
    use crate::compile::compile;
    use crate::file_format::{Circuit, ComponentInstance};
    use crate::netlist::flatten;
    use crate::sim::Simulation;
    use plugin_abi::Bit;
    use serde_json::json;
    use std::collections::BTreeMap;

    #[test]
    fn rejects_zero_duration_clock() {
        let mut attrs = BTreeMap::new();
        attrs.insert("highDuration".to_string(), json!(0));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("clk", "core:Clock", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidClockDuration {
                circuit: "main".to_string(),
                id: "clk".to_string(),
                field: "highDuration",
                value: 0,
            }
        );
    }

    #[test]
    fn rejects_unknown_trigger_option() {
        let mut attrs = BTreeMap::new();
        attrs.insert("trigger".to_string(), json!("on_edge"));
        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![ComponentInstance { attrs, ..comp("r", "core:Register", 0, 0) }],
            wires: vec![],
            annotations: vec![],
        });
        assert_eq!(
            compile(&project).unwrap_err(),
            CompileError::InvalidTrigger { circuit: "main".to_string(), id: "r".to_string(), value: "on_edge".to_string() }
        );
    }

    /// A `Clock` driving a `Register`'s CK end to end through compile ->
    /// flatten -> `Simulation::tick()` -> settle — the actual point of
    /// "one shared tick counter" (PLAN.md §14): `sim.tick()` is called on
    /// the `Simulation`, never on the clock gate directly, and the
    /// register still sees the right rising edges through it.
    ///
    /// Layout (see `register_geometry`/`source_geometry` offsets):
    /// register "r" at (10,0) -> D=(10,0), CK=(11,2), CLR=(12,2), EN=(13,2),
    /// Q=(14,0). `d` (Constant, coincides with D), `clk` (Clock, coincides
    /// with CK) placed to land exactly on those points; CLR/EN left
    /// floating on purpose (Unknown — exercises the same "undriven EN
    /// still enables" default as the unit test, now through the compiler).
    #[test]
    fn clock_drives_a_register_through_simulation_tick() {
        let mut const_attrs = BTreeMap::new();
        const_attrs.insert("width".to_string(), json!(3));
        const_attrs.insert("value".to_string(), json!(0b101));
        let mut reg_attrs = BTreeMap::new();
        reg_attrs.insert("width".to_string(), json!(3));

        let project = single_circuit_project(Circuit {
            name: "main".to_string(),
            components: vec![
                ComponentInstance { attrs: const_attrs, ..comp("d", "core:Constant", 10, 0) },
                ComponentInstance { attrs: reg_attrs, ..comp("r", "core:Register", 10, 0) },
                comp("clk", "core:Clock", 11, 2), // default high=low=1 -> period 2
                ComponentInstance {
                    attrs: { let mut a = BTreeMap::new(); a.insert("width".to_string(), json!(3)); a },
                    ..comp("out", "core:OutputPin", 14, 0)
                },
            ],
            wires: vec![],
            annotations: vec![],
        });

        let library = compile(&project).unwrap();
        let netlist = flatten(&project.main_circuit, &library).unwrap();
        let mut sim = Simulation::new(netlist);
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::Zero, Bit::Zero, Bit::Zero], "nothing latched before the first tick");

        sim.tick(); // global tick 1: 1%2=1, not < low(1) -> high phase -> clock rises 0->1
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::One, Bit::Zero, Bit::One], "rising edge latches D=0b101");

        sim.tick(); // global tick 2: 2%2=0 < 1 -> low phase -> clock falls 1->0
        sim.run_to_quiescence();
        assert_eq!(get_bits(&sim, 3), vec![Bit::One, Bit::Zero, Bit::One], "falling edge must not relatch");
    }
}
