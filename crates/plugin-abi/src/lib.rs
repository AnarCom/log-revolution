//! Draft component ABI (PLAN.md §5) — the contract shared by built-in and
//! WASM-plugin components alike. This crate defines the *shape* only: no
//! wasmtime/wasm-bindgen wiring yet (that's the host-side loader, added
//! when there's an actual engine to load components into — Phase 2).
//!
//! Two independent halves, on purpose (see PLAN.md §5 "Actions/readouts"):
//! - `eval`/`pin_count`/`serialize_state` — the simulation interface.
//! - `actions`/`readouts` — the interactive/test interface (`.ctest`
//!   scripts, PLAN.md §7). Not every component needs the second half, so
//!   it defaults to "none" rather than forcing every gate to implement it.

/// A single bit's value. Four states, not two — needed to reproduce
/// Logisim's tri-state/floating-input semantics (PLAN.md §9), not just
/// "0 or 1".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bit {
    Zero,
    One,
    /// Not driven by anything (high-impedance / disconnected).
    Floating,
    /// Driven by conflicting sources, or otherwise indeterminate.
    Unknown,
}

/// One pin's value — a bit vector, width is however many bits that pin is.
pub type Signal = Vec<Bit>;

/// Argument/return value for `invoke`/`read` — deliberately separate from
/// the file format's `AttrValue` (serde_json::Value): this is a simulated
/// interaction value, not instantiation config.
#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Bool(bool),
    Int(i64),
    Bits(Vec<Bit>),
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActionError {
    UnknownAction(String),
    InvalidArg { action: String, reason: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReadoutError {
    UnknownReadout(String),
}

pub trait Component {
    fn init(&mut self);

    fn pin_count(&self) -> usize;

    /// Pure function of current input pin values to output pin values —
    /// no I/O, no hidden state changes beyond what the component's own
    /// fields track (timing/scheduling is the engine's job, not the
    /// component's — see PLAN.md §3).
    fn eval(&mut self, inputs: &[Signal]) -> Vec<Signal>;

    fn serialize_state(&self) -> Vec<u8>;
    fn deserialize_state(&mut self, state: &[u8]);

    /// Named, side-effecting methods (`on`, `off`, `press`, `set(v)`, ...).
    /// Empty by default — most components (plain gates) have none.
    fn actions(&self) -> Vec<&'static str> {
        Vec::new()
    }

    fn invoke(&mut self, name: &str, _arg: Option<Value>) -> Result<(), ActionError> {
        Err(ActionError::UnknownAction(name.to_string()))
    }

    /// Named, side-effect-free reads (`get`, ...) — must never mutate
    /// component state; `.ctest`'s `assert` relies on that (PLAN.md §7).
    fn readouts(&self) -> Vec<&'static str> {
        Vec::new()
    }

    fn read(&self, name: &str) -> Result<Value, ReadoutError> {
        Err(ReadoutError::UnknownReadout(name.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Plain gate — no actions/readouts, relies entirely on the trait's
    /// defaults, to confirm the split is genuinely optional.
    struct AndGate;

    impl Component for AndGate {
        fn init(&mut self) {}
        fn pin_count(&self) -> usize {
            3
        }
        fn eval(&mut self, inputs: &[Signal]) -> Vec<Signal> {
            let bit = |s: &Signal| matches!(s.first(), Some(Bit::One));
            let out = if bit(&inputs[0]) && bit(&inputs[1]) {
                Bit::One
            } else {
                Bit::Zero
            };
            vec![vec![out]]
        }
        fn serialize_state(&self) -> Vec<u8> {
            Vec::new()
        }
        fn deserialize_state(&mut self, _state: &[u8]) {}
    }

    #[test]
    fn and_gate_evaluates_without_actions() {
        let mut gate = AndGate;
        gate.init();
        let out = gate.eval(&[vec![Bit::One], vec![Bit::One]]);
        assert_eq!(out, vec![vec![Bit::One]]);
        assert!(gate.actions().is_empty());
        assert!(gate.readouts().is_empty());
        assert_eq!(
            gate.invoke("on", None),
            Err(ActionError::UnknownAction("on".to_string()))
        );
    }

    /// Interactive component — no simulated inputs, state driven entirely
    /// through `actions`/`readouts` (the Poke-tool-as-ABI case from §5).
    struct Switch {
        on: bool,
    }

    impl Component for Switch {
        fn init(&mut self) {
            self.on = false;
        }
        fn pin_count(&self) -> usize {
            1
        }
        fn eval(&mut self, _inputs: &[Signal]) -> Vec<Signal> {
            vec![vec![if self.on { Bit::One } else { Bit::Zero }]]
        }
        fn serialize_state(&self) -> Vec<u8> {
            vec![self.on as u8]
        }
        fn deserialize_state(&mut self, state: &[u8]) {
            self.on = state.first().copied().unwrap_or(0) != 0;
        }

        fn actions(&self) -> Vec<&'static str> {
            vec!["on", "off", "toggle"]
        }
        fn invoke(&mut self, name: &str, _arg: Option<Value>) -> Result<(), ActionError> {
            match name {
                "on" => {
                    self.on = true;
                    Ok(())
                }
                "off" => {
                    self.on = false;
                    Ok(())
                }
                "toggle" => {
                    self.on = !self.on;
                    Ok(())
                }
                other => Err(ActionError::UnknownAction(other.to_string())),
            }
        }

        fn readouts(&self) -> Vec<&'static str> {
            vec!["get"]
        }
        fn read(&self, name: &str) -> Result<Value, ReadoutError> {
            match name {
                "get" => Ok(Value::Bool(self.on)),
                other => Err(ReadoutError::UnknownReadout(other.to_string())),
            }
        }
    }

    #[test]
    fn switch_toggles_via_actions_and_reads_via_readouts() {
        let mut sw = Switch { on: false };
        sw.init();
        assert_eq!(sw.read("get"), Ok(Value::Bool(false)));

        sw.invoke("on", None).unwrap();
        assert_eq!(sw.read("get"), Ok(Value::Bool(true)));
        assert_eq!(sw.eval(&[]), vec![vec![Bit::One]]);

        sw.invoke("toggle", None).unwrap();
        assert_eq!(sw.read("get"), Ok(Value::Bool(false)));

        assert_eq!(
            sw.read("nonexistent"),
            Err(ReadoutError::UnknownReadout("nonexistent".to_string()))
        );
    }

    #[test]
    fn state_round_trips() {
        let sw = Switch { on: true };
        let state = sw.serialize_state();
        let mut restored = Switch { on: false };
        restored.deserialize_state(&state);
        assert_eq!(restored.read("get"), Ok(Value::Bool(true)));
    }
}
