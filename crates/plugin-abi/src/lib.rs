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

/// A single bit's value. Four states, not two — mirrors Logisim's own
/// `Value` class exactly (`logisim-port/.../data/Value.java`: FALSE, TRUE,
/// UNKNOWN, ERROR), colors included (blue = `Unknown`/nothing driving,
/// red = `Error`/conflicting drivers — a short circuit). Needed to
/// reproduce `.circ` import semantics faithfully (PLAN.md §9), not just as
/// a nicety — Logisim circuits routinely rely on this (e.g. an
/// intentionally floating input, or detecting two outputs tied together).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bit {
    Zero,
    One,
    /// Nothing is actively driving this point (floating/high-impedance).
    /// The identity element of `combine` — matches `Value.UNKNOWN`.
    Unknown,
    /// Two or more drivers disagree on this point — a short circuit.
    /// Matches `Value.ERROR`.
    Error,
}

impl Bit {
    /// What a point's value is when multiple drivers feed it — used to
    /// resolve nets with more than one source (PLAN.md §9 tri-state
    /// behavior). Mirrors `Value.combine` for the single-bit case
    /// byte-for-byte: `Unknown` is the identity, agreement passes through,
    /// disagreement is `Error`. Fold over drivers starting from `Unknown`
    /// (an undriven point's value, same as `Value.createUnknown`).
    pub fn combine(self, other: Bit) -> Bit {
        if self == Bit::Unknown {
            other
        } else if other == Bit::Unknown {
            self
        } else if self == other {
            self
        } else {
            Bit::Error
        }
    }

    /// Mirrors `Value.and`: `Zero` is dominant (absorbing) regardless of
    /// the other input's uncertainty — matches real gate electronics, not
    /// naive "error propagates through everything".
    pub fn and(self, other: Bit) -> Bit {
        if self == Bit::Zero || other == Bit::Zero {
            Bit::Zero
        } else if self == Bit::One && other == Bit::One {
            Bit::One
        } else {
            Bit::Error
        }
    }

    /// Mirrors `Value.or`: `One` is dominant, symmetric to `and`.
    pub fn or(self, other: Bit) -> Bit {
        if self == Bit::One || other == Bit::One {
            Bit::One
        } else if self == Bit::Zero && other == Bit::Zero {
            Bit::Zero
        } else {
            Bit::Error
        }
    }

    /// Mirrors `Value.not`: undefined input stays undefined-as-error, not
    /// `Unknown` — matches the reference implementation exactly rather
    /// than guessing at "nicer" behavior.
    pub fn not(self) -> Bit {
        match self {
            Bit::One => Bit::Zero,
            Bit::Zero => Bit::One,
            Bit::Unknown | Bit::Error => Bit::Error,
        }
    }
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

    fn input_count(&self) -> usize;
    fn output_count(&self) -> usize;

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
        fn input_count(&self) -> usize {
            2
        }
        fn output_count(&self) -> usize {
            1
        }
        fn eval(&mut self, inputs: &[Signal]) -> Vec<Signal> {
            let bit = |s: &Signal| s.first().copied().unwrap_or(Bit::Unknown);
            vec![vec![bit(&inputs[0]).and(bit(&inputs[1]))]]
        }
        fn serialize_state(&self) -> Vec<u8> {
            Vec::new()
        }
        fn deserialize_state(&mut self, _state: &[u8]) {}
    }

    /// Truth tables lifted directly from `logisim-port`'s `Value.java`
    /// (`and`/`or`/`not`/`combine`) — not reinvented, checked against the
    /// reference oracle (PLAN.md §9).
    #[test]
    fn bit_matches_logisim_value_semantics() {
        use Bit::*;

        // `and`: Zero is absorbing regardless of the other side.
        assert_eq!(Zero.and(Unknown), Zero);
        assert_eq!(Zero.and(Error), Zero);
        assert_eq!(One.and(One), One);
        assert_eq!(One.and(Unknown), Error);
        assert_eq!(Unknown.and(Unknown), Error);

        // `or`: One is absorbing, symmetric to `and`.
        assert_eq!(One.or(Unknown), One);
        assert_eq!(One.or(Error), One);
        assert_eq!(Zero.or(Zero), Zero);
        assert_eq!(Zero.or(Unknown), Error);

        // `not`: only defined values invert cleanly.
        assert_eq!(One.not(), Zero);
        assert_eq!(Zero.not(), One);
        assert_eq!(Unknown.not(), Error);
        assert_eq!(Error.not(), Error);

        // `combine`: Unknown is the identity (undriven point); agreement
        // passes through; disagreement is a short circuit.
        assert_eq!(Unknown.combine(One), One);
        assert_eq!(One.combine(Unknown), One);
        assert_eq!(One.combine(One), One);
        assert_eq!(One.combine(Zero), Error);
        assert_eq!(Unknown.combine(Unknown), Unknown);
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
        fn input_count(&self) -> usize {
            0
        }
        fn output_count(&self) -> usize {
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
