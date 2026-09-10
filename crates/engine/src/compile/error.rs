//! Every way `compile()` can fail — split out from `compile/mod.rs` purely
//! for size (this enum's doc comments account for a third of what used to
//! be one file), not because it depends on anything special: every variant
//! is just `{circuit, id, ..offending value}`, so callers can point at
//! exactly which component in which circuit is wrong.

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CompileError {
    UnknownComponentType { circuit: String, id: String, type_: String },
    UnknownSubcircuit { circuit: String, id: String, referenced: String },
    InvalidPullTarget { circuit: String, id: String, value: String },
    DuplicateComponentId { circuit: String, id: String },
    /// `attrs["width"]` outside 1..=32 — `logisim-port`'s own ceiling
    /// (`Value.MAX_WIDTH`; see `BitWidth.create`, which rejects <= 0, and
    /// `Value.create`, which rejects > `MAX_WIDTH`). Rejected outright
    /// rather than silently clamped, so a schema/import bug shows up at
    /// compile time, not as a quietly-truncated bus.
    InvalidWidth { circuit: String, id: String, value: i64 },
    /// `attrs["inputs"]` outside 2..=32 — `GateAttributes.MAX_INPUTS`/
    /// `ATTR_INPUTS`'s own range (a gate needs at least 2 inputs to mean
    /// anything, and Logisim caps it at 32 same as bit width).
    InvalidInputCount { circuit: String, id: String, value: i64 },
    /// `attrs["highDuration"]`/`attrs["lowDuration"]` below 1 —
    /// `Clock.ATTR_HIGH`/`ATTR_LOW`'s own `DurationAttribute` range (a
    /// clock phase lasting zero ticks is meaningless).
    InvalidClockDuration { circuit: String, id: String, field: &'static str, value: i64 },
    /// `attrs["trigger"]` isn't one of `StdAttr.TRIGGER`'s own four option
    /// strings (`"rising"`/`"falling"`/`"high"`/`"low"`).
    InvalidTrigger { circuit: String, id: String, value: String },
    /// `attrs["select"]` outside 1..=5 — `Plexers.ATTR_SELECT`'s own range
    /// (`Attributes.forBitWidth("select", .., 1, 5)`).
    InvalidSelectWidth { circuit: String, id: String, value: i64 },
    /// `attrs["disabled"]` isn't one of `Plexers.ATTR_DISABLED`'s two
    /// option strings (`"Z"`/`"0"`).
    InvalidDisabledOption { circuit: String, id: String, value: String },
    /// `attrs["mode"]` for `core:Comparator` isn't one of `Comparator.java`'s
    /// own two option strings (`"twosComplement"`/`"unsigned"`).
    InvalidComparatorMode { circuit: String, id: String, value: String },
    /// `attrs["fanout"]` outside 1..=32 — `SplitterAttributes.ATTR_FANOUT`'s
    /// own range (`Attributes.forIntegerRange("fanout", .., 1, 32)`).
    InvalidFanout { circuit: String, id: String, value: i64 },
    /// `attrs["bits"]` (this schema's own encoding of `bit_end`, see
    /// `compile/splitter.rs`) has the wrong length or an out-of-range
    /// entry.
    InvalidSplitterBits { circuit: String, id: String, reason: String },
    /// `attrs["type"]` for `core:BitExtender` isn't one of `BitExtender.
    /// java`'s own four option strings (`"zero"`/`"one"`/`"sign"`/`"input"`).
    InvalidExtendType { circuit: String, id: String, value: String },
    /// `attrs["bus"]` for `core:Ram` isn't one of `Ram.java`'s own three
    /// option strings (`"combined"`/`"asynch"`/`"separate"`).
    InvalidRamBus { circuit: String, id: String, value: String },
}
