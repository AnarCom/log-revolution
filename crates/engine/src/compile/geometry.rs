//! Component pin geometry — relative pin positions, used only to make
//! points "distinct enough to not coincide by accident" when two unrelated
//! components sit at different `(x, y)`s. Deliberately small/arbitrary:
//! this is our own schema, not `.circ`'s exact pixel layout (matching that
//! is the importer's job, PLAN.md §9), and nothing renders any of it yet —
//! only whether two pins' *absolute* points coincide matters to the
//! compiler, never where they'd actually be drawn.

use crate::file_format::Facing;

pub(crate) type Point = (i32, i32);

/// A component's pins, relative to its own origin, at `Facing::East`
/// (rotated per-instance for other facings by `rotate`).
pub(crate) struct Geometry {
    pub(crate) inputs: Vec<Point>,
    pub(crate) outputs: Vec<Point>,
}

/// Single-input, single-output leaf shape (Not/Buffer): one input dead
/// center, one output two units east.
pub(crate) fn unary_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0)], outputs: vec![(2, 0)] }
}

/// Source/sink leaf shape with no input side (InputPin/Constant/
/// PullResistor/Clock — spans both `wiring` and `memory`, hence living
/// here rather than in either): a single output pin at the origin.
pub(crate) fn source_geometry() -> Geometry {
    Geometry { inputs: vec![], outputs: vec![(0, 0)] }
}

/// `OutputPin`: the mirror of `source_geometry` — one input, no outputs.
pub(crate) fn sink_geometry() -> Geometry {
    Geometry { inputs: vec![(0, 0)], outputs: vec![] }
}

/// A subcircuit instance's port count varies per reference, so its
/// footprint is computed rather than looked up: input ports stacked down
/// the left edge, output ports down the right, on a fixed-width box.
/// Arbitrary — nothing renders this yet, only connectivity matters.
pub(crate) const SUBCIRCUIT_WIDTH: i32 = 6;

pub(crate) fn subcircuit_geometry(input_ports: usize, output_ports: usize) -> Geometry {
    Geometry {
        inputs: (0..input_ports).map(|i| (0, 2 * i as i32)).collect(),
        outputs: (0..output_ports).map(|i| (SUBCIRCUIT_WIDTH, 2 * i as i32)).collect(),
    }
}

/// Rotates a `Facing::East`-relative offset for the other three facings.
/// East is the identity; the rest are 90°-step rotations, applied
/// consistently (not matched to any particular on-screen convention, since
/// nothing renders this yet).
pub(crate) fn rotate(offset: Point, facing: Facing) -> Point {
    let (dx, dy) = offset;
    match facing {
        Facing::East => (dx, dy),
        Facing::South => (-dy, dx),
        Facing::West => (-dx, -dy),
        Facing::North => (dy, -dx),
    }
}
