//! Union-find over pin/wire-endpoint bit lanes — the core mechanism behind
//! `compile_circuit`'s net resolution (see `compile/mod.rs`'s doc comment
//! for the *why*; this file is purely the *how*, generic over any `Lane`).

use super::geometry::Point;
use std::collections::HashMap;

/// `Value.MAX_WIDTH` — the upper bound for any pin's width, so also the
/// number of lanes worth unioning at any coincident pair of points. Wires
/// (and `Splitter` ends) don't know in advance how wide the pins touching
/// them are, so every wire unconditionally unions all 32; lanes no real pin
/// ever registers just sit unqueried, harmless.
pub(crate) const MAX_WIDTH: u8 = 32;

/// A single electrical *bit lane*: bit `1` of a point coinciding with a
/// 4-bit pin is a different lane than bit `0` there, and a `Splitter` can
/// fuse it to an entirely different point's bit than an ordinary wire
/// would — see `compile/mod.rs`'s doc comment.
pub(crate) type Lane = (Point, u8);

/// Union-find over pin/wire-endpoint bit lanes.
pub(crate) struct Dsu {
    parent: HashMap<Lane, Lane>,
}

impl Dsu {
    pub(crate) fn new() -> Self {
        Dsu { parent: HashMap::new() }
    }

    pub(crate) fn find(&mut self, p: Lane) -> Lane {
        let parent = *self.parent.entry(p).or_insert(p);
        if parent == p {
            p
        } else {
            let root = self.find(parent);
            self.parent.insert(p, root);
            root
        }
    }

    pub(crate) fn union(&mut self, a: Lane, b: Lane) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            self.parent.insert(ra, rb);
        }
    }
}
