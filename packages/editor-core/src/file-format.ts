/**
 * Native project file format (JSON) — mirrors crates/engine/src/file_format.rs
 * field-for-field. Rust is the source of truth (the engine loads/validates
 * these files); this file must be kept in sync by hand until the two sides
 * drift enough to justify codegen. See PLAN.md §9.
 *
 * Deliberately stops at the envelope: `type` and `attrs` are opaque here —
 * the standard component library is Phase 2, not part of this schema.
 */

export interface ProjectFile {
  schemaVersion: number;
  mainCircuit: string;
  circuits: Circuit[];
}

export interface Circuit {
  name: string;
  components: ComponentInstance[];
  wires: WireSegment[];
  annotations: Annotation[];
}

/** "core:AndGate" | "core:circuit/<name>" | "plugin:<plugin-id>/<component-id>" */
export type ComponentType = string;

export type Facing = "east" | "west" | "north" | "south";

export interface ComponentInstance {
  id: string;
  type: ComponentType;
  x: number;
  y: number;
  facing: Facing;
  attrs: Record<string, unknown>;
}

/** Two-point segment, mirroring `.circ`'s own wire model — junctions are
 * implicit at shared endpoints, not a separate entity. */
export interface WireSegment {
  id: string;
  from: [number, number];
  to: [number, number];
}

/** Non-simulated editor annotations — never have pins, never enter the netlist. */
export interface Annotation {
  id: string;
  kind: string;
  x: number;
  y: number;
  text?: string;
}
