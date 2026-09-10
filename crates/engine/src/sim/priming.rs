//! `Simulation::new`'s initial priming schedule.

use super::Time;
use crate::netlist::Netlist;

/// Each gate's distance (in hops) from the nearest gate with no real
/// drivers on any input pin — i.e. a source, or something floating enough
/// to be treated as one. Used only to stagger `Simulation::new`'s initial
/// priming events (see its doc comment for why staggering matters, not
/// just simultaneous `t=0` for everyone).
///
/// A fixed-point relaxation, not a single topological pass, because
/// Logisim circuits can genuinely contain feedback loops (cross-coupled
/// gates, oscillators) where no acyclic topological order exists at all —
/// gates that never resolve through the relaxation (because they're part
/// of, or depend only on, such a cycle) fall back to depth `0`, the same
/// treatment as an actual source. That's a reasonable fallback, not a
/// hack: a cycle has no well-defined "settled initial value" to begin
/// with, so there's nothing a smarter depth assignment could preserve for
/// it anyway — same as today, it settles (or is detected oscillating)
/// through ordinary re-triggering once the simulation actually starts
/// stepping.
pub(super) fn priming_depths(netlist: &Netlist) -> Vec<Time> {
    let n = netlist.gates.len();
    let direct_sources: Vec<Vec<usize>> =
        netlist.input_sources.iter().map(|pins| pins.iter().flatten().flatten().map(|&(g, _, _)| g).collect()).collect();

    let mut depth: Vec<Option<Time>> = vec![None; n];
    loop {
        let mut progressed = false;
        for i in 0..n {
            if depth[i].is_some() {
                continue;
            }
            if direct_sources[i].is_empty() {
                depth[i] = Some(0);
                progressed = true;
                continue;
            }
            let mut max_dep = 0;
            let mut all_resolved = true;
            for &s in &direct_sources[i] {
                match depth[s] {
                    Some(d) => max_dep = max_dep.max(d),
                    None => {
                        all_resolved = false;
                        break;
                    }
                }
            }
            if all_resolved {
                depth[i] = Some(max_dep + 1);
                progressed = true;
            }
        }
        if !progressed {
            break;
        }
    }
    depth.into_iter().map(|d| d.unwrap_or(0)).collect()
}
