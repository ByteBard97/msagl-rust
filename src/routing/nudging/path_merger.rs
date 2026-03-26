//! PathMerger: detects and removes self-loops from paths.
//!
//! Ported from PathMerger.ts. Scans each path's vertices for self-cycles
//! (the same point visited twice) and removes the loop portion.

use std::collections::HashMap;

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;

/// Removes self-loops and multi-crossing patterns from paths.
/// Ported from PathMerger.ts.
pub struct PathMerger;

/// Hashable point key for HashMap lookup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
struct PointKey(OrderedFloat<f64>, OrderedFloat<f64>);

impl From<Point> for PointKey {
    fn from(p: Point) -> Self {
        PointKey(OrderedFloat(p.x()), OrderedFloat(p.y()))
    }
}

impl PathMerger {
    /// Remove self-cycles from all paths.
    ///
    /// A self-cycle occurs when a path visits the same point twice.
    /// For each such occurrence at indices `i` and `j` (`i < j`),
    /// the points `(i+1)..=j` are removed.
    pub fn merge_paths(paths: &mut [Vec<Point>]) {
        for path in paths.iter_mut() {
            Self::remove_self_cycles(path);
        }
    }

    /// Remove self-cycles from a single path.
    ///
    /// If point P appears at indices `i` and `j` (`i < j`),
    /// removes points `[i+1..=j]` and restarts the scan from the
    /// beginning (since earlier points may now form new cycles).
    fn remove_self_cycles(path: &mut Vec<Point>) {
        let mut seen: HashMap<PointKey, usize> = HashMap::new();
        let mut i = 0;
        while i < path.len() {
            let key = PointKey::from(path[i]);
            if let Some(&prev_idx) = seen.get(&key) {
                // Self-cycle detected: remove points (prev_idx+1)..=i
                path.drain((prev_idx + 1)..=i);
                // Reset — recheck from beginning since path changed
                seen.clear();
                i = 0;
                continue;
            }
            seen.insert(key, i);
            i += 1;
        }
    }
}
