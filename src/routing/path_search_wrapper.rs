//! Backward-compatible wrapper and utility functions for path search.
//!
//! Re-exports: `PathSearch`, `SearchArena`, `manhattan_distance`,
//! `estimated_bends_to_target`.

use super::compass_direction::CompassDirection;
use super::path_search::{
    SearchEntry, SsstRectilinearPath, DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE,
};
use crate::geometry::point::Point;
use crate::routing::vertex_entry::VertexEntryIndex;
use crate::visibility::graph::VisibilityGraph;

/// Compute Manhattan distance between two points.
pub fn manhattan_distance(a: Point, b: Point) -> f64 {
    (b.x() - a.x()).abs() + (b.y() - a.y()).abs()
}

/// Estimate bends needed to reach target from point traveling in direction.
pub fn estimated_bends_to_target(direction: CompassDirection, point: Point, target: Point) -> u32 {
    let dx = target.x() - point.x();
    let dy = target.y() - point.y();

    if dx.abs() < 1e-9 && dy.abs() < 1e-9 {
        return 0;
    }

    let going_toward = match direction {
        CompassDirection::East => dx > 1e-9,
        CompassDirection::West => dx < -1e-9,
        CompassDirection::North => dy > 1e-9,
        CompassDirection::South => dy < -1e-9,
    };

    let need_horizontal = dx.abs() > 1e-9;
    let need_vertical = dy.abs() > 1e-9;

    if !need_horizontal && !need_vertical {
        0
    } else if need_horizontal && need_vertical {
        if going_toward {
            1
        } else {
            2
        }
    } else if going_toward {
        0
    } else {
        2
    }
}

/// Zero-cost wrapper over `Vec<SearchEntry>` for external arena access.
#[repr(transparent)]
pub struct SearchArena {
    inner: [SearchEntry],
}

impl SearchArena {
    pub(crate) fn from_vec(v: &Vec<SearchEntry>) -> &SearchArena {
        unsafe { &*(v.as_slice() as *const [SearchEntry] as *const SearchArena) }
    }

    pub fn get(&self, idx: VertexEntryIndex) -> &SearchEntry {
        &self.inner[idx.0]
    }

    pub fn entries(&self) -> &[SearchEntry] {
        &self.inner
    }
}

/// Direction-aware path search on a rectilinear visibility graph.
/// Backward-compatible wrapper around `SsstRectilinearPath`.
pub struct PathSearch {
    pub bend_penalty_as_percentage: f64,
}

impl PathSearch {
    pub fn new(bend_penalty_as_percentage: f64) -> Self {
        Self {
            bend_penalty_as_percentage,
        }
    }

    pub fn default_penalty() -> Self {
        Self::new(DEFAULT_BEND_PENALTY_AS_PERCENTAGE_OF_DISTANCE)
    }

    /// Compute cost for backward compatibility with existing tests.
    ///
    /// Uses the old formula: `length + (source_target_distance * percentage/100) * bends`.
    pub fn compute_cost(&self, length: f64, bends: u32, source_target_distance: f64) -> f64 {
        let bend_cost = source_target_distance * self.bend_penalty_as_percentage / 100.0;
        length + bend_cost * bends as f64
    }

    /// Find shortest path between two points.
    pub fn find_path(
        &self,
        graph: &VisibilityGraph,
        source: Point,
        target: Point,
    ) -> Option<Vec<Point>> {
        let source_id = graph.find_vertex(source)?;
        let target_id = graph.find_vertex(target)?;

        if source_id == target_id {
            return Some(vec![source, target]);
        }

        let dist = manhattan_distance(source, target);
        let mut ssst = SsstRectilinearPath::new();
        ssst.length_importance = 1.0;
        ssst.bends_importance = f64::max(0.001, dist * self.bend_penalty_as_percentage * 0.01);

        let result = ssst.get_path_with_cost(
            None,
            source_id,
            0.0,
            None,
            target_id,
            0.0,
            f64::INFINITY,
            graph,
        );

        match result {
            Some(entry_idx) => {
                let path =
                    SsstRectilinearPath::restore_path_v(ssst.arena(), Some(entry_idx), graph);
                if path.is_empty() {
                    None
                } else {
                    Some(path)
                }
            }
            None => None,
        }
    }
}
