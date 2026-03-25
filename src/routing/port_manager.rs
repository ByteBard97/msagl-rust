use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use super::compass_direction::CompassDirection;
use super::transient_graph_utility::TransientGraphUtility;

/// Result of splicing a port into the visibility graph.
///
/// Holds the port vertex ID and the transient-graph utility that tracks
/// all additions so they can be cleanly removed by `PortManager::unsplice`.
pub struct PortSpliceResult {
    pub port_vertex: VertexId,
    pub tgu: TransientGraphUtility,
}

/// Manages splicing port locations into and out of a visibility graph.
///
/// Port splicing temporarily connects a point (port location) to the
/// visibility graph by finding axis-aligned neighbours and adding
/// bracket-aware edges via `TransientGraphUtility`.
pub struct PortManager;

impl PortManager {
    /// Splice a port location into the visibility graph using
    /// `TransientGraphUtility` for proper edge management.
    ///
    /// Finds the nearest axis-aligned vertex in each compass direction and
    /// connects with bracket-aware edge management. Returns a `PortSpliceResult`
    /// that must be passed to `unsplice` to restore the graph.
    pub fn splice_port(graph: &mut VisibilityGraph, location: Point) -> PortSpliceResult {
        let mut tgu = TransientGraphUtility::new();
        let port_vertex = tgu.find_or_add_vertex(graph, location);

        for &direction in &CompassDirection::all() {
            if let Some((neighbor, dist)) =
                Self::find_nearest_aligned(graph, port_vertex, location, direction)
            {
                tgu.find_or_add_edge(graph, port_vertex, neighbor, dist);
            }
        }

        PortSpliceResult { port_vertex, tgu }
    }

    /// Remove all transient port-splice modifications, restoring the graph.
    pub fn unsplice(graph: &mut VisibilityGraph, result: &mut PortSpliceResult) {
        result.tgu.remove_from_graph(graph);
    }

    /// Find the nearest vertex axis-aligned with `location` in `direction`.
    ///
    /// Scans all vertices, keeping only those that lie on the same axis
    /// (same X for North/South, same Y for East/West) in the requested
    /// direction. Returns the closest one together with its distance.
    fn find_nearest_aligned(
        graph: &VisibilityGraph,
        port_vertex: VertexId,
        location: Point,
        direction: CompassDirection,
    ) -> Option<(VertexId, f64)> {
        let mut best: Option<(VertexId, f64)> = None;
        let eps = GeomConstants::DISTANCE_EPSILON;

        for i in 0..graph.vertex_count() {
            let vid = VertexId(i);
            if vid == port_vertex {
                continue;
            }
            let pt = graph.point(vid);

            let is_aligned = match direction {
                CompassDirection::East => {
                    (pt.y() - location.y()).abs() < eps && pt.x() > location.x() + eps
                }
                CompassDirection::West => {
                    (pt.y() - location.y()).abs() < eps && pt.x() < location.x() - eps
                }
                CompassDirection::North => {
                    (pt.x() - location.x()).abs() < eps && pt.y() > location.y() + eps
                }
                CompassDirection::South => {
                    (pt.x() - location.x()).abs() < eps && pt.y() < location.y() - eps
                }
            };

            if is_aligned {
                let dist = ((pt.x() - location.x()).powi(2)
                    + (pt.y() - location.y()).powi(2))
                .sqrt();
                if best.is_none_or(|(_, d)| dist < d) {
                    best = Some((vid, dist));
                }
            }
        }

        best
    }
}
