use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};

/// Manages splicing port locations into and out of a visibility graph.
///
/// Port splicing connects a point (port location) to the visibility graph
/// by finding axis-aligned neighbors and adding edges.
pub struct PortManager;

impl PortManager {
    /// Splice a port location into the visibility graph.
    ///
    /// Finds the nearest axis-aligned vertex in each compass direction and
    /// adds bidirectional edges. Returns the vertex ID of the port and
    /// tracking information for later cleanup.
    pub fn splice_port(
        graph: &mut VisibilityGraph,
        location: Point,
    ) -> (VertexId, Vec<VertexId>, Vec<(VertexId, VertexId)>) {
        let port_id = graph.find_or_add_vertex(location);
        let added_vertices = Vec::new();
        let mut added_edges = Vec::new();

        // Find nearest aligned vertex in each of the 4 directions
        let mut best_east: Option<(VertexId, f64)> = None;
        let mut best_west: Option<(VertexId, f64)> = None;
        let mut best_north: Option<(VertexId, f64)> = None;
        let mut best_south: Option<(VertexId, f64)> = None;

        for i in 0..graph.vertex_count() {
            let vid = VertexId(i);
            if vid == port_id {
                continue;
            }
            let pt = graph.point(vid);

            // Check horizontal alignment (same Y)
            if GeomConstants::close(pt.y(), location.y()) {
                let dx = pt.x() - location.x();
                let dist = dx.abs();
                if dx > GeomConstants::DISTANCE_EPSILON {
                    // East
                    if best_east.is_none_or(|(_, d)| dist < d) {
                        best_east = Some((vid, dist));
                    }
                } else if dx < -GeomConstants::DISTANCE_EPSILON {
                    // West
                    if best_west.is_none_or(|(_, d)| dist < d) {
                        best_west = Some((vid, dist));
                    }
                }
            }

            // Check vertical alignment (same X)
            if GeomConstants::close(pt.x(), location.x()) {
                let dy = pt.y() - location.y();
                let dist = dy.abs();
                if dy > GeomConstants::DISTANCE_EPSILON {
                    // North
                    if best_north.is_none_or(|(_, d)| dist < d) {
                        best_north = Some((vid, dist));
                    }
                } else if dy < -GeomConstants::DISTANCE_EPSILON {
                    // South
                    if best_south.is_none_or(|(_, d)| dist < d) {
                        best_south = Some((vid, dist));
                    }
                }
            }
        }

        // Add bidirectional edges to nearest aligned vertices
        for (vid, dist) in [best_east, best_west, best_north, best_south].into_iter().flatten() {
            graph.add_edge(port_id, vid, dist);
            graph.add_edge(vid, port_id, dist);
            added_edges.push((port_id, vid));
            added_edges.push((vid, port_id));
        }

        (port_id, added_vertices, added_edges)
    }

    /// Remove transient edges added by splice_port.
    ///
    /// Note: vertices are not removed since the graph uses Vec storage
    /// and removing would invalidate indices. Orphaned vertices are harmless.
    pub fn unsplice(
        graph: &mut VisibilityGraph,
        _added_vertices: &[VertexId],
        added_edges: &[(VertexId, VertexId)],
    ) {
        for &(s, t) in added_edges {
            graph.remove_edge(s, t);
        }
    }
}
