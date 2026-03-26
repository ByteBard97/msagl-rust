use super::compass_direction::CompassDirection;
use super::transient_graph_utility::TransientGraphUtility;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};

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
///
/// When a port is inside an obstacle (the common case for center ports),
/// the standard aligned-vertex search will find nothing because VG segments
/// have gaps where obstacles are. In that case, the port manager fires
/// axis-aligned rays outward and finds the nearest VG *edge* that the ray
/// crosses, creates a vertex at the intersection, and connects the port.
pub struct PortManager;

impl PortManager {
    /// Splice a port location into the visibility graph using
    /// `TransientGraphUtility` for proper edge management.
    ///
    /// First tries to find existing axis-aligned VG vertices. If none are
    /// found (port is inside an obstacle gap), falls back to ray-casting
    /// against VG edges to create intersection vertices.
    pub fn splice_port(graph: &mut VisibilityGraph, location: Point) -> PortSpliceResult {
        let mut tgu = TransientGraphUtility::new();
        let port_vertex = tgu.find_or_add_vertex(graph, location);

        // For each direction, try ray-casting first (finds the nearest VG edge
        // the axis-ray crosses). This correctly handles ports inside obstacles by
        // connecting only to the nearest boundary, not to distant aligned vertices
        // that may be on the other side of an obstacle.
        //
        // If ray-casting finds nothing (port is already on a VG edge), fall back
        // to aligned-vertex search.
        for &direction in &CompassDirection::all() {
            if let Some(intersection) = Self::find_nearest_crossing_edge(graph, location, direction)
            {
                let (edge_source, edge_target, intersect_point) = intersection;
                let split_vertex = tgu.add_edge_to_target_edge(
                    graph,
                    port_vertex,
                    edge_source,
                    edge_target,
                    intersect_point,
                );
                let dist = ((intersect_point.x() - location.x()).powi(2)
                    + (intersect_point.y() - location.y()).powi(2))
                .sqrt();
                if dist > GeomConstants::DISTANCE_EPSILON {
                    tgu.find_or_add_edge(graph, port_vertex, split_vertex, dist);
                }
            } else if let Some((neighbor, dist)) =
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

    /// Find the nearest VG edge that an axis-aligned ray from `location`
    /// in `direction` crosses.
    ///
    /// For East/West: looks for vertical VG edges on the same row (perpendicular
    /// edges the horizontal ray would cross). Actually, more precisely, we look
    /// for any edge whose perpendicular extent brackets the port's coordinate.
    ///
    /// Returns (edge_source, edge_target, intersection_point).
    fn find_nearest_crossing_edge(
        graph: &VisibilityGraph,
        location: Point,
        direction: CompassDirection,
    ) -> Option<(VertexId, VertexId, Point)> {
        let eps = GeomConstants::DISTANCE_EPSILON;
        let mut best: Option<(VertexId, VertexId, Point, f64)> = None;

        for i in 0..graph.vertex_count() {
            let vid = VertexId(i);
            let v_point = graph.point(vid);

            // Look at all edges from this vertex
            for edge in graph.out_edges(vid) {
                let neighbor_id = edge.target;
                let n_point = graph.point(neighbor_id);

                // Only consider edges perpendicular to the ray direction.
                let crossing = match direction {
                    CompassDirection::East | CompassDirection::West => {
                        // Ray is horizontal. Look for vertical edges.
                        if (v_point.x() - n_point.x()).abs() > eps {
                            continue; // not vertical
                        }
                        let edge_x = v_point.x();
                        let (min_y, max_y) = ordered_pair(v_point.y(), n_point.y());
                        // Does the edge span the port's y?
                        if location.y() < min_y - eps || location.y() > max_y + eps {
                            continue;
                        }
                        // Is the edge in the right direction?
                        let in_direction = match direction {
                            CompassDirection::East => edge_x > location.x() + eps,
                            CompassDirection::West => edge_x < location.x() - eps,
                            _ => false,
                        };
                        if !in_direction {
                            continue;
                        }
                        let intersect = Point::new(edge_x, location.y());
                        let dist = (edge_x - location.x()).abs();
                        Some((intersect, dist))
                    }
                    CompassDirection::North | CompassDirection::South => {
                        // Ray is vertical. Look for horizontal edges.
                        if (v_point.y() - n_point.y()).abs() > eps {
                            continue; // not horizontal
                        }
                        let edge_y = v_point.y();
                        let (min_x, max_x) = ordered_pair(v_point.x(), n_point.x());
                        // Does the edge span the port's x?
                        if location.x() < min_x - eps || location.x() > max_x + eps {
                            continue;
                        }
                        // Is the edge in the right direction?
                        let in_direction = match direction {
                            CompassDirection::North => edge_y > location.y() + eps,
                            CompassDirection::South => edge_y < location.y() - eps,
                            _ => false,
                        };
                        if !in_direction {
                            continue;
                        }
                        let intersect = Point::new(location.x(), edge_y);
                        let dist = (edge_y - location.y()).abs();
                        Some((intersect, dist))
                    }
                };

                if let Some((intersect, dist)) = crossing {
                    if best.is_none_or(|(_, _, _, d)| dist < d) {
                        best = Some((vid, neighbor_id, intersect, dist));
                    }
                }
            }
        }

        if best.is_none() && (location.x() - 70.0).abs() < 0.1 && (location.y() - 70.0).abs() < 0.1
        {
            eprintln!(
                "  find_crossing({:?}) from ({},{}) found NOTHING in {} verts",
                direction,
                location.x(),
                location.y(),
                graph.vertex_count()
            );
        }
        best.map(|(s, t, p, _)| (s, t, p))
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
                let dist =
                    ((pt.x() - location.x()).powi(2) + (pt.y() - location.y()).powi(2)).sqrt();
                if best.is_none_or(|(_, d)| dist < d) {
                    best = Some((vid, dist));
                }
            }
        }

        best
    }
}

/// Return (min, max) of two f64 values.
fn ordered_pair(a: f64, b: f64) -> (f64, f64) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}
