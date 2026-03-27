use super::compass_direction::CompassDirection;
use super::static_graph_utility::StaticGraphUtility;
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
    /// For each compass direction, finds the nearest VG structure (either
    /// a perpendicular crossing edge or an aligned vertex) and connects the
    /// port to it.
    ///
    /// Uses O(log V) coordinate-index lookups + O(chain) VG adjacency walks
    /// (ported from C# `FindNearestPerpendicularOrContainingEdge`) instead of
    /// O(V) brute-force vertex scanning.
    pub fn splice_port(graph: &mut VisibilityGraph, location: Point) -> PortSpliceResult {
        let mut tgu = TransientGraphUtility::new();
        let port_vertex = tgu.find_or_add_vertex(graph, location);

        for &direction in &CompassDirection::all() {
            // Strategy 1: Find the nearest perpendicular VG edge crossing
            // using coordinate-index lookup + VG adjacency chain walk.
            if let Some(intersection) =
                Self::find_nearest_crossing_edge(graph, location, direction)
            {
                let (edge_source, edge_target, intersect_point) = intersection;
                // Detect zombie vertices: vertices left behind by a previous
                // splice that was unspliced, leaving them completely disconnected.
                let pre_existing = graph.find_vertex(intersect_point);
                let was_orphan = pre_existing
                    .is_some_and(|v| graph.out_degree(v) == 0 && graph.in_degree(v) == 0);

                let split_vertex = tgu.add_edge_to_target_edge(
                    graph,
                    port_vertex,
                    edge_source,
                    edge_target,
                    intersect_point,
                );

                // If the vertex was orphaned and the original VG edge still
                // exists unsplit, split it to reconnect the vertex to the VG.
                if was_orphan && graph.find_edge(edge_source, edge_target).is_some() {
                    tgu.split_edge(graph, edge_source, edge_target, split_vertex);
                }

                let dist = ((intersect_point.x() - location.x()).powi(2)
                    + (intersect_point.y() - location.y()).powi(2))
                .sqrt();
                if dist > GeomConstants::DISTANCE_EPSILON {
                    tgu.find_or_add_edge(graph, port_vertex, split_vertex, dist);
                }
            } else if let Some((neighbor, dist)) =
                Self::find_nearest_aligned(graph, port_vertex, location, direction)
            {
                // Strategy 2: Direct aligned-vertex connection.
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
    /// Uses the C# approach ported from TransientGraphUtility.
    /// FindNearestPerpendicularOrContainingEdge (lines 294-329):
    ///
    /// 1. O(log V) coordinate-index lookup to find seed vertices on the
    ///    nearest perpendicular VG lines in the search direction.
    /// 2. O(chain) walk from each seed vertex along the perpendicular line
    ///    to find the edge that brackets the port's perpendicular coordinate.
    ///
    /// Total complexity: O(k * (log V + chain_length)) where k is the number
    /// of perpendicular lines checked (typically 1-3, usually the first one
    /// succeeds). Replaces the former O(V) vertex scan approach.
    ///
    /// Ported from C# TransientGraphUtility.FindNearestPerpendicularOrContainingEdge
    /// (lines 294-329) and StaticGraphUtility.FindAdjacentVertex.
    ///
    /// Returns (edge_source, edge_target, intersection_point).
    fn find_nearest_crossing_edge(
        graph: &VisibilityGraph,
        location: Point,
        direction: CompassDirection,
    ) -> Option<(VertexId, VertexId, Point)> {
        let eps = GeomConstants::DISTANCE_EPSILON;
        let is_horizontal = matches!(direction, CompassDirection::East | CompassDirection::West);

        // Phase 1: Use coordinate indices to find seed vertices on
        // perpendicular VG lines in the search direction.
        //
        // For horizontal search (East/West): find vertical VG lines (X coords)
        //   in the search direction, get the vertex closest to port.Y on each.
        // For vertical search (North/South): find horizontal VG lines (Y coords)
        //   in the search direction, get the vertex closest to port.X on each.
        //
        // This is O(log V) per line via BTreeMap range queries, replacing
        // the former O(V) scan over all vertices.
        let seeds = graph.find_perpendicular_line_seeds(location, direction);

        if seeds.is_empty() {
            return None;
        }

        // Phase 2: Iterate perpendicular lines nearest to port first.
        // For each seed vertex, walk the VG adjacency chain to find
        // the edge that brackets the port's perpendicular coordinate.
        // This is the same chain walk as the original Phase 2.
        let mut best: Option<(VertexId, VertexId, Point, f64)> = None;

        for &(seed_vid, seed_primary_dist) in &seeds {
            // Early termination: if we already found a crossing at distance d,
            // skip any perpendicular line that is farther away.
            if let Some((_, _, _, best_dist)) = best {
                if seed_primary_dist > best_dist + eps {
                    break;
                }
            }

            // Walk from this seed vertex using the VG adjacency chain to
            // find if there's a perpendicular edge that brackets the port's
            // coordinate.
            if let Some((edge_src, edge_tgt)) =
                Self::find_bracketing_edge_from_vertex(graph, seed_vid, location, direction)
            {
                let src_pt = graph.point(edge_src);
                let tgt_pt = graph.point(edge_tgt);
                let intersect = compute_intersection(location, direction, src_pt, tgt_pt);
                let dist = if is_horizontal {
                    (intersect.x() - location.x()).abs()
                } else {
                    (intersect.y() - location.y()).abs()
                };
                if best.is_none_or(|(_, _, _, d)| dist < d) {
                    best = Some((edge_src, edge_tgt, intersect, dist));
                }
            }
        }

        best.map(|(s, t, p, _)| (s, t, p))
    }

    /// From a vertex on a perpendicular VG line, walk the adjacency chain
    /// to find an edge that brackets the port's perpendicular coordinate.
    ///
    /// This is the core of the C# adjacency walk: from a vertex on a
    /// perpendicular line, walk toward the port along that line to find
    /// the edge spanning the port's coordinate.
    ///
    /// Ported from C# TransientGraphUtility.FindPerpendicularOrContainingEdge
    /// (lines 270-292).
    fn find_bracketing_edge_from_vertex(
        graph: &VisibilityGraph,
        start_vertex: VertexId,
        location: Point,
        search_dir: CompassDirection,
    ) -> Option<(VertexId, VertexId)> {
        let is_horizontal = matches!(search_dir, CompassDirection::East | CompassDirection::West);

        // Determine the perpendicular direction from the vertex toward the
        // port's perpendicular coordinate.
        let start_pt = graph.point(start_vertex);
        let perp_to_port = if is_horizontal {
            // Search is horizontal; perpendicular axis is vertical.
            let dy = location.y() - start_pt.y();
            if dy > GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::North)
            } else if dy < -GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::South)
            } else {
                None // Already on the same Y -- check for edge in search direction.
            }
        } else {
            // Search is vertical; perpendicular axis is horizontal.
            let dx = location.x() - start_pt.x();
            if dx > GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::East)
            } else if dx < -GeomConstants::DISTANCE_EPSILON {
                Some(CompassDirection::West)
            } else {
                None // Already on the same X -- check for edge in search direction.
            }
        };

        match perp_to_port {
            None => {
                // The start vertex is on the same perpendicular coordinate as
                // the port. The "perpendicular edge" is the edge from this
                // vertex in the search direction (if it exists).
                // Return the edge as (start_vertex, next_vertex).
                let next = StaticGraphUtility::find_adjacent_vertex(
                    graph,
                    start_vertex,
                    search_dir,
                )?;
                // Verify the edge crosses or is past the port.
                let next_pt = graph.point(next);
                let crosses = if is_horizontal {
                    match search_dir {
                        CompassDirection::East => {
                            next_pt.x() >= location.x() - GeomConstants::DISTANCE_EPSILON
                        }
                        CompassDirection::West => {
                            next_pt.x() <= location.x() + GeomConstants::DISTANCE_EPSILON
                        }
                        _ => unreachable!(),
                    }
                } else {
                    match search_dir {
                        CompassDirection::North => {
                            next_pt.y() >= location.y() - GeomConstants::DISTANCE_EPSILON
                        }
                        CompassDirection::South => {
                            next_pt.y() <= location.y() + GeomConstants::DISTANCE_EPSILON
                        }
                        _ => unreachable!(),
                    }
                };
                if crosses {
                    Some((start_vertex, next))
                } else {
                    None
                }
            }
            Some(perp_dir) => {
                // Walk from start_vertex toward the port's perpendicular
                // coordinate. When we find an edge that brackets it, return
                // that edge. This is FindPerpendicularOrContainingEdge.
                let mut current = start_vertex;
                loop {
                    let next = StaticGraphUtility::find_adjacent_vertex(
                        graph, current, perp_dir,
                    );
                    match next {
                        None => return None, // Dead end before reaching port's coordinate.
                        Some(next_id) => {
                            let next_pt = graph.point(next_id);
                            let (next_perp, port_perp) = if is_horizontal {
                                (next_pt.y(), location.y())
                            } else {
                                (next_pt.x(), location.x())
                            };

                            // Check if the edge from current to next brackets the
                            // port's perpendicular coordinate.
                            let current_pt = graph.point(current);
                            let current_perp = if is_horizontal {
                                current_pt.y()
                            } else {
                                current_pt.x()
                            };

                            let brackets = ordered_brackets(
                                current_perp, next_perp, port_perp,
                            );

                            if brackets {
                                // This edge brackets the port's perpendicular
                                // coordinate. The crossing edge is between
                                // current and next on this perpendicular line.
                                return graph
                                    .find_edge(current, next_id)
                                    .map(|_| (current, next_id))
                                    .or_else(|| {
                                        graph
                                            .find_edge(next_id, current)
                                            .map(|_| (next_id, current))
                                    });
                            }

                            // If next is past the port's coordinate, no bracket found.
                            let overshot = match perp_dir {
                                CompassDirection::North | CompassDirection::East => {
                                    next_perp > port_perp + GeomConstants::DISTANCE_EPSILON
                                }
                                CompassDirection::South | CompassDirection::West => {
                                    next_perp < port_perp - GeomConstants::DISTANCE_EPSILON
                                }
                            };
                            if overshot && !brackets {
                                return None;
                            }

                            current = next_id;
                        }
                    }
                }
            }
        }
    }

    /// Find the nearest vertex axis-aligned with `location` in `direction`.
    ///
    /// Uses the VisibilityGraph's coordinate indices for O(log V) lookup
    /// instead of scanning all vertices. For East/West, finds the nearest
    /// vertex on the same Y line; for North/South, on the same X line.
    fn find_nearest_aligned(
        graph: &VisibilityGraph,
        port_vertex: VertexId,
        location: Point,
        direction: CompassDirection,
    ) -> Option<(VertexId, f64)> {
        let vid = graph.find_nearest_vertex_in_direction(location, direction)?;
        if vid == port_vertex {
            return None;
        }
        let pt = graph.point(vid);

        // Verify the vertex is actually axis-aligned (same X for N/S, same Y for E/W).
        let is_aligned = match direction {
            CompassDirection::East | CompassDirection::West => {
                (pt.y() - location.y()).abs() < GeomConstants::DISTANCE_EPSILON
            }
            CompassDirection::North | CompassDirection::South => {
                (pt.x() - location.x()).abs() < GeomConstants::DISTANCE_EPSILON
            }
        };

        if !is_aligned {
            return None;
        }

        let dist =
            ((pt.x() - location.x()).powi(2) + (pt.y() - location.y()).powi(2)).sqrt();
        Some((vid, dist))
    }
}

/// Check if `port_coord` is between `a` and `b` (inclusive with epsilon tolerance).
fn ordered_brackets(a: f64, b: f64, port_coord: f64) -> bool {
    let eps = GeomConstants::DISTANCE_EPSILON;
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    port_coord >= lo - eps && port_coord <= hi + eps
}

/// Compute the intersection point of an axis-aligned ray from `location`
/// in `direction` with the edge from `src` to `tgt`.
///
/// For horizontal rays (East/West), the intersection is at (edge_x, location.y).
/// For vertical rays (North/South), the intersection is at (location.x, edge_y).
fn compute_intersection(
    location: Point,
    direction: CompassDirection,
    src: Point,
    _tgt: Point,
) -> Point {
    match direction {
        CompassDirection::East | CompassDirection::West => {
            // Horizontal ray intersects a vertical edge.
            // Both endpoints share the same X in a rectilinear VG.
            Point::new(src.x(), location.y())
        }
        CompassDirection::North | CompassDirection::South => {
            // Vertical ray intersects a horizontal edge.
            // Both endpoints share the same Y in a rectilinear VG.
            Point::new(location.x(), src.y())
        }
    }
}
