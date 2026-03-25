use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use super::compass_direction::CompassDirection;

/// Utility functions for static visibility-graph analysis.
///
/// Ported from `StaticGraphUtility.ts`.
pub struct StaticGraphUtility;

impl StaticGraphUtility {
    /// Determine the compass direction from `source` to `target`.
    ///
    /// Returns `None` when the two points are identical.
    pub fn edge_direction(source: Point, target: Point) -> Option<CompassDirection> {
        CompassDirection::from_points(source, target)
    }

    /// Find the next vertex adjacent to `vertex` in the given direction.
    ///
    /// Checks both out-edges and in-edges so that the traversal works on
    /// directed graphs stored with bidirectional links.
    pub fn find_adjacent_vertex(
        graph: &VisibilityGraph,
        vertex: VertexId,
        direction: CompassDirection,
    ) -> Option<VertexId> {
        let p = graph.point(vertex);

        for edge in graph.out_edges(vertex) {
            let tp = graph.point(edge.target);
            if let Some(d) = CompassDirection::from_points(p, tp) {
                if d == direction {
                    return Some(edge.target);
                }
            }
        }

        for &src in &graph.vertex(vertex).in_edges {
            let sp = graph.point(src);
            if let Some(d) = CompassDirection::from_points(p, sp) {
                if d == direction {
                    return Some(src);
                }
            }
        }

        None
    }

    /// Check if a point lies strictly inside a rectangle (not on the boundary).
    pub fn point_is_in_rectangle_interior(point: Point, rect: &Rectangle) -> bool {
        point.x() > rect.left()
            && point.x() < rect.right()
            && point.y() > rect.bottom()
            && point.y() < rect.top()
    }

    /// Check if two points share a horizontal or vertical coordinate
    /// (i.e. they are collinear with either axis).
    pub fn is_collinear(a: Point, b: Point) -> bool {
        (a.x() - b.x()).abs() < 1e-10 || (a.y() - b.y()).abs() < 1e-10
    }

    /// Find the out-edge from `vertex` in the given direction.
    ///
    /// Returns `(target_vertex_id, weight)` if found.
    pub fn find_adjacent_edge(
        graph: &VisibilityGraph,
        vertex: VertexId,
        direction: CompassDirection,
    ) -> Option<(VertexId, f64)> {
        let p = graph.point(vertex);
        for edge in graph.out_edges(vertex) {
            let tp = graph.point(edge.target);
            if let Some(d) = CompassDirection::from_points(p, tp) {
                if d == direction {
                    return Some((edge.target, edge.weight));
                }
            }
        }
        None
    }

    /// Is `a` "lower" than `b` in ascending order?
    ///
    /// Returns true if the direction from `a` to `b` is East or North.
    /// Ported from `PointComparer.IsPureLower`.
    pub fn is_pure_lower(a: Point, b: Point) -> bool {
        matches!(
            CompassDirection::from_points(a, b),
            Some(CompassDirection::East) | Some(CompassDirection::North),
        )
    }

    /// Is the given direction vertical (North or South)?
    pub fn is_vertical(dir: CompassDirection) -> bool {
        matches!(dir, CompassDirection::North | CompassDirection::South)
    }

    /// Find the bend point between two non-collinear points.
    ///
    /// The `final_edge_dir` determines which axis the final edge uses.
    /// Ported from `StaticGraphUtility.FindBendPointBetween`.
    pub fn find_bend_point_between(
        source: Point,
        target: Point,
        final_edge_dir: CompassDirection,
    ) -> Point {
        if !Self::is_vertical(final_edge_dir) {
            // Final edge is horizontal, so bend is at (source.x, target.y)
            Point::new(source.x(), target.y())
        } else {
            // Final edge is vertical, so bend is at (target.x, source.y)
            Point::new(target.x(), source.y())
        }
    }
}