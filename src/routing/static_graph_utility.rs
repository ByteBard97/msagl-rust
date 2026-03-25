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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::visibility::graph::VisibilityGraph;

    // -----------------------------------------------------------------------
    // edge_direction
    // -----------------------------------------------------------------------

    #[test]
    fn edge_direction_horizontal() {
        let src = Point::new(0.0, 0.0);
        assert_eq!(
            StaticGraphUtility::edge_direction(src, Point::new(10.0, 0.0)),
            Some(CompassDirection::East),
        );
        assert_eq!(
            StaticGraphUtility::edge_direction(src, Point::new(-5.0, 0.0)),
            Some(CompassDirection::West),
        );
    }

    #[test]
    fn edge_direction_vertical() {
        let src = Point::new(0.0, 0.0);
        assert_eq!(
            StaticGraphUtility::edge_direction(src, Point::new(0.0, 7.0)),
            Some(CompassDirection::North),
        );
        assert_eq!(
            StaticGraphUtility::edge_direction(src, Point::new(0.0, -3.0)),
            Some(CompassDirection::South),
        );
    }

    #[test]
    fn edge_direction_identical_points() {
        let p = Point::new(1.0, 1.0);
        assert_eq!(StaticGraphUtility::edge_direction(p, p), None);
    }

    // -----------------------------------------------------------------------
    // find_adjacent_vertex
    // -----------------------------------------------------------------------

    #[test]
    fn find_adjacent_vertex_via_out_edge() {
        let mut g = VisibilityGraph::new();
        let a = g.add_vertex(Point::new(0.0, 0.0));
        let b = g.add_vertex(Point::new(10.0, 0.0));
        g.add_edge(a, b, 10.0);

        assert_eq!(
            StaticGraphUtility::find_adjacent_vertex(&g, a, CompassDirection::East),
            Some(b),
        );
        assert_eq!(
            StaticGraphUtility::find_adjacent_vertex(&g, a, CompassDirection::North),
            None,
        );
    }

    #[test]
    fn find_adjacent_vertex_via_in_edge() {
        // `b` has `a` as an in-edge; traversing West from `b` should find `a`.
        let mut g = VisibilityGraph::new();
        let a = g.add_vertex(Point::new(0.0, 0.0));
        let b = g.add_vertex(Point::new(10.0, 0.0));
        g.add_edge(a, b, 10.0);

        assert_eq!(
            StaticGraphUtility::find_adjacent_vertex(&g, b, CompassDirection::West),
            Some(a),
        );
    }

    // -----------------------------------------------------------------------
    // point_is_in_rectangle_interior
    // -----------------------------------------------------------------------

    #[test]
    fn point_in_rectangle_interior_true() {
        let rect = Rectangle::new(0.0, 0.0, 10.0, 10.0);
        assert!(StaticGraphUtility::point_is_in_rectangle_interior(
            Point::new(5.0, 5.0),
            &rect,
        ));
    }

    #[test]
    fn point_on_rectangle_boundary_false() {
        let rect = Rectangle::new(0.0, 0.0, 10.0, 10.0);
        // On the left edge — not strictly interior.
        assert!(!StaticGraphUtility::point_is_in_rectangle_interior(
            Point::new(0.0, 5.0),
            &rect,
        ));
    }

    #[test]
    fn point_outside_rectangle_false() {
        let rect = Rectangle::new(0.0, 0.0, 10.0, 10.0);
        assert!(!StaticGraphUtility::point_is_in_rectangle_interior(
            Point::new(15.0, 5.0),
            &rect,
        ));
    }

    // -----------------------------------------------------------------------
    // is_collinear
    // -----------------------------------------------------------------------

    #[test]
    fn is_collinear_same_x() {
        assert!(StaticGraphUtility::is_collinear(
            Point::new(3.0, 0.0),
            Point::new(3.0, 10.0),
        ));
    }

    #[test]
    fn is_collinear_same_y() {
        assert!(StaticGraphUtility::is_collinear(
            Point::new(0.0, 5.0),
            Point::new(8.0, 5.0),
        ));
    }

    #[test]
    fn is_not_collinear_diagonal() {
        assert!(!StaticGraphUtility::is_collinear(
            Point::new(0.0, 0.0),
            Point::new(1.0, 1.0),
        ));
    }
}
