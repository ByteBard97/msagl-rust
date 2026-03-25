//! Hard invariant checks for routing results.
//!
//! Ported from `RectilinearVerifier.cs` — only the subset needed for
//! the first batch of tests.

use msagl_rust::routing::rectilinear_edge_router::RoutingResult;
use msagl_rust::routing::shape::Shape;
use msagl_rust::geometry::point::Point;
use msagl_rust::geometry::rectangle::Rectangle;

/// Tolerance for floating-point comparisons in axis alignment checks.
const AXIS_TOLERANCE: f64 = 1e-6;
/// Small margin for obstacle boundary touching (paths may graze boundaries).
const BOUNDARY_EPSILON: f64 = 0.1;

/// Routing result verifier.
///
/// Provides static methods that assert hard invariants on a `RoutingResult`.
pub struct Verifier;

impl Verifier {
    /// Run all hard invariant checks on a routing result.
    pub fn verify_all(result: &RoutingResult, shapes: &[Shape], padding: f64) {
        Self::assert_all_edges_routed(result);
        Self::assert_rectilinear(result);
        Self::assert_no_obstacle_crossings(result, shapes, padding);
    }

    /// Every edge should have been routed (i.e. have at least 2 waypoints).
    pub fn assert_all_edges_routed(result: &RoutingResult) {
        for (i, edge) in result.edges.iter().enumerate() {
            assert!(
                edge.points.len() >= 2,
                "Edge {} was not routed (has {} points)",
                i,
                edge.points.len()
            );
        }
    }

    /// All path segments must be axis-aligned (rectilinear).
    pub fn assert_rectilinear(result: &RoutingResult) {
        for (i, edge) in result.edges.iter().enumerate() {
            for (j, w) in edge.points.windows(2).enumerate() {
                let dx = (w[0].x() - w[1].x()).abs();
                let dy = (w[0].y() - w[1].y()).abs();
                assert!(
                    dx < AXIS_TOLERANCE || dy < AXIS_TOLERANCE,
                    "Edge {} segment {} is not rectilinear: ({}, {}) -> ({}, {})",
                    i,
                    j,
                    w[0].x(),
                    w[0].y(),
                    w[1].x(),
                    w[1].y()
                );
            }
        }
    }

    /// No path segment midpoint should lie strictly inside a padded obstacle,
    /// unless that obstacle contains the path's source or target endpoint.
    ///
    /// Ports are typically at obstacle centers, so the first/last path segments
    /// naturally pass through the source/target obstacle. Only intermediate
    /// obstacle crossings are violations.
    pub fn assert_no_obstacle_crossings(
        result: &RoutingResult,
        shapes: &[Shape],
        padding: f64,
    ) {
        let padded_bboxes: Vec<Rectangle> = shapes
            .iter()
            .map(|shape| {
                let bb = shape.bounding_box();
                Rectangle::new(
                    bb.left() - padding,
                    bb.bottom() - padding,
                    bb.right() + padding,
                    bb.top() + padding,
                )
            })
            .collect();

        for (i, edge) in result.edges.iter().enumerate() {
            let source = edge.points.first().unwrap();
            let target = edge.points.last().unwrap();

            // Find which obstacles contain the source and target endpoints.
            let endpoint_obstacles: Vec<usize> = padded_bboxes
                .iter()
                .enumerate()
                .filter(|(_k, padded)| {
                    Self::point_inside(source, padded) || Self::point_inside(target, padded)
                })
                .map(|(k, _)| k)
                .collect();

            for (j, w) in edge.points.windows(2).enumerate() {
                let mid = Point::new(
                    (w[0].x() + w[1].x()) / 2.0,
                    (w[0].y() + w[1].y()) / 2.0,
                );
                for (k, padded) in padded_bboxes.iter().enumerate() {
                    // Skip source/target obstacles — the path naturally exits
                    // through them.
                    if endpoint_obstacles.contains(&k) {
                        continue;
                    }
                    let strictly_inside = mid.x() > padded.left() + BOUNDARY_EPSILON
                        && mid.x() < padded.right() - BOUNDARY_EPSILON
                        && mid.y() > padded.bottom() + BOUNDARY_EPSILON
                        && mid.y() < padded.top() - BOUNDARY_EPSILON;
                    assert!(
                        !strictly_inside,
                        "Edge {} segment {} passes through obstacle {}: \
                         midpoint ({}, {}) is inside padded bbox \
                         [{}, {}, {}, {}]",
                        i,
                        j,
                        k,
                        mid.x(),
                        mid.y(),
                        padded.left(),
                        padded.bottom(),
                        padded.right(),
                        padded.top()
                    );
                }
            }
        }
    }

    /// Check if a point is strictly inside a padded rectangle.
    fn point_inside(p: &Point, r: &Rectangle) -> bool {
        p.x() > r.left() + BOUNDARY_EPSILON
            && p.x() < r.right() - BOUNDARY_EPSILON
            && p.y() > r.bottom() + BOUNDARY_EPSILON
            && p.y() < r.top() - BOUNDARY_EPSILON
    }
}
