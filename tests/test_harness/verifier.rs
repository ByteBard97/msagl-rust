/// Verification utilities for rectilinear routing results.
///
/// Mirrors the invariant-checking philosophy of the C# RectilinearVerifier:
/// every routed edge must have a valid rectilinear path that does not
/// penetrate any obstacle.
use msagl_rust::{RoutingResult, Shape};

/// Tolerance for floating-point comparisons when checking rectilinearity.
const RECTILINEAR_TOL: f64 = 0.5;

pub struct Verifier;

impl Verifier {
    /// Run all standard invariant checks on a routing result.
    ///
    /// `tolerance` — absolute epsilon for geometric comparisons.
    ///
    /// Panics with a descriptive message if any invariant is violated.
    pub fn verify_all(result: &RoutingResult, shapes: &[Shape], tolerance: f64) {
        Self::verify_all_edges_present(result);
        Self::verify_paths_have_at_least_two_points(result);
        Self::verify_paths_are_rectilinear(result, tolerance);
        Self::verify_paths_dont_pass_through_obstacles(result, shapes, tolerance);
        Self::verify_no_zero_length_segments(result, tolerance);
    }

    /// Every edge in the result must carry at least one path.
    fn verify_all_edges_present(result: &RoutingResult) {
        for (i, edge) in result.edges.iter().enumerate() {
            assert!(
                !edge.points.is_empty(),
                "edge[{}] has no waypoints",
                i
            );
        }
    }

    /// Every routed path must have at least two waypoints (source and target).
    fn verify_paths_have_at_least_two_points(result: &RoutingResult) {
        for (i, edge) in result.edges.iter().enumerate() {
            assert!(
                edge.points.len() >= 2,
                "edge[{}] has only {} waypoint(s); need at least 2",
                i,
                edge.points.len()
            );
        }
    }

    /// All segments must be axis-aligned (no diagonal segments).
    fn verify_paths_are_rectilinear(result: &RoutingResult, tolerance: f64) {
        for (ei, edge) in result.edges.iter().enumerate() {
            for (si, seg) in edge.points.windows(2).enumerate() {
                let dx = (seg[1].x() - seg[0].x()).abs();
                let dy = (seg[1].y() - seg[0].y()).abs();
                let is_horizontal = dy < tolerance;
                let is_vertical = dx < tolerance;
                assert!(
                    is_horizontal || is_vertical,
                    "edge[{}] segment[{}] is diagonal: ({:.3},{:.3}) -> ({:.3},{:.3})",
                    ei,
                    si,
                    seg[0].x(),
                    seg[0].y(),
                    seg[1].x(),
                    seg[1].y()
                );
            }
        }
    }

    /// No waypoint of any routed path should land strictly inside an obstacle
    /// (within the padded bounding box).  The path may touch obstacle borders.
    fn verify_paths_dont_pass_through_obstacles(
        result: &RoutingResult,
        shapes: &[Shape],
        tolerance: f64,
    ) {
        for (ei, edge) in result.edges.iter().enumerate() {
            for (pi, pt) in edge.points.iter().enumerate() {
                // Only check interior waypoints (skip first and last which sit on
                // the obstacle boundary by construction).
                if pi == 0 || pi == edge.points.len() - 1 {
                    continue;
                }
                for (oi, shape) in shapes.iter().enumerate() {
                    let bb = shape.bounding_box();
                    let strictly_inside = pt.x() > bb.left() + tolerance
                        && pt.x() < bb.right() - tolerance
                        && pt.y() > bb.bottom() + tolerance
                        && pt.y() < bb.top() - tolerance;
                    assert!(
                        !strictly_inside,
                        "edge[{}] point[{}] ({:.3},{:.3}) is inside obstacle[{}] \
                         bbox ({:.3},{:.3})-({:.3},{:.3})",
                        ei,
                        pi,
                        pt.x(),
                        pt.y(),
                        oi,
                        bb.left(),
                        bb.bottom(),
                        bb.right(),
                        bb.top()
                    );
                }
            }
        }
    }

    /// No two consecutive waypoints should be identical (zero-length segments).
    fn verify_no_zero_length_segments(result: &RoutingResult, tolerance: f64) {
        for (ei, edge) in result.edges.iter().enumerate() {
            for (si, seg) in edge.points.windows(2).enumerate() {
                let dist = ((seg[1].x() - seg[0].x()).powi(2)
                    + (seg[1].y() - seg[0].y()).powi(2))
                .sqrt();
                assert!(
                    dist >= tolerance,
                    "edge[{}] segment[{}] has near-zero length {:.6}",
                    ei,
                    si,
                    dist
                );
            }
        }
    }

    /// Assert exactly `expected` edges were routed.
    pub fn assert_edge_count(result: &RoutingResult, expected: usize) {
        assert_eq!(
            result.edges.len(),
            expected,
            "expected {} routed edge(s), got {}",
            expected,
            result.edges.len()
        );
    }

    /// Assert that edges are separated by at least `min_separation` on any
    /// shared horizontal or vertical coordinate.
    #[allow(dead_code)]
    pub fn assert_edges_separated(result: &RoutingResult, min_separation: f64) {
        let edges = &result.edges;
        for i in 0..edges.len() {
            for j in (i + 1)..edges.len() {
                for pi in &edges[i].points {
                    for pj in &edges[j].points {
                        // Collinear horizontal: same Y, check X separation.
                        if (pi.y() - pj.y()).abs() < 0.01 {
                            let sep = (pi.x() - pj.x()).abs();
                            if sep < min_separation * 5.0 {
                                assert!(
                                    sep >= min_separation - 0.5,
                                    "edges {} and {} have collinear horizontal points \
                                     too close: sep={:.3} < {:.3}",
                                    i,
                                    j,
                                    sep,
                                    min_separation
                                );
                            }
                        }
                        // Collinear vertical: same X, check Y separation.
                        if (pi.x() - pj.x()).abs() < 0.01 {
                            let sep = (pi.y() - pj.y()).abs();
                            if sep < min_separation * 5.0 {
                                assert!(
                                    sep >= min_separation - 0.5,
                                    "edges {} and {} have collinear vertical points \
                                     too close: sep={:.3} < {:.3}",
                                    i,
                                    j,
                                    sep,
                                    min_separation
                                );
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Rectilinear tolerance constant reused by tests.
pub const RECTILINEAR_TOLERANCE: f64 = RECTILINEAR_TOL;
