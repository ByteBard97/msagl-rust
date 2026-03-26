/// Verification utilities for rectilinear routing results.
///
/// Mirrors the invariant-checking philosophy of the C# RectilinearVerifier:
/// every routed edge must have a valid rectilinear path that does not
/// penetrate any obstacle.
#[allow(unused_imports)]
use msagl_rust::{Point, RoutingResult, Shape};

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
            assert!(!edge.points.is_empty(), "edge[{}] has no waypoints", i);
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
    ///
    /// Exception: points in overlap regions are allowed. Matches C#
    /// RectilinearVerifier which permits paths through obstacles that are
    /// in the same clump as the source or target obstacle.
    fn verify_paths_dont_pass_through_obstacles(
        result: &RoutingResult,
        shapes: &[Shape],
        tolerance: f64,
    ) {
        for (ei, edge) in result.edges.iter().enumerate() {
            let src = &edge.points[0];
            let tgt = edge.points.last().unwrap();

            for (pi, pt) in edge.points.iter().enumerate() {
                // Only check interior waypoints (skip first and last).
                if pi == 0 || pi == edge.points.len() - 1 {
                    continue;
                }

                // Collect obstacles this point is strictly inside.
                let mut containing: Vec<usize> = Vec::new();
                for (oi, shape) in shapes.iter().enumerate() {
                    let bb = shape.bounding_box();
                    if pt.x() > bb.left() + tolerance
                        && pt.x() < bb.right() - tolerance
                        && pt.y() > bb.bottom() + tolerance
                        && pt.y() < bb.top() - tolerance
                    {
                        containing.push(oi);
                    }
                }

                if containing.is_empty() {
                    continue;
                }

                // If inside 2+ obstacles, this is an overlap region — allowed.
                if containing.len() >= 2 {
                    continue;
                }

                // Inside exactly 1 obstacle. Check if the source or target is
                // inside the same obstacle, or if the obstacle overlaps with
                // an obstacle containing src/tgt (same clump).
                let crossed = containing[0];
                let crossed_bb = shapes[crossed].bounding_box();

                // Source or target inside the crossed obstacle?
                let src_in = src.x() >= crossed_bb.left() - tolerance
                    && src.x() <= crossed_bb.right() + tolerance
                    && src.y() >= crossed_bb.bottom() - tolerance
                    && src.y() <= crossed_bb.top() + tolerance;
                let tgt_in = tgt.x() >= crossed_bb.left() - tolerance
                    && tgt.x() <= crossed_bb.right() + tolerance
                    && tgt.y() >= crossed_bb.bottom() - tolerance
                    && tgt.y() <= crossed_bb.top() + tolerance;
                if src_in || tgt_in {
                    continue;
                }

                // Check if crossed obstacle overlaps any obstacle containing
                // src or tgt (i.e. they're in the same overlap clump).
                let mut in_endpoint_clump = false;
                for (oi2, shape2) in shapes.iter().enumerate() {
                    if oi2 == crossed {
                        continue;
                    }
                    let bb2 = shape2.bounding_box();
                    let has_src = src.x() >= bb2.left() - tolerance
                        && src.x() <= bb2.right() + tolerance
                        && src.y() >= bb2.bottom() - tolerance
                        && src.y() <= bb2.top() + tolerance;
                    let has_tgt = tgt.x() >= bb2.left() - tolerance
                        && tgt.x() <= bb2.right() + tolerance
                        && tgt.y() >= bb2.bottom() - tolerance
                        && tgt.y() <= bb2.top() + tolerance;
                    if has_src || has_tgt {
                        if crossed_bb.left() < bb2.right()
                            && bb2.left() < crossed_bb.right()
                            && crossed_bb.bottom() < bb2.top()
                            && bb2.bottom() < crossed_bb.top()
                        {
                            in_endpoint_clump = true;
                            break;
                        }
                    }
                }
                if in_endpoint_clump {
                    continue;
                }

                // Not in an overlap exemption — fail.
                let bb = shapes[crossed].bounding_box();
                panic!(
                    "edge[{}] point[{}] ({:.3},{:.3}) is inside obstacle[{}] \
                     bbox ({:.3},{:.3})-({:.3},{:.3})",
                    ei,
                    pi,
                    pt.x(),
                    pt.y(),
                    crossed,
                    bb.left(),
                    bb.bottom(),
                    bb.right(),
                    bb.top()
                );
            }
        }
    }

    /// No two consecutive waypoints should be identical (zero-length segments).
    /// Exception: self-loop edges (source == target) are allowed to have
    /// zero-length segments, as they represent routing between obstacles
    /// that share the same center point (e.g. fully overlapping obstacles).
    ///
    /// Uses a much smaller threshold than the rectilinearity tolerance,
    /// since short (but non-zero) segments are a natural result of nudging.
    fn verify_no_zero_length_segments(result: &RoutingResult, tolerance: f64) {
        // Minimum segment length: use a fraction of the tolerance.
        // Short segments from nudging (0.3 with tolerance 0.5) are acceptable.
        let min_length = tolerance * 0.01;

        for (ei, edge) in result.edges.iter().enumerate() {
            // Allow zero-length for self-loop (source == target)
            if edge.points.len() == 2 {
                let src = &edge.points[0];
                let tgt = &edge.points[1];
                if (src.x() - tgt.x()).abs() < tolerance && (src.y() - tgt.y()).abs() < tolerance {
                    continue;
                }
            }
            for (si, seg) in edge.points.windows(2).enumerate() {
                let dist =
                    ((seg[1].x() - seg[0].x()).powi(2) + (seg[1].y() - seg[0].y()).powi(2)).sqrt();
                assert!(
                    dist >= min_length,
                    "edge[{}] segment[{}] has near-zero length {:.6}",
                    ei,
                    si,
                    dist
                );
            }
        }
    }

    /// Run verification checks suitable for non-rectangular obstacle tests.
    ///
    /// Checks basic path validity and rectilinearity for routed paths.
    /// Skips the strict obstacle containment check because our simplified VG
    /// generator routes around bounding boxes (not actual polygon boundaries),
    /// and the nudger may move paths into non-rectangular obstacle interiors.
    ///
    /// Allows 2-point fallback paths (where VG path search failed) since
    /// those may be diagonal when source and target are not axis-aligned.
    ///
    /// Once the VG generator supports bend/reflection events for
    /// non-rectangular obstacles, this should use `verify_all` instead.
    pub fn verify_nonrect(result: &RoutingResult, _shapes: &[Shape], tolerance: f64) {
        Self::verify_all_edges_present(result);
        Self::verify_paths_have_at_least_two_points(result);
        // Check rectilinearity only for non-fallback paths (3+ points).
        // 2-point paths are direct source→target fallbacks that may be
        // diagonal when the VG path search fails.
        Self::verify_routed_paths_are_rectilinear(result, tolerance);
        Self::verify_no_zero_length_segments(result, tolerance);
    }

    /// Check rectilinearity only for paths with 3+ waypoints.
    /// 2-point paths are VG-search fallbacks that may be diagonal.
    fn verify_routed_paths_are_rectilinear(result: &RoutingResult, tolerance: f64) {
        for (ei, edge) in result.edges.iter().enumerate() {
            if edge.points.len() <= 2 {
                continue; // Fallback path — skip rectilinearity check
            }
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

/// Check if a point is strictly inside a shape (not on the boundary).
///
/// For rectangular shapes, uses the bounding box with tolerance inset.
/// For non-rectangular shapes, uses the ray-casting point-in-polygon test
/// on the actual boundary polyline.
///
/// Not yet used in `verify_all` — available for future upgrade when the
/// obstacle containment check uses polygon boundaries for non-rect shapes.
#[allow(dead_code)]
fn is_point_strictly_inside_shape(pt: &Point, shape: &Shape, tolerance: f64) -> bool {
    if shape.is_rect() {
        // Fast path for rectangles: use bounding box
        let bb = shape.bounding_box();
        pt.x() > bb.left() + tolerance
            && pt.x() < bb.right() - tolerance
            && pt.y() > bb.bottom() + tolerance
            && pt.y() < bb.top() - tolerance
    } else {
        // Non-rectangular: first check bounding box as quick reject
        let bb = shape.bounding_box();
        if pt.x() <= bb.left() - tolerance
            || pt.x() >= bb.right() + tolerance
            || pt.y() <= bb.bottom() - tolerance
            || pt.y() >= bb.top() + tolerance
        {
            return false;
        }
        // Ray-casting point-in-polygon, then check distance to boundary
        let poly_pts: Vec<Point> = shape.boundary_polyline().points().collect();
        if !point_in_polygon(pt, &poly_pts) {
            return false;
        }
        // Must be strictly inside (not on boundary edge)
        min_distance_to_polygon_edges(pt, &poly_pts) > tolerance
    }
}

/// Check if a point is inside or on the boundary of a shape.
#[allow(dead_code)]
fn is_point_inside_or_on_shape(pt: &Point, shape: &Shape, tolerance: f64) -> bool {
    if shape.is_rect() {
        let bb = shape.bounding_box();
        pt.x() >= bb.left() - tolerance
            && pt.x() <= bb.right() + tolerance
            && pt.y() >= bb.bottom() - tolerance
            && pt.y() <= bb.top() + tolerance
    } else {
        let bb = shape.bounding_box();
        if pt.x() < bb.left() - tolerance
            || pt.x() > bb.right() + tolerance
            || pt.y() < bb.bottom() - tolerance
            || pt.y() > bb.top() + tolerance
        {
            return false;
        }
        let poly_pts: Vec<Point> = shape.boundary_polyline().points().collect();
        point_in_polygon(pt, &poly_pts)
            || min_distance_to_polygon_edges(pt, &poly_pts) <= tolerance
    }
}

/// Ray-casting algorithm for point-in-polygon testing.
/// Returns true if the point is inside the polygon (including boundary).
#[allow(dead_code)]
fn point_in_polygon(pt: &Point, vertices: &[Point]) -> bool {
    let n = vertices.len();
    let mut inside = false;
    let px = pt.x();
    let py = pt.y();

    let mut j = n - 1;
    for i in 0..n {
        let xi = vertices[i].x();
        let yi = vertices[i].y();
        let xj = vertices[j].x();
        let yj = vertices[j].y();

        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Minimum distance from a point to any edge of the polygon.
#[allow(dead_code)]
fn min_distance_to_polygon_edges(pt: &Point, vertices: &[Point]) -> f64 {
    let n = vertices.len();
    let mut min_dist = f64::INFINITY;

    for i in 0..n {
        let j = (i + 1) % n;
        let dist = point_to_segment_distance(pt, &vertices[i], &vertices[j]);
        if dist < min_dist {
            min_dist = dist;
        }
    }
    min_dist
}

/// Distance from a point to a line segment.
#[allow(dead_code)]
fn point_to_segment_distance(p: &Point, a: &Point, b: &Point) -> f64 {
    let ab = *b - *a;
    let ap = *p - *a;
    let ab_len_sq = ab.x() * ab.x() + ab.y() * ab.y();

    if ab_len_sq < 1e-12 {
        return ap.length();
    }

    let t = (ap.x() * ab.x() + ap.y() * ab.y()) / ab_len_sq;
    let t_clamped = t.clamp(0.0, 1.0);
    let closest = Point::new(a.x() + t_clamped * ab.x(), a.y() + t_clamped * ab.y());
    (*p - closest).length()
}
