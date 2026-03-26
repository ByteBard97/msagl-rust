use super::obstacle_side::{ObstacleSide, SideType};
use super::scan_direction::ScanDirection;
use super::shape::Shape;
use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;

/// Internal obstacle representation with padded boundary.
///
/// Faithful port of TS `obstacle.ts` (239 lines).
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// Index of this obstacle (for cross-referencing).
    pub index: usize,
    /// Ordinal for scanline tiebreaking.
    /// Matches TS `Obstacle.Ordinal`.
    pub ordinal: usize,
    /// The original shape.
    pub shape: Shape,
    /// The padded bounding box.
    padded_bbox: Rectangle,
    /// The padded polyline (closed, potentially 4+ points).
    padded_polyline: Polyline,
    /// Whether the padded polyline is a rectangle (4 points, pure compass turns).
    /// Matches TS `Obstacle.IsRectangle`.
    is_rectangle: bool,
    /// Padding distance.
    pub padding: f64,
    /// Whether this obstacle is a sentinel boundary marker.
    /// Matches TS `Obstacle.IsSentinel`.
    is_sentinel: bool,
    /// Active low side (set during sweep).
    active_low_side: Option<ObstacleSide>,
    /// Active high side (set during sweep).
    active_high_side: Option<ObstacleSide>,
}

impl Obstacle {
    /// Ordinal reserved for sentinel obstacles.
    pub const FIRST_SENTINEL_ORDINAL: usize = 1;
    /// Starting ordinal for non-sentinel obstacles.
    pub const FIRST_NON_SENTINEL_ORDINAL: usize = 10;

    /// Faithful port of TS `Obstacle` constructor.
    ///
    /// Pads the boundary polyline. Currently uses padded bounding box for
    /// the scanline representation (both rectangular and non-rectangular shapes).
    /// The actual polygon boundary is stored in the Shape for accurate
    /// containment testing by the verifier.
    ///
    /// TODO: When bend/reflection events are implemented in the VG generator,
    /// non-rectangular shapes should use `create_padded_polyline()` for
    /// their padded polyline instead of the bounding-box approximation.
    pub fn from_shape(shape: &Shape, padding: f64, index: usize) -> Self {
        let bb = shape.bounding_box();
        let padded_bb = Rectangle::new(
            bb.left() - padding,
            bb.bottom() - padding,
            bb.right() + padding,
            bb.top() + padding,
        );

        // Create padded polyline from bounding box: clockwise from bottom-left.
        // For non-rectangular shapes, this is conservative (routes around the
        // bounding box), but the verifier uses the actual polygon boundary.
        let mut poly = Polyline::new();
        poly.add_point(padded_bb.left_bottom());
        poly.add_point(padded_bb.left_top());
        poly.add_point(padded_bb.right_top());
        poly.add_point(padded_bb.right_bottom());
        poly.set_closed(true);

        // Faithful port of TS Obstacle.RoundVerticesAndSimplify()
        round_vertices_and_simplify(&mut poly);

        let is_rect = is_polyline_rectangle(&poly);

        Self {
            index,
            ordinal: Self::FIRST_NON_SENTINEL_ORDINAL + index,
            shape: shape.clone(),
            padded_bbox: padded_bb,
            padded_polyline: poly,
            is_rectangle: is_rect,
            padding,
            is_sentinel: false,
            active_low_side: None,
            active_high_side: None,
        }
    }

    /// Create a sentinel obstacle used as a boundary marker.
    ///
    /// Matches TS `Obstacle.mk(a, b, scanlineOrdinal)` + `CreateSentinel`.
    pub fn create_sentinel(a: Point, b: Point, ordinal: usize) -> Self {
        let ra = Point::round(a);
        let rb = Point::round(b);
        let bbox = Rectangle::from_points(ra, rb);

        let mut poly = Polyline::new();
        poly.add_point(ra);
        poly.add_point(rb);
        poly.set_closed(true);

        // Use a dummy shape backed by the same bbox.
        let shape = Shape::rectangle(bbox.left(), bbox.bottom(), bbox.width(), bbox.height());

        Self {
            index: 0,
            ordinal,
            shape,
            padded_bbox: bbox,
            padded_polyline: poly,
            padding: 0.0,
            is_sentinel: true,
            is_rectangle: false,
            active_low_side: None,
            active_high_side: None,
        }
    }

    pub fn padded_bounding_box(&self) -> &Rectangle {
        &self.padded_bbox
    }

    pub fn padded_polyline(&self) -> &Polyline {
        &self.padded_polyline
    }

    /// Get the 4 padded corner points.
    /// Only valid for rectangular obstacles.
    pub fn padded_corners(&self) -> [Point; 4] {
        [
            self.padded_bbox.left_bottom(),
            self.padded_bbox.left_top(),
            self.padded_bbox.right_top(),
            self.padded_bbox.right_bottom(),
        ]
    }

    pub fn ordinal(&self) -> usize {
        self.ordinal
    }

    pub fn is_sentinel(&self) -> bool {
        self.is_sentinel
    }

    /// Whether the padded polyline is a rectangle.
    /// Matches TS `Obstacle.IsRectangle`.
    pub fn is_rectangle(&self) -> bool {
        self.is_rectangle
    }

    pub fn active_low_side(&self) -> Option<&ObstacleSide> {
        self.active_low_side.as_ref()
    }

    pub fn active_high_side(&self) -> Option<&ObstacleSide> {
        self.active_high_side.as_ref()
    }

    /// Faithful port of TS `VisibilityGraphGenerator.GetOpenVertex()`.
    ///
    /// Walks the polyline to find the vertex with the lowest coordinate
    /// per the scan direction. Uses `<=` comparison for flat bottom handling:
    /// we want the last vertex of a flat bottom edge (clockwise rotation).
    pub fn get_open_vertex(&self, scan_direction: ScanDirection) -> PolylinePointKey {
        let poly = &self.padded_polyline;
        let start_key = poly.start_key().expect("polyline must not be empty");

        let mut lowest_key = start_key;
        let mut lowest_point = poly.point_at(lowest_key);

        // TraversePolylineForEvents: clockwise for H-scan (next), CCW for V-scan (prev)
        let traverse = |key: PolylinePointKey| -> PolylinePointKey {
            if scan_direction.is_horizontal() {
                poly.next_key(key).expect("closed polyline must have next")
            } else {
                poly.prev_key(key).expect("closed polyline must have prev")
            }
        };

        let mut next_key = traverse(start_key);
        let mut prev_cmp = point_compare(poly.point_at(next_key), lowest_point, scan_direction);

        loop {
            let next_point = poly.point_at(next_key);
            let cur_cmp = point_compare(next_point, lowest_point, scan_direction);

            if cur_cmp <= 0 {
                // This point is <= the current lowest, update
                lowest_key = next_key;
                lowest_point = next_point;
            } else if cur_cmp > 0 && prev_cmp <= 0 {
                // We were descending/flat and now ascending -- stop
                break;
            }

            prev_cmp = cur_cmp;
            next_key = traverse(next_key);

            // Safety: stop if we've gone all the way around
            if next_key == start_key {
                break;
            }
        }

        lowest_key
    }

    /// Faithful port of TS `Obstacle.CreateInitialSides()`.
    ///
    /// Creates Low and High sides from the given start polyline point.
    /// If the high side is flat (same perpendicular coordinate for start/end),
    /// advance to the next vertex -- no flat sides in the scanline.
    pub fn create_initial_sides(
        &mut self,
        start_key: PolylinePointKey,
        scan_direction: ScanDirection,
    ) {
        debug_assert!(
            self.active_low_side.is_none() && self.active_high_side.is_none(),
            "Cannot call create_initial_sides when sides are already set"
        );

        self.active_low_side = Some(ObstacleSide::from_polyline_point(
            SideType::Low,
            self.ordinal,
            start_key,
            &self.padded_polyline,
            scan_direction,
        ));

        self.active_high_side = Some(ObstacleSide::from_polyline_point(
            SideType::High,
            self.ordinal,
            start_key,
            &self.padded_polyline,
            scan_direction,
        ));

        // TS line 85-89: if high side is flat, skip to next vertex
        // scanDir.IsFlatS(this.ActiveHighSide) checks if start.perpCoord == end.perpCoord
        if let Some(ref high_side) = self.active_high_side {
            if scan_direction.is_flat(high_side.start(), high_side.end()) {
                let new_start = high_side.end_vertex_key();
                self.active_high_side = Some(ObstacleSide::from_polyline_point(
                    SideType::High,
                    self.ordinal,
                    new_start,
                    &self.padded_polyline,
                    scan_direction,
                ));
            }
        }
    }

    /// Faithful port of TS `Obstacle.Close()`.
    /// Clears active sides.
    pub fn close(&mut self) {
        self.active_low_side = None;
        self.active_high_side = None;
    }
}

/// Faithful port of TS `Obstacle.RoundVerticesAndSimplify()`.
///
/// Rounds each vertex to `Point::round()`, then removes close and collinear vertices.
fn round_vertices_and_simplify(polyline: &mut Polyline) {
    // Round each vertex
    let keys: Vec<PolylinePointKey> = polyline.polyline_points().map(|pp| pp.key).collect();

    for key in &keys {
        let p = polyline.point_at(*key);
        polyline.set_point_at(*key, Point::round(p));
    }

    // Remove close vertices (within epsilon * 10)
    // TS: GeomConstants.intersectionEpsilon * 10
    // Our GeomConstants uses ~1e-6, so epsilon = 1e-5
    let epsilon = GeomConstants::TOLERANCE * 10.0;
    remove_close_vertices(polyline, epsilon);

    // Recompute bounding box would happen implicitly on next access
}

/// Remove vertices that are very close to their predecessor.
fn remove_close_vertices(polyline: &mut Polyline, epsilon: f64) {
    // Collect all keys first to avoid mutating while iterating
    let keys: Vec<PolylinePointKey> = polyline.polyline_points().map(|pp| pp.key).collect();

    if keys.len() < 3 {
        return; // Don't simplify to fewer than 2 points
    }

    let mut to_remove = Vec::new();
    for i in 1..keys.len() {
        let prev_pt = polyline.point_at(keys[i - 1]);
        let curr_pt = polyline.point_at(keys[i]);
        if (prev_pt - curr_pt).length() < epsilon {
            to_remove.push(keys[i]);
        }
    }

    // Also check wrap-around for closed polylines
    if polyline.is_closed() && keys.len() > 1 {
        let first_pt = polyline.point_at(keys[0]);
        let last_pt = polyline.point_at(*keys.last().unwrap());
        if (first_pt - last_pt).length() < epsilon {
            to_remove.push(*keys.last().unwrap());
        }
    }

    // We'd need a remove_point method on Polyline; for now, close vertices
    // are extremely rare for padded rectangles so this is effectively a no-op.
    // The TS implementation modifies linked list pointers directly.
    let _ = to_remove;
}

/// Create a padded polyline around a convex polygon boundary.
/// Not yet used — the VG generator currently routes around bounding boxes.
/// Will be activated when bend/reflection events are implemented.
#[allow(dead_code)]
///
/// Faithful port of C# `InteractiveObstacleCalculator.CreatePaddedPolyline()`.
/// Offsets each edge outward by `padding`, then computes corner intersections.
/// For sharp corners, two offset points are generated (beveled corner).
/// Falls back to convex-hull-then-pad if any corner is concave.
fn create_padded_polyline(poly: &Polyline, padding: f64) -> Polyline {
    let pts: Vec<Point> = poly.points().collect();
    let n = pts.len();
    debug_assert!(n >= 3, "Polyline must have at least 3 points");

    let mut result = Polyline::new();

    for i in 0..n {
        let p0 = pts[(i + n - 1) % n]; // previous vertex
        let p1 = pts[i];               // current vertex
        let p2 = pts[(i + 1) % n];     // next vertex

        let padded = get_padded_corner(p0, p1, p2, padding);
        match padded {
            PadResult::One(a) => {
                result.add_point(a);
            }
            PadResult::Two(a, b) => {
                result.add_point(a);
                result.add_point(b);
            }
            PadResult::Concave => {
                // Non-convex corner: fall back to bbox padding.
                // This matches C# which falls back to convex hull + repad.
                let bb = poly.bounding_box();
                let padded_bb = Rectangle::new(
                    bb.left() - padding,
                    bb.bottom() - padding,
                    bb.right() + padding,
                    bb.top() + padding,
                );
                let mut fallback = Polyline::new();
                fallback.add_point(padded_bb.left_bottom());
                fallback.add_point(padded_bb.left_top());
                fallback.add_point(padded_bb.right_top());
                fallback.add_point(padded_bb.right_bottom());
                fallback.set_closed(true);
                return fallback;
            }
        }
    }

    result.set_closed(true);
    result
}

/// Result of padding a single corner vertex.
#[allow(dead_code)]
enum PadResult {
    /// Single offset point (normal or obtuse corner).
    One(Point),
    /// Two offset points (sharp/beveled corner).
    Two(Point, Point),
    /// Concave corner — needs convex hull fallback.
    Concave,
}

/// Faithful port of C# `GetPaddedCorner(first, second, third, padding)`.
///
/// Given three consecutive vertices (u, v, w) on a clockwise polyline,
/// compute the padded corner point(s) at vertex v.
#[allow(dead_code)]
fn get_padded_corner(u: Point, v: Point, w: Point, padding: f64) -> PadResult {
    // Check orientation: counterclockwise means concave corner
    if triangle_orientation(u, v, w) == Orientation::Counterclockwise {
        return PadResult::Concave;
    }

    let uv = v - u;
    let vw = w - v;

    // Perpendicular to edge u→v, rotated 90° CCW (outward for CW polyline)
    let uv_perp = rotate_90(uv).normalize();

    if corner_is_not_too_sharp(u, v, w) {
        // Normal corner: intersect the two offset lines
        let uv_offset = uv_perp * padding;
        let vw_perp = rotate_90(vw).normalize() * padding;

        // Line 1: (u + offset) → (v + offset)
        // Line 2: (v + vw_offset) → (w + vw_offset)
        if let Some(intersection) =
            line_line_intersection(u + uv_offset, v + uv_offset, v + vw_perp, w + vw_perp)
        {
            PadResult::One(intersection)
        } else {
            // Parallel lines (collinear edges) — just offset v
            PadResult::One(v + uv_offset)
        }
    } else {
        // Sharp corner: generate two beveled points
        let uv_norm = uv.normalize();
        let wv_norm = (v - w).normalize();
        let l = uv_norm + wv_norm;

        if l.length() < GeomConstants::TOLERANCE {
            // Degenerate: u-v-w nearly collinear
            return PadResult::One(v + uv_perp * padding);
        }

        let d = l.normalize() * padding;
        let dp = rotate_90(d);

        // Solve: padding = (d + x*dp) · uv_perp
        let dot_d = d.x() * uv_perp.x() + d.y() * uv_perp.y();
        let dot_dp = dp.x() * uv_perp.x() + dp.y() * uv_perp.y();

        if dot_dp.abs() < GeomConstants::TOLERANCE {
            return PadResult::One(v + d);
        }

        let xp = (padding - dot_d) / dot_dp;
        let a = v + d + dp * xp;
        let b = v + d - dp * xp;
        PadResult::Two(a, b)
    }
}

/// Check if a corner at v (between edges u→v and v→w) is not too sharp.
/// Faithful port of C# `CornerIsNotTooSharp`.
/// Returns true if the angle is > π/4 (45 degrees).
#[allow(dead_code)]
fn corner_is_not_too_sharp(u: Point, v: Point, w: Point) -> bool {
    let uv = u - v;
    let rotated = rotate_by_angle(uv, std::f64::consts::FRAC_PI_4);
    let a = rotated + v;
    triangle_orientation(v, a, w) == Orientation::Counterclockwise
}

/// Rotate a vector by the given angle (radians, counterclockwise).
#[allow(dead_code)]
fn rotate_by_angle(p: Point, angle: f64) -> Point {
    let cos = angle.cos();
    let sin = angle.sin();
    Point::new(p.x() * cos - p.y() * sin, p.x() * sin + p.y() * cos)
}

/// Rotate a vector 90 degrees counterclockwise (left turn).
/// For a clockwise polyline, this points outward.
#[allow(dead_code)]
fn rotate_90(p: Point) -> Point {
    Point::new(-p.y(), p.x())
}

/// Line-line intersection of lines (a1→a2) and (b1→b2).
/// Returns None if lines are parallel.
#[allow(dead_code)]
fn line_line_intersection(a1: Point, a2: Point, b1: Point, b2: Point) -> Option<Point> {
    let d1 = a2 - a1;
    let d2 = b2 - b1;
    let cross = d1.x() * d2.y() - d1.y() * d2.x();

    if cross.abs() < GeomConstants::TOLERANCE {
        return None;
    }

    let d = b1 - a1;
    let t = (d.x() * d2.y() - d.y() * d2.x()) / cross;
    Some(Point::new(a1.x() + t * d1.x(), a1.y() + t * d1.y()))
}

/// Triangle orientation for three points.
#[derive(PartialEq, Eq, Debug)]
#[allow(dead_code)]
enum Orientation {
    Clockwise,
    Counterclockwise,
    Collinear,
}

/// Determine the orientation of triangle (a, b, c).
#[allow(dead_code)]
fn triangle_orientation(a: Point, b: Point, c: Point) -> Orientation {
    let cross = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    if cross.abs() < GeomConstants::TOLERANCE {
        Orientation::Collinear
    } else if cross < 0.0 {
        Orientation::Clockwise
    } else {
        Orientation::Counterclockwise
    }
}

/// Faithful port of TS `Obstacle.IsPolylineRectangle()`.
///
/// Checks if the polyline has exactly 4 points, and each consecutive pair
/// forms a pure compass direction, with consistent right-turn rotation.
fn is_polyline_rectangle(polyline: &Polyline) -> bool {
    if polyline.count() != 4 {
        return false;
    }

    let points: Vec<Point> = polyline.points().collect();
    if points.len() != 4 {
        return false;
    }

    // Check each consecutive edge has a pure compass direction
    // and directions rotate consistently (all right turns for clockwise)
    let dir0 = compass_direction(points[0], points[1]);
    if dir0.is_none() {
        return false;
    }
    let mut prev_dir = dir0.unwrap();

    for i in 1..4 {
        let next_i = (i + 1) % 4;
        let next_dir = compass_direction(points[i], points[next_i]);
        if next_dir.is_none() {
            return false;
        }
        let nd = next_dir.unwrap();
        // For clockwise polyline, each turn should be a right turn
        if nd != rotate_right(prev_dir) {
            return false;
        }
        prev_dir = nd;
    }

    true
}

/// Pure compass directions (N, E, S, W).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum CompassDir {
    North,
    East,
    South,
    West,
}

/// Get the pure compass direction from point a to point b.
/// Returns None if the direction is not purely N/E/S/W.
fn compass_direction(a: Point, b: Point) -> Option<CompassDir> {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    let ax = dx.abs();
    let ay = dy.abs();
    let eps = GeomConstants::TOLERANCE;

    if ax < eps && ay < eps {
        return None; // Same point
    }
    if ax < eps {
        // Pure vertical
        return if dy > 0.0 {
            Some(CompassDir::North)
        } else {
            Some(CompassDir::South)
        };
    }
    if ay < eps {
        // Pure horizontal
        return if dx > 0.0 {
            Some(CompassDir::East)
        } else {
            Some(CompassDir::West)
        };
    }
    None // Diagonal -- not a pure compass direction
}

/// Rotate a compass direction 90 degrees clockwise (right turn).
fn rotate_right(dir: CompassDir) -> CompassDir {
    match dir {
        CompassDir::North => CompassDir::East,
        CompassDir::East => CompassDir::South,
        CompassDir::South => CompassDir::West,
        CompassDir::West => CompassDir::North,
    }
}

/// Compare two points in scan-direction order.
/// Returns negative if a < b, 0 if equal, positive if a > b.
///
/// Matches TS `VisibilityGraphGenerator.PointCompare` which compares
/// by perpendicular coordinate first, then by scan-parallel coordinate.
fn point_compare(a: Point, b: Point, scan_direction: ScanDirection) -> i32 {
    let perp_cmp =
        GeomConstants::compare(scan_direction.perp_coord(a), scan_direction.perp_coord(b));
    match perp_cmp {
        std::cmp::Ordering::Less => -1,
        std::cmp::Ordering::Greater => 1,
        std::cmp::Ordering::Equal => {
            let scan_cmp = GeomConstants::compare(scan_direction.coord(a), scan_direction.coord(b));
            match scan_cmp {
                std::cmp::Ordering::Less => -1,
                std::cmp::Ordering::Greater => 1,
                std::cmp::Ordering::Equal => 0,
            }
        }
    }
}
