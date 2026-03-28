use super::scan_direction::ScanDirection;
use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::polyline::Polyline;

/// Whether this is a low (bottom/left) or high (top/right) side of an obstacle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SideType {
    Low,
    High,
}

/// A side of an obstacle boundary, with slope tracking for scanline intersection.
///
/// Faithful port of BasicObstacleSide.ts + LowObstacleSide/HighObstacleSide.
///
/// In the TS:
/// - LowObstacleSide traverses clockwise for H-scan (counter-clockwise for V-scan)
/// - HighObstacleSide traverses counter-clockwise for H-scan (clockwise for V-scan)
///
/// Slope is computed relative to the scan direction:
///   slope = (change in scan-parallel coord) / (change in perp coord)
///   slope_inverse = 1 / slope
/// Both are 0 when the side is perpendicular to the scan.
#[derive(Debug, Clone)]
pub struct ObstacleSide {
    side_type: SideType,
    /// Raw 0-based index into the obstacles Vec (for array lookups).
    obstacle_index: usize,
    /// Scan-line ordinal (FIRST_NON_SENTINEL_ORDINAL + index for real obstacles,
    /// FIRST_SENTINEL_ORDINAL / +1 for sentinels). Used for BTreeMap key ordering
    /// and same-obstacle detection. Kept separate from obstacle_index to avoid
    /// collisions between obstacle index 1 and sentinel ordinal 1.
    ordinal: usize,
    start: Point,
    end: Point,
    start_key: PolylinePointKey,
    end_key: PolylinePointKey,
    slope: f64,
    slope_inverse: f64,
}

impl ObstacleSide {
    /// Create from a polyline point, traversing in the correct direction.
    ///
    /// Matches TS: `new LowObstacleSide(obstacle, startVertex, scanDir)` and
    /// `new HighObstacleSide(obstacle, startVertex, scanDir)`.
    ///
    /// Traversal direction:
    /// - Low + H-scan: clockwise (next) -- `traverseClockwise = scanDir.IsHorizontal`
    /// - Low + V-scan: counter-clockwise (prev)
    /// - High + H-scan: counter-clockwise (prev) -- `traverseClockwise = scanDir.IsVertical`
    /// - High + V-scan: clockwise (next)
    pub fn from_polyline_point(
        side_type: SideType,
        obstacle_index: usize,
        ordinal: usize,
        start_key: PolylinePointKey,
        polyline: &Polyline,
        scan_direction: ScanDirection,
    ) -> Self {
        let traverse_clockwise = match side_type {
            SideType::Low => scan_direction.is_horizontal(),
            SideType::High => scan_direction.is_vertical(),
        };

        let end_key = if traverse_clockwise {
            polyline
                .next_key(start_key)
                .expect("polyline must be closed or have a next point")
        } else {
            polyline
                .prev_key(start_key)
                .expect("polyline must be closed or have a prev point")
        };

        let start = polyline.point_at(start_key);
        let end = polyline.point_at(end_key);

        // Compute slope relative to scan direction.
        // TS: StaticGraphUtility.Slope(start, end, scanDir)
        //   = (scanDir.Coord(end) - scanDir.Coord(start)) /
        //     (scanDir.PerpCoord(end) - scanDir.PerpCoord(start))
        let (slope, slope_inverse) = if scan_direction.is_perpendicular(start, end) {
            // Perpendicular side -- slope is 0 (TS sets both to 0)
            (0.0, 0.0)
        } else {
            let d_scan = scan_direction.coord(end) - scan_direction.coord(start);
            let d_perp = scan_direction.perp_coord(end) - scan_direction.perp_coord(start);
            let s = d_scan / d_perp;
            (s, 1.0 / s)
        };

        Self {
            side_type,
            obstacle_index,
            ordinal,
            start,
            end,
            start_key,
            end_key,
            slope,
            slope_inverse,
        }
    }

    /// Create a sentinel side (no polyline traversal needed).
    /// Used for boundary sentinels that are simple line segments.
    pub fn sentinel(
        side_type: SideType,
        start: Point,
        end: Point,
        obstacle_ordinal: usize,
    ) -> Self {
        Self {
            side_type,
            obstacle_index: obstacle_ordinal,
            ordinal: obstacle_ordinal,
            start,
            end,
            start_key: PolylinePointKey::default(),
            end_key: PolylinePointKey::default(),
            slope: 0.0,
            slope_inverse: 0.0,
        }
    }

    pub fn side_type(&self) -> SideType {
        self.side_type
    }

    pub fn set_side_type(&mut self, t: SideType) {
        self.side_type = t;
    }
    pub fn start(&self) -> Point {
        self.start
    }
    pub fn end(&self) -> Point {
        self.end
    }
    pub fn obstacle_index(&self) -> usize {
        self.obstacle_index
    }
    pub fn start_vertex_key(&self) -> PolylinePointKey {
        self.start_key
    }
    pub fn end_vertex_key(&self) -> PolylinePointKey {
        self.end_key
    }
    pub fn slope(&self) -> f64 {
        self.slope
    }
    pub fn slope_inverse(&self) -> f64 {
        self.slope_inverse
    }

    /// Returns the scan-line ordinal (FIRST_NON_SENTINEL_ORDINAL + index for real obstacles).
    /// Used for BTreeMap key ordering and same-obstacle detection. Distinct from obstacle_index
    /// to avoid collisions between obstacle raw indices and sentinel ordinals.
    pub fn obstacle_ordinal(&self) -> usize {
        self.ordinal
    }

    /// The direction vector from start to end.
    /// Matches TS: `side.Direction` used in `ScanLineIntersectSidePBS`.
    pub fn direction(&self) -> Point {
        self.end - self.start
    }

    /// Compute the scanline intersection point at a given site.
    ///
    /// Faithful port of TS `VisibilityGraphGenerator.ScanLineIntersectSidePBS()`.
    /// Projects along the slope of the side to find where the scanline at `site`
    /// intersects this side.
    pub fn scanline_intersect(&self, site: Point, scan_direction: ScanDirection) -> Point {
        let dir = self.direction();
        let mut ix = self.start.x();
        let mut iy = self.start.y();

        if scan_direction.is_horizontal() {
            if dir.y().abs() > 1e-20 {
                ix += (dir.x() / dir.y()) * (site.y() - self.start.y());
            }
            // MungeIntersect: clamp ix to be within [start.x, end.x] range
            ix = munge_intersect(site.x(), ix, self.start.x(), self.end.x());
            iy = site.y();
        } else {
            ix = site.x();
            if dir.x().abs() > 1e-20 {
                iy += (dir.y() / dir.x()) * (site.x() - self.start.x());
            }
            iy = munge_intersect(site.y(), iy, self.start.y(), self.end.y());
        }

        Point::new(ix, iy)
    }

    /// Legacy constructor for tests and sentinel creation.
    /// Computes slope from raw dx/dy (not scan-relative).
    /// Prefer `from_polyline_point` or `sentinel` for new code.
    pub fn new(side_type: SideType, start: Point, end: Point, obstacle_ordinal: usize) -> Self {
        let dx = end.x() - start.x();
        let dy = end.y() - start.y();
        let slope = if dx.abs() < 1e-10 {
            f64::INFINITY
        } else {
            dy / dx
        };
        let slope_inverse = if dy.abs() < 1e-10 {
            f64::INFINITY
        } else {
            dx / dy
        };
        Self {
            side_type,
            start,
            end,
            obstacle_index: obstacle_ordinal,
            ordinal: obstacle_ordinal,
            start_key: PolylinePointKey::default(),
            end_key: PolylinePointKey::default(),
            slope,
            slope_inverse,
        }
    }

    /// Legacy scanline intersection (non-scan-direction-aware).
    /// Kept for backward compatibility with existing tests.
    pub fn scanline_intersection(&self, perp_coord: f64, is_horizontal_scan: bool) -> f64 {
        if is_horizontal_scan {
            if self.slope.is_infinite() || self.slope.abs() < 1e-10 {
                self.start.x()
            } else {
                self.start.x() + (perp_coord - self.start.y()) * self.slope_inverse
            }
        } else if self.slope_inverse.is_infinite() || self.slope_inverse.abs() < 1e-10 {
            self.start.y()
        } else {
            self.start.y() + (perp_coord - self.start.x()) * self.slope
        }
    }
}

/// Clamp the intersection coordinate to be within the side's range.
///
/// Faithful port of TS `SpliceUtility.MungeIntersect()`.
/// If `site_coord` is between `start` and `end`, prefer `intersect`;
/// otherwise clamp to the nearest endpoint.
fn munge_intersect(_site_coord: f64, intersect: f64, start: f64, end: f64) -> f64 {
    let lo = start.min(end);
    let hi = start.max(end);
    if intersect < lo {
        lo
    } else if intersect > hi {
        hi
    } else {
        intersect
    }
}
