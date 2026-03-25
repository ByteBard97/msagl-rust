use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::geometry::polyline::Polyline;
use super::obstacle_side::{ObstacleSide, SideType};
use super::overlap_convex_hull::OverlapConvexHull;
use super::shape::Shape;

/// Internal obstacle representation with padded boundary.
#[derive(Clone, Debug)]
pub struct Obstacle {
    /// Index of this obstacle (for cross-referencing).
    pub index: usize,
    /// Ordinal for scanline tiebreaking.
    pub ordinal: usize,
    /// The original shape.
    pub shape: Shape,
    /// The padded bounding box.
    padded_bbox: Rectangle,
    /// The padded polyline (closed rectangle, 4 points).
    padded_polyline: Polyline,
    /// Padding distance.
    pub padding: f64,
    /// Whether the padded polyline is a rectangle (4 points, pure compass turns).
    /// Matches TS `Obstacle.IsRectangle`.
    is_rectangle: bool,
    /// Whether this obstacle is a sentinel boundary marker.
    is_sentinel: bool,
    /// Active low side (set during sweep).
    active_low_side: Option<ObstacleSide>,
    /// Active high side (set during sweep).
    active_high_side: Option<ObstacleSide>,
    /// Convex hull this obstacle belongs to, if overlapping with others.
    /// Matches TS `Obstacle.ConvexHull`.
    convex_hull: Option<OverlapConvexHull>,
    /// Clump of overlapping rectangular obstacles (indices).
    /// Matches TS `Obstacle.clump`.
    clump: Vec<usize>,
    /// Whether this obstacle overlaps a group corner.
    /// Matches TS `Obstacle.OverlapsGroupCorner`.
    pub overlaps_group_corner: bool,
}

impl Obstacle {
    /// Ordinal reserved for sentinel obstacles.
    pub const FIRST_SENTINEL_ORDINAL: usize = 1;
    /// Starting ordinal for non-sentinel obstacles.
    pub const FIRST_NON_SENTINEL_ORDINAL: usize = 10;

    pub fn from_shape(shape: &Shape, padding: f64, index: usize) -> Self {
        let bb = shape.bounding_box();
        let padded = Rectangle::new(
            bb.left() - padding,
            bb.bottom() - padding,
            bb.right() + padding,
            bb.top() + padding,
        );

        // Create closed polyline: 4 corners clockwise from bottom-left
        let mut poly = Polyline::new();
        poly.add_point(padded.left_bottom());
        poly.add_point(padded.left_top());
        poly.add_point(padded.right_top());
        poly.add_point(padded.right_bottom());
        poly.set_closed(true);

        Self {
            index,
            ordinal: Self::FIRST_NON_SENTINEL_ORDINAL + index,
            shape: shape.clone(),
            padded_bbox: padded,
            padded_polyline: poly,
            padding,
            is_rectangle: true, // Rectangle shapes produce rectangular padded polylines
            is_sentinel: false,
            active_low_side: None,
            active_high_side: None,
            convex_hull: None,
            clump: Vec::new(),
            overlaps_group_corner: false,
        }
    }

    /// Create a sentinel obstacle used as a boundary marker.
    pub fn create_sentinel(a: Point, b: Point, ordinal: usize) -> Self {
        let bbox = Rectangle::from_points(a, b);

        let mut poly = Polyline::new();
        poly.add_point(a);
        poly.add_point(b);

        // Use a dummy shape backed by the same bbox.
        let shape = Shape::rectangle(bbox.left(), bbox.bottom(), bbox.width(), bbox.height());

        Self {
            index: 0,
            ordinal,
            shape,
            padded_bbox: bbox,
            padded_polyline: poly,
            padding: 0.0,
            is_rectangle: false,
            is_sentinel: true,
            active_low_side: None,
            active_high_side: None,
            convex_hull: None,
            clump: Vec::new(),
            overlaps_group_corner: false,
        }
    }

    pub fn padded_bounding_box(&self) -> &Rectangle {
        &self.padded_bbox
    }

    pub fn padded_polyline(&self) -> &Polyline {
        &self.padded_polyline
    }

    /// Get the 4 padded corner points.
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

    /// Whether this obstacle is inside a convex hull.
    /// Matches TS `Obstacle.IsInConvexHull`.
    pub fn is_in_convex_hull(&self) -> bool {
        self.convex_hull.is_some()
    }

    /// Whether this obstacle is the primary representative in its convex hull.
    /// Only the primary obstacle participates in the visibility graph hierarchy.
    ///
    /// Matches TS `Obstacle.IsPrimaryObstacle`.
    pub fn is_primary_obstacle(&self) -> bool {
        match &self.convex_hull {
            None => true,
            Some(hull) => hull.primary_obstacle_index() == self.index,
        }
    }

    /// The polyline used for visibility graph generation.
    /// If in a convex hull, returns the hull polyline; otherwise the padded polyline.
    ///
    /// Matches TS `Obstacle.VisibilityPolyline`.
    pub fn visibility_polyline(&self) -> &Polyline {
        match &self.convex_hull {
            Some(hull) => &hull.polyline,
            None => &self.padded_polyline,
        }
    }

    /// The bounding box of the visibility polyline.
    /// Matches TS `Obstacle.VisibilityBoundingBox`.
    pub fn visibility_bounding_box(&self) -> Rectangle {
        self.visibility_polyline().bounding_box()
    }

    /// Whether this obstacle is overlapped (in a clump).
    /// Matches TS `Obstacle.isOverlapped`.
    pub fn is_overlapped(&self) -> bool {
        !self.clump.is_empty()
    }

    /// Whether this obstacle is in the same clump as another.
    /// Matches TS `Obstacle.IsInSameClump`.
    pub fn is_in_same_clump(&self, other: &Obstacle) -> bool {
        self.is_overlapped() && self.clump == other.clump
    }

    /// Get the clump indices this obstacle belongs to.
    pub fn clump(&self) -> &[usize] {
        &self.clump
    }

    /// Set the clump for this obstacle.
    pub fn set_clump(&mut self, clump: Vec<usize>) {
        self.clump = clump;
    }

    /// Get the convex hull this obstacle belongs to.
    pub fn convex_hull(&self) -> Option<&OverlapConvexHull> {
        self.convex_hull.as_ref()
    }

    /// Set the convex hull for this obstacle.
    ///
    /// Matches TS `Obstacle.SetConvexHull(hull)`.
    /// Clears any existing clump and marks as non-rectangular.
    pub fn set_convex_hull(&mut self, hull: OverlapConvexHull) {
        self.clump.clear();
        self.is_rectangle = false;
        self.convex_hull = Some(hull);
    }

    /// Clear the convex hull.
    pub fn clear_convex_hull(&mut self) {
        self.convex_hull = None;
    }

    pub fn active_low_side(&self) -> Option<&ObstacleSide> {
        self.active_low_side.as_ref()
    }

    pub fn active_high_side(&self) -> Option<&ObstacleSide> {
        self.active_high_side.as_ref()
    }

    /// Create the initial active sides for sweep-line processing.
    ///
    /// For a **horizontal scan** (sweeping vertically):
    /// - Low side = left side (low X): vertical line from bottom-left to top-left.
    /// - High side = right side (high X): vertical line from top-right to bottom-right.
    ///
    /// For a **vertical scan** (sweeping horizontally):
    /// - Low side = bottom side (low Y): horizontal line from left-bottom to right-bottom.
    /// - High side = top side (high Y): horizontal line from right-top to left-top.
    pub fn create_initial_sides(&mut self, is_horizontal_scan: bool) {
        let bb = &self.padded_bbox;
        if is_horizontal_scan {
            // Low = left (low X): bottom-left → top-left
            self.active_low_side = Some(ObstacleSide::new(
                SideType::Low,
                bb.left_bottom(),
                bb.left_top(),
                self.ordinal,
            ));
            // High = right (high X): top-right → bottom-right
            self.active_high_side = Some(ObstacleSide::new(
                SideType::High,
                bb.right_top(),
                bb.right_bottom(),
                self.ordinal,
            ));
        } else {
            // Low = bottom (low Y): left-bottom → right-bottom
            self.active_low_side = Some(ObstacleSide::new(
                SideType::Low,
                bb.left_bottom(),
                bb.right_bottom(),
                self.ordinal,
            ));
            // High = top (high Y): right-top → left-top
            self.active_high_side = Some(ObstacleSide::new(
                SideType::High,
                bb.right_top(),
                bb.left_top(),
                self.ordinal,
            ));
        }
    }
}