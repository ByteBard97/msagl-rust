//! StaircaseRemover: detects and collapses zig-zag patterns in nudged paths.
//!
//! A staircase is a pattern of 5 consecutive points where segments alternate
//! direction: a-b-c-d-f where dir(a,b)==dir(c,d) and dir(b,c)==dir(d,f).
//! The staircase can be collapsed by replacing the middle 3 points with 2.
//!
//! Uses an R-tree of path segments to detect cross-path crossings, matching
//! the C# StaircaseRemover which checks both obstacle intersection AND
//! whether collapsing a staircase would cause two paths' segments to overlap.

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use rstar::{RTree, RTreeObject, AABB};

/// A path segment identified by its path index and segment offset within that path.
/// Used as the R-tree element for cross-path crossing detection.
#[derive(Clone, Debug)]
struct SegWithIndex {
    start: Point,
    end: Point,
    path_index: usize,
    offset: usize,
}

impl SegWithIndex {
    fn new(pts: &[Point], offset: usize, path_index: usize) -> Self {
        Self {
            start: pts[offset],
            end: pts[offset + 1],
            path_index,
            offset,
        }
    }

    fn rect_min(&self) -> [f64; 2] {
        [
            self.start.x().min(self.end.x()),
            self.start.y().min(self.end.y()),
        ]
    }

    fn rect_max(&self) -> [f64; 2] {
        [
            self.start.x().max(self.end.x()),
            self.start.y().max(self.end.y()),
        ]
    }
}

/// rstar envelope wrapper for SegWithIndex.
#[derive(Clone, Debug)]
struct SegEnvelope {
    seg: SegWithIndex,
}

impl RTreeObject for SegEnvelope {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.seg.rect_min(), self.seg.rect_max())
    }
}

impl PartialEq for SegEnvelope {
    fn eq(&self, other: &Self) -> bool {
        self.seg.path_index == other.seg.path_index && self.seg.offset == other.seg.offset
    }
}

/// R-tree based staircase remover.
/// Matches C# StaircaseRemover: uses an R-tree of path segments to check
/// whether collapsing a staircase would cross another path's segment.
struct RTreeStaircaseRemover<'a> {
    paths: &'a mut [Vec<Point>],
    obstacles: &'a [Rectangle],
    seg_tree: RTree<SegEnvelope>,
    crossed_out: Vec<bool>,
}

impl<'a> RTreeStaircaseRemover<'a> {
    fn new(paths: &'a mut [Vec<Point>], obstacles: &'a [Rectangle]) -> Self {
        let crossed_out = vec![false; paths.len()];
        let seg_tree = RTree::new();
        let mut remover = Self {
            paths,
            obstacles,
            seg_tree,
            crossed_out,
        };
        remover.init_seg_tree();
        remover
    }

    /// C# InitHierarchies: insert all path segments into the R-tree.
    fn init_seg_tree(&mut self) {
        let mut envelopes = Vec::new();
        for (path_idx, path) in self.paths.iter().enumerate() {
            for seg_idx in 0..path.len().saturating_sub(1) {
                envelopes.push(SegEnvelope {
                    seg: SegWithIndex::new(path, seg_idx, path_idx),
                });
            }
        }
        self.seg_tree = RTree::bulk_load(envelopes);
    }

    /// C# Calculate: iterate until no more staircases can be removed.
    fn calculate(&mut self) {
        let mut success = true;
        while success {
            success = false;
            for path_idx in 0..self.paths.len() {
                if self.crossed_out[path_idx] {
                    continue;
                }
                if self.process_path(path_idx) {
                    success = true;
                }
            }
        }
    }

    /// C# ProcessPath: try to find and remove one staircase in the path.
    fn process_path(&mut self, path_idx: usize) -> bool {
        let pts = self.paths[path_idx].clone();
        let mut can_have_staircase = false;
        let staircase_start = self.find_staircase_start(&pts, path_idx, &mut can_have_staircase);
        if let Some(start) = staircase_start {
            let new_pts = self.remove_staircase(&pts, start, path_idx);
            self.paths[path_idx] = new_pts;
            return true;
        }
        if !can_have_staircase {
            self.crossed_out[path_idx] = true;
        }
        false
    }

    /// C# FindStaircaseStart: find the first staircase pattern in the path.
    /// Uses a sliding window of 4 segments, matching the C# circular buffer approach.
    fn find_staircase_start(
        &self,
        pts: &[Point],
        path_idx: usize,
        can_have_staircase: &mut bool,
    ) -> Option<usize> {
        *can_have_staircase = false;
        if pts.len() < 5 {
            return None;
        }

        // Build initial 4 segments for the sliding window (C# segs array).
        let mut segs = [
            SegWithIndex::new(pts, 0, path_idx),
            SegWithIndex::new(pts, 1, path_idx),
            SegWithIndex::new(pts, 2, path_idx),
            SegWithIndex::new(pts, 3, path_idx),
        ];
        let mut seg_to_replace = 0usize;

        let mut i = 0;
        loop {
            let mut can_have_at_i = false;
            if self.is_staircase(pts, i, &segs, &mut can_have_at_i) {
                *can_have_staircase = true;
                return Some(i);
            }
            *can_have_staircase = *can_have_staircase || can_have_at_i;
            i += 1;
            if pts.len() < i + 5 {
                return None;
            }
            // Slide window: replace oldest segment with new one at i+3.
            segs[seg_to_replace] = SegWithIndex::new(pts, i + 3, path_idx);
            seg_to_replace = (seg_to_replace + 1) % 4;
        }
    }

    /// C# IsStaircase: check if pts[offset..offset+5] form a staircase that
    /// can be collapsed without crossing obstacles or other paths' segments.
    fn is_staircase(
        &self,
        pts: &[Point],
        offset: usize,
        segs_to_ignore: &[SegWithIndex; 4],
        can_have_staircase_at_i: &mut bool,
    ) -> bool {
        let a = pts[offset];
        let b = pts[offset + 1];
        let c = pts[offset + 2];
        let d = pts[offset + 3];
        let f = pts[offset + 4];

        *can_have_staircase_at_i = false;

        // Check alternating directions: dir(a,b)==dir(c,d) and dir(b,c)==dir(d,f).
        if seg_direction(a, b) != seg_direction(c, d)
            || seg_direction(b, c) != seg_direction(d, f)
        {
            return false;
        }

        // Compute the flipped point.
        let flipped = get_flipped_point(pts, offset);

        // Check obstacle intersection for the two new segments (b->flipped, flipped->d).
        if self.intersect_obstacle_hierarchy(b, flipped, d) {
            return false;
        }

        *can_have_staircase_at_i = true;

        // Check cross-path segment crossing via R-tree.
        !self.crossing(b, flipped, segs_to_ignore)
    }

    /// C# Crossing: check if segment a->b crosses any path segment in the R-tree,
    /// ignoring the segments in segs_to_ignore (which belong to the current staircase).
    fn crossing(&self, a: Point, b: Point, segs_to_ignore: &[SegWithIndex; 4]) -> bool {
        let seg_rect = Rectangle::from_points(a, b);
        let aabb = AABB::from_corners(
            [seg_rect.left(), seg_rect.bottom()],
            [seg_rect.right(), seg_rect.top()],
        );

        for env in self.seg_tree.locate_in_envelope_intersecting(&aabb) {
            // Skip segments that are part of the current staircase window.
            let dominated = segs_to_ignore.iter().any(|s| {
                s.path_index == env.seg.path_index && s.offset == env.seg.offset
            });
            if !dominated {
                return true;
            }
        }
        false
    }

    /// C# IntersectObstacleHierarchy(a, b, c): check if segments a->b or b->c
    /// cross any obstacle interior.
    fn intersect_obstacle_hierarchy(&self, a: Point, b: Point, c: Point) -> bool {
        segments_intersect_obstacles(a, b, c, self.obstacles)
    }

    /// C# RemoveStaircase: collapse the staircase and update the R-tree.
    fn remove_staircase(&mut self, pts: &[Point], staircase_start: usize, path_idx: usize) -> Vec<Point> {
        let a = pts[staircase_start];
        let b = pts[staircase_start + 1];
        let horiz = (a.y() - b.y()).abs() < GeomConstants::DISTANCE_EPSILON / 2.0;

        // Remove all segments of this path from the R-tree (C# RemoveSegs).
        self.remove_path_segs(pts, path_idx);

        // Build the new points array: remove 2 points, insert flipped point.
        // C#: ret = new Point[pts.Length - 2]
        //     Copy first staircaseStart+1 points
        //     Set ret[staircaseStart+1] = flipped
        //     Copy remaining from staircaseStart+4 onward
        let mut ret = Vec::with_capacity(pts.len() - 2);
        // Copy pts[0..=staircase_start] (first staircase_start+1 points).
        ret.extend_from_slice(&pts[..staircase_start + 1]);

        // Compute the flipped point.
        let seg_a = pts[staircase_start + 1];
        let seg_c = pts[staircase_start + 3];
        let flipped = if horiz {
            Point::new(seg_c.x(), seg_a.y())
        } else {
            Point::new(seg_a.x(), seg_c.y())
        };
        ret.push(flipped);

        // Copy pts[staircase_start+4..] (rest of the path).
        ret.extend_from_slice(&pts[staircase_start + 4..]);

        // Insert new segments around the staircase into the R-tree (C# InsertNewSegs).
        // C# inserts segs at staircase_start and staircase_start+1.
        self.insert_new_segs(&ret, staircase_start, path_idx);

        ret
    }

    /// C# RemoveSegs: remove all segments of a path from the R-tree.
    fn remove_path_segs(&mut self, pts: &[Point], path_idx: usize) {
        for i in 0..pts.len().saturating_sub(1) {
            let env = SegEnvelope {
                seg: SegWithIndex::new(pts, i, path_idx),
            };
            self.seg_tree.remove(&env);
        }
    }

    /// C# InsertNewSegs: insert the two new segments around the collapsed staircase.
    fn insert_new_segs(&mut self, pts: &[Point], staircase_start: usize, path_idx: usize) {
        if staircase_start < pts.len().saturating_sub(1) {
            self.seg_tree.insert(SegEnvelope {
                seg: SegWithIndex::new(pts, staircase_start, path_idx),
            });
        }
        if staircase_start + 1 < pts.len().saturating_sub(1) {
            self.seg_tree.insert(SegEnvelope {
                seg: SegWithIndex::new(pts, staircase_start + 1, path_idx),
            });
        }
    }
}

/// Remove staircase patterns from all paths.
///
/// Uses an R-tree of path segments to avoid collapsing staircases that would
/// cause cross-path segment crossings. Iterates until no more staircases
/// can be removed.
pub fn remove_staircases(paths: &mut [Vec<Point>], obstacles: &[Rectangle]) {
    let mut remover = RTreeStaircaseRemover::new(paths, obstacles);
    remover.calculate();
}

/// Get the flipped point for staircase collapse.
fn get_flipped_point(pts: &[Point], offset: usize) -> Point {
    let horiz = GeomConstants::close(pts[offset].y(), pts[offset + 1].y());
    if horiz {
        Point::new(pts[offset + 4].x(), pts[offset].y())
    } else {
        Point::new(pts[offset].x(), pts[offset + 4].y())
    }
}

/// Check if segments (a-b) and (b-c) intersect any obstacle.
fn segments_intersect_obstacles(a: Point, b: Point, c: Point, obstacles: &[Rectangle]) -> bool {
    let seg1_rect = Rectangle::from_points(a, b);
    let seg2_rect = Rectangle::from_points(b, c);

    for obs in obstacles {
        if obs.intersects(&seg1_rect) || obs.intersects(&seg2_rect) {
            // More precise check: the segment must actually cross the obstacle interior.
            if segment_crosses_obstacle(a, b, obs) || segment_crosses_obstacle(b, c, obs) {
                return true;
            }
        }
    }
    false
}

/// Check if a rectilinear segment strictly crosses an obstacle's interior.
fn segment_crosses_obstacle(p0: Point, p1: Point, obs: &Rectangle) -> bool {
    let eps = GeomConstants::DISTANCE_EPSILON;
    if GeomConstants::close(p0.y(), p1.y()) {
        // Horizontal segment.
        let y = p0.y();
        if y <= obs.bottom() + eps || y >= obs.top() - eps {
            return false;
        }
        let x_min = p0.x().min(p1.x());
        let x_max = p0.x().max(p1.x());
        x_min < obs.right() - eps && x_max > obs.left() + eps
    } else if GeomConstants::close(p0.x(), p1.x()) {
        // Vertical segment.
        let x = p0.x();
        if x <= obs.left() + eps || x >= obs.right() - eps {
            return false;
        }
        let y_min = p0.y().min(p1.y());
        let y_max = p0.y().max(p1.y());
        y_min < obs.top() - eps && y_max > obs.bottom() + eps
    } else {
        false
    }
}

/// Get the direction character of a segment (simplified compass direction).
fn seg_direction(a: Point, b: Point) -> u8 {
    let dx = b.x() - a.x();
    let dy = b.y() - a.y();
    if dx.abs() > dy.abs() {
        if dx > 0.0 { b'E' } else { b'W' }
    } else if dy > 0.0 {
        b'N'
    } else {
        b'S'
    }
}

// =========================================================================
// R-tree based StaircaseRemover — C# StaircaseRemover.cs (200 lines)
//
// The existing implementation uses brute-force obstacle checks (O(N) per
// staircase test). The C# uses an R-tree of path segments (segTree) for
// O(log S) cross-path collision detection, plus the obstacle R-tree.
//
// The struct below implements the full C# StaircaseRemover with:
// - rstar::RTree<SegWithIndex> for cross-path segment collision
// - rstar::RTree for obstacle hierarchy
// - Crossed-out path tracking to skip paths with no more staircases
// =========================================================================

use rstar::{RTree, RTreeObject, AABB};
use std::collections::HashSet;

/// A segment reference: indexes into a path's point array.
///
/// C# file: SegWithIndex.cs, lines 7-25
/// Stores a reference to a points array and an offset index.
/// In Rust, we store path_index + point_offset for identity.
#[derive(Clone, Debug)]
pub struct SegWithIndex {
    /// Index of the path in the paths array.
    pub path_index: usize,
    /// Offset within the path's point array (segment is pts[offset]..pts[offset+1]).
    pub point_offset: usize,
    /// Cached start point.
    pub start: Point,
    /// Cached end point.
    pub end: Point,
}

impl SegWithIndex {
    pub fn new(path_index: usize, point_offset: usize, start: Point, end: Point) -> Self {
        Self {
            path_index,
            point_offset,
            start,
            end,
        }
    }

    /// Bounding box for R-tree insertion.
    pub fn bounding_rect(&self) -> Rectangle {
        Rectangle::from_points(self.start, self.end)
    }
}

impl PartialEq for SegWithIndex {
    fn eq(&self, other: &Self) -> bool {
        self.path_index == other.path_index && self.point_offset == other.point_offset
    }
}
impl Eq for SegWithIndex {}

/// R-tree envelope wrapper for SegWithIndex.
#[derive(Clone, Debug)]
struct SegEnvelope {
    seg: SegWithIndex,
    min: [f64; 2],
    max: [f64; 2],
}

impl RTreeObject for SegEnvelope {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.min, self.max)
    }
}

/// Full StaircaseRemover with R-tree based cross-path collision detection.
///
/// C# file: StaircaseRemover.cs, lines 9-199
/// Uses an R-tree of path segments for O(log S) crossing detection instead
/// of brute-force O(N) obstacle scanning.
pub struct RTreeStaircaseRemover {
    /// R-tree of all path segments for cross-path crossing detection.
    /// C# StaircaseRemover.cs line 13: segTree
    /// MUST use rstar::RTree for O(log S) spatial query
    seg_tree: RTree<SegEnvelope>,

    /// Set of path indices that have been determined to have no more staircases.
    /// C# StaircaseRemover.cs line 14: crossedOutPaths
    /// MUST use HashSet for O(1) membership testing
    crossed_out_paths: HashSet<usize>,
}

impl RTreeStaircaseRemover {
    /// Create a new R-tree staircase remover.
    ///
    /// C# file: StaircaseRemover.cs, lines 16-18
    pub fn new() -> Self {
        Self {
            seg_tree: RTree::new(),
            crossed_out_paths: HashSet::new(),
        }
    }

    /// Remove staircases from all paths using R-tree collision detection.
    ///
    /// C# file: StaircaseRemover.cs, lines 22-25 (static entry point)
    /// Big-O: O(P * S * log S) where P = paths, S = total segments
    /// MUST use R-tree for cross-path crossing detection
    pub fn remove_staircases_rtree(
        paths: &mut [Vec<Point>],
        obstacles: &[Rectangle],
    ) {
        let mut remover = Self::new();
        remover.calculate(paths, obstacles);
    }

    /// Main calculation loop: init hierarchies, iterate until no more changes.
    ///
    /// C# file: StaircaseRemover.cs, lines 27-35
    /// Big-O: O(iterations * P * S * log S)
    fn calculate(
        &mut self,
        _paths: &mut [Vec<Point>],
        _obstacles: &[Rectangle],
    ) {
        // Steps:
        // 1. Call init_hierarchies to populate seg_tree (C# line 28)
        // 2. Loop: for each non-crossed-out path, try process_path (C# lines 30-34)
        // 3. Repeat until no success
        todo!()
    }

    /// Initialize the segment R-tree with all path segments.
    ///
    /// C# file: StaircaseRemover.cs, lines 176-183
    /// Big-O: O(S log S) for R-tree bulk insertion
    /// MUST use rstar::RTree for spatial indexing of segments
    fn init_hierarchies(&mut self, _paths: &[Vec<Point>]) {
        // For each path, insert all segments (pts[i]..pts[i+1]) into seg_tree
        todo!()
    }

    /// Process a single path: find and remove one staircase.
    ///
    /// C# file: StaircaseRemover.cs, lines 37-47
    /// Big-O: O(S * log S) per path for staircase detection
    fn process_path(
        &mut self,
        _path_index: usize,
        _path: &mut Vec<Point>,
        _obstacles: &[Rectangle],
    ) -> bool {
        // Steps:
        // 1. Call find_staircase_start (C# line 40)
        // 2. If found, call remove_staircase (C# line 42), return true
        // 3. If no staircase possible, add to crossed_out_paths (C# line 45)
        todo!()
    }

    /// Find the start index of a staircase pattern using the R-tree.
    ///
    /// C# file: StaircaseRemover.cs, lines 57-81
    /// Big-O: O(S * log S) for sliding window with R-tree queries
    /// MUST maintain a 4-segment sliding window for segs-to-ignore
    fn find_staircase_start(
        &self,
        _path_index: usize,
        _pts: &[Point],
        _obstacles: &[Rectangle],
    ) -> (i32, bool) {
        // Returns (staircase_start_index, can_have_staircase)
        // -1 if no staircase found
        // Steps:
        // 1. Check pts.len() >= 5 (C# line 59)
        // 2. Create sliding window of 4 SegWithIndex (C# lines 61-64)
        // 3. For each offset, call is_staircase_rtree (C# line 69)
        // 4. If found, return offset; else slide window (C# lines 78-80)
        todo!()
    }

    /// Check if a 5-point window is a staircase, using R-tree for crossing detection.
    ///
    /// C# file: StaircaseRemover.cs, lines 123-139
    /// Big-O: O(log S) for R-tree crossing query
    /// MUST use R-tree query (not brute-force scan) for cross-path collision
    fn is_staircase_rtree(
        &self,
        _pts: &[Point],
        _offset: usize,
        _segs_to_ignore: &[SegWithIndex],
        _obstacles: &[Rectangle],
    ) -> (bool, bool) {
        // Returns (is_staircase, can_have_staircase_at_i)
        // Steps (C# lines 124-138):
        // 1. Check alternating directions (C# lines 130-131)
        // 2. Compute flipped point (C# line 134)
        // 3. Check obstacle intersection (C# line 135) — use obstacle R-tree
        // 4. Check cross-path crossing via seg_tree R-tree query (C# line 138)
        //    MUST use rstar locate_in_envelope_intersecting
        todo!()
    }

    /// Check for crossing with other path segments using the R-tree.
    ///
    /// C# file: StaircaseRemover.cs, lines 96-109 (Crossing + IsCrossing)
    /// Big-O: O(log S + K) where K = candidate segments in bounding box
    /// MUST use rstar R-tree locate_in_envelope_intersecting query
    fn crossing(
        &self,
        _a: Point,
        _b: Point,
        _segs_to_ignore: &[SegWithIndex],
    ) -> bool {
        // Steps (C# lines 107-108):
        // 1. Create bounding box from segment (a, b)
        // 2. Query seg_tree R-tree for all segments intersecting the bbox
        // 3. Filter out segs_to_ignore
        // 4. Return true if any remaining
        todo!()
    }

    /// Remove a staircase and update the segment R-tree.
    ///
    /// C# file: StaircaseRemover.cs, lines 141-158
    /// Big-O: O(S log S) for removing old segments and inserting new ones
    /// MUST remove old segments from R-tree before modifying path
    /// MUST insert new segments after modifying path
    fn remove_staircase_rtree(
        &mut self,
        _path_index: usize,
        _pts: &mut Vec<Point>,
        _staircase_start: usize,
    ) {
        // Steps (C# lines 149-158):
        // 1. Remove all old segments from seg_tree (C# line 150: RemoveSegs)
        // 2. Modify the points array (collapse staircase) (C# lines 151-156)
        // 3. Insert new segments around the modified area (C# line 157: InsertNewSegs)
        todo!()
    }

    /// Insert a single path segment into the R-tree.
    ///
    /// C# file: StaircaseRemover.cs, lines 191-194
    /// Big-O: O(log S) for R-tree insertion
    /// MUST use rstar R-tree insert
    fn insert_seg(
        &mut self,
        _path_index: usize,
        _point_offset: usize,
        _pts: &[Point],
    ) {
        todo!()
    }

    /// Remove a path segment from the R-tree.
    ///
    /// C# file: StaircaseRemover.cs, lines 166-168
    /// Big-O: O(log S) for R-tree removal
    fn remove_seg(&mut self, _seg: &SegWithIndex) {
        todo!()
    }

    /// Check if segments (a-b) or (b-c) intersect any obstacle using the obstacle R-tree.
    ///
    /// C# file: StaircaseRemover.cs, lines 112-121
    /// Big-O: O(log N) for R-tree query where N = obstacles
    /// MUST use obstacle R-tree for intersection test
    fn intersect_obstacle_hierarchy(
        &self,
        _a: Point,
        _b: Point,
        _c: Point,
        _obstacles: &[Rectangle],
    ) -> bool {
        todo!()
    }
}
