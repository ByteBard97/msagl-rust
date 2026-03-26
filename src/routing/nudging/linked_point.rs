use crate::geometry::point::Point;

/// Index into the LinkedPoint arena.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LinkedPointIndex(pub usize);

/// A node in an arena-based linked list of path points.
#[derive(Debug, Clone)]
pub struct LinkedPointNode {
    pub point: Point,
    pub next: Option<LinkedPointIndex>,
}

/// Arena-based linked list of path points for efficient insertion during refinement.
/// Ported from LinkedPoint.ts using Vec + index links instead of heap allocation.
pub struct LinkedPointList {
    nodes: Vec<LinkedPointNode>,
}

impl LinkedPointList {
    pub fn new() -> Self {
        Self { nodes: Vec::new() }
    }

    /// Add a new point, returning its index.
    pub fn add(&mut self, point: Point) -> LinkedPointIndex {
        let idx = LinkedPointIndex(self.nodes.len());
        self.nodes.push(LinkedPointNode { point, next: None });
        idx
    }

    /// Build a linked list from a slice of points (in order).
    pub fn from_points(points: &[Point]) -> (Self, Option<LinkedPointIndex>) {
        let mut list = Self::new();
        if points.is_empty() {
            return (list, None);
        }
        let first = list.add(points[0]);
        let mut prev = first;
        for &p in &points[1..] {
            let cur = list.add(p);
            list.nodes[prev.0].next = Some(cur);
            prev = cur;
        }
        (list, Some(first))
    }

    /// Insert a new point after the given node, returning the new node's index.
    pub fn insert_after(&mut self, after: LinkedPointIndex, point: Point) -> LinkedPointIndex {
        let new_idx = self.add(point);
        let old_next = self.nodes[after.0].next;
        self.nodes[after.0].next = Some(new_idx);
        self.nodes[new_idx.0].next = old_next;
        new_idx
    }

    pub fn node(&self, idx: LinkedPointIndex) -> &LinkedPointNode {
        &self.nodes[idx.0]
    }

    pub fn point(&self, idx: LinkedPointIndex) -> Point {
        self.nodes[idx.0].point
    }

    pub fn next(&self, idx: LinkedPointIndex) -> Option<LinkedPointIndex> {
        self.nodes[idx.0].next
    }

    /// Collect all points from the given start index into a Vec.
    pub fn collect_points(&self, start: LinkedPointIndex) -> Vec<Point> {
        let mut result = Vec::new();
        let mut cur = Some(start);
        while let Some(idx) = cur {
            result.push(self.nodes[idx.0].point);
            cur = self.nodes[idx.0].next;
        }
        result
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }
}

impl Default for LinkedPointList {
    fn default() -> Self {
        Self::new()
    }
}
