//! Arena-based linked list of path points.
//!
//! Ported from LinkedPoint.ts using Vec + index links instead of heap allocation.
//! Each LinkedPointIndex points to a node in the arena. Nodes form singly-linked
//! chains representing path segments.

use crate::geometry::point::Point;

/// Index into the LinkedPoint arena.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
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
    /// Returns the list and the index of the first node (head).
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
    /// Faithful port of LinkedPoint.SetNewNext().
    pub fn insert_after(&mut self, after: LinkedPointIndex, point: Point) -> LinkedPointIndex {
        let new_idx = self.add(point);
        let old_next = self.nodes[after.0].next;
        self.nodes[after.0].next = Some(new_idx);
        self.nodes[new_idx.0].next = old_next;
        new_idx
    }

    /// Insert points[i+1..j] after `after` in forward order.
    /// Faithful port of LinkedPoint.InsertVerts(i, j, points).
    ///
    /// The TS code iterates `for (j--; i < j; j--)` inserting points[j]
    /// via SetNewNext, which reverses the slice into forward linked order
    /// because each insert goes right after `self`.
    pub fn insert_verts(
        &mut self,
        after: LinkedPointIndex,
        i: usize,
        j: usize,
        points: &[Point],
    ) {
        // TS: for (j--; i < j; j--) { this.SetNewNext(points[j]) }
        // Each SetNewNext inserts right after `after`, pushing previous
        // insertions forward. So we iterate j-1 down to i+1.
        let mut jj = j;
        loop {
            if jj == 0 {
                break;
            }
            jj -= 1;
            if i >= jj {
                break;
            }
            self.insert_after(after, points[jj]);
        }
    }

    /// Insert points[i+1..j] after `after` in reverse order.
    /// Faithful port of LinkedPoint.InsertVertsInReverse(i, j, points).
    ///
    /// The TS code iterates `for (i++; i < j; i++)` inserting points[i]
    /// via SetNewNext, which puts them in reverse linked order.
    pub fn insert_verts_in_reverse(
        &mut self,
        after: LinkedPointIndex,
        i: usize,
        j: usize,
        points: &[Point],
    ) {
        // TS: for (i++; i < j; i++) { this.SetNewNext(points[i]) }
        let mut ii = i + 1;
        while ii < j {
            self.insert_after(after, points[ii]);
            ii += 1;
        }
    }

    /// Set the next pointer of a node directly.
    /// Used by PathMerger to splice linked list chains.
    pub fn set_next(&mut self, idx: LinkedPointIndex, next: Option<LinkedPointIndex>) {
        self.nodes[idx.0].next = next;
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

    /// Iterate over all node indices starting from `start`.
    pub fn iter_indices(&self, start: LinkedPointIndex) -> LinkedPointIter<'_> {
        LinkedPointIter {
            list: self,
            current: Some(start),
        }
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

/// Iterator over node indices in a linked chain.
pub struct LinkedPointIter<'a> {
    list: &'a LinkedPointList,
    current: Option<LinkedPointIndex>,
}

impl<'a> Iterator for LinkedPointIter<'a> {
    type Item = LinkedPointIndex;

    fn next(&mut self) -> Option<Self::Item> {
        let idx = self.current?;
        self.current = self.list.nodes[idx.0].next;
        Some(idx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pt(x: f64, y: f64) -> Point {
        Point::new(x, y)
    }

    #[test]
    fn from_points_creates_linked_chain() {
        let points = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)];
        let (list, head) = LinkedPointList::from_points(&points);
        let head = head.unwrap();
        let collected = list.collect_points(head);
        assert_eq!(collected.len(), 3);
        assert!(collected[0].close_to(pt(0.0, 0.0)));
        assert!(collected[1].close_to(pt(1.0, 0.0)));
        assert!(collected[2].close_to(pt(2.0, 0.0)));
    }

    #[test]
    fn insert_after_splices_correctly() {
        let points = vec![pt(0.0, 0.0), pt(2.0, 0.0)];
        let (mut list, head) = LinkedPointList::from_points(&points);
        let head = head.unwrap();
        list.insert_after(head, pt(1.0, 0.0));
        let collected = list.collect_points(head);
        assert_eq!(collected.len(), 3);
        assert!(collected[1].close_to(pt(1.0, 0.0)));
        assert!(collected[2].close_to(pt(2.0, 0.0)));
    }

    #[test]
    fn insert_verts_inserts_in_forward_order() {
        // Start: A -> D
        // points = [A, B, C, D], insert_verts(after=A, i=0, j=3, points)
        // Should insert points[2], points[1] via SetNewNext (reverse iteration)
        // Result: A -> B -> C -> D
        let (mut list, head) = LinkedPointList::from_points(&[pt(0.0, 0.0), pt(30.0, 0.0)]);
        let head = head.unwrap();
        let points = [pt(0.0, 0.0), pt(10.0, 0.0), pt(20.0, 0.0), pt(30.0, 0.0)];
        list.insert_verts(head, 0, 3, &points);
        let collected = list.collect_points(head);
        assert_eq!(collected.len(), 4);
        assert!(collected[1].close_to(pt(10.0, 0.0)));
        assert!(collected[2].close_to(pt(20.0, 0.0)));
    }

    #[test]
    fn insert_verts_in_reverse_inserts_reversed() {
        // Start: A -> D
        // points = [A, B, C, D], insert_verts_in_reverse(after=A, i=0, j=3, points)
        // TS: for (i++; i < j; i++) SetNewNext(points[i])
        // Inserts B then C via SetNewNext after A
        // Since SetNewNext always inserts right after `after`:
        //   after B: A -> B -> D
        //   after C: A -> C -> B -> D  (C inserted after A, pushing B forward)
        let (mut list, head) = LinkedPointList::from_points(&[pt(0.0, 0.0), pt(30.0, 0.0)]);
        let head = head.unwrap();
        let points = [pt(0.0, 0.0), pt(10.0, 0.0), pt(20.0, 0.0), pt(30.0, 0.0)];
        list.insert_verts_in_reverse(head, 0, 3, &points);
        let collected = list.collect_points(head);
        assert_eq!(collected.len(), 4);
        // Reversed order: C, B inserted after A via SetNewNext
        assert!(collected[1].close_to(pt(20.0, 0.0)));
        assert!(collected[2].close_to(pt(10.0, 0.0)));
    }

    #[test]
    fn set_next_changes_link() {
        let (mut list, head) = LinkedPointList::from_points(&[pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)]);
        let head = head.unwrap();
        let second = list.next(head).unwrap();
        let third = list.next(second).unwrap();
        // Skip second: head -> third
        list.set_next(head, Some(third));
        let collected = list.collect_points(head);
        assert_eq!(collected.len(), 2);
        assert!(collected[1].close_to(pt(2.0, 0.0)));
    }

    #[test]
    fn iter_indices_works() {
        let points = vec![pt(0.0, 0.0), pt(1.0, 0.0), pt(2.0, 0.0)];
        let (list, head) = LinkedPointList::from_points(&points);
        let head = head.unwrap();
        let indices: Vec<_> = list.iter_indices(head).collect();
        assert_eq!(indices.len(), 3);
    }

    #[test]
    fn empty_list() {
        let (list, head) = LinkedPointList::from_points(&[]);
        assert!(head.is_none());
        assert!(list.is_empty());
    }
}
