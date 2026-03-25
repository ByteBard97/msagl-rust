//! Sweep-line H/V crossing detector for LinkedPointLists.
//!
//! Ported from LinkedPointSplitter.ts (152 lines).
//! Finds all intersections between horizontal and vertical path segments
//! and inserts crossing points into both lists.

use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::collections::BTreeMap;

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use super::linked_point::{LinkedPointIndex, LinkedPointList};

// ---------------------------------------------------------------------------
// Event types
// ---------------------------------------------------------------------------

/// Whether an event is a vertical-segment open/close or a horizontal query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EventKind {
    /// Vertical segment low endpoint — insert into active tree.
    VertLow,
    /// Vertical segment high endpoint — remove from active tree.
    VertHigh,
    /// Horizontal segment — intersect with active vertical segments.
    Horizontal,
}

/// A sweep-line event.
/// `priority` is the Y coordinate at which the event fires.
/// Vertical low events fire before horizontal events at the same Y
/// (so we insert the vertical segment before a horizontal at the same Y queries it).
/// Vertical high events fire last (after horizontals) so a segment is still
/// active when a horizontal at the same Y intersects it.
#[derive(Debug, Clone)]
struct Event {
    /// Sweep Y coordinate (negated for min-heap storage).
    y: OrderedFloat<f64>,
    /// Tie-break: lower kind value fires first.
    kind_ord: u8,
    kind: EventKind,
    /// Index of the start node of the segment.
    start: LinkedPointIndex,
    /// Which list owns this segment (unused after queue construction, kept for readability).
    #[allow(dead_code)]
    is_vertical: bool,
}

impl PartialEq for Event {
    fn eq(&self, other: &Self) -> bool {
        self.y == other.y && self.kind_ord == other.kind_ord
    }
}
impl Eq for Event {}

impl PartialOrd for Event {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Event {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        // Min-heap: smallest Y fires first; then smallest kind_ord.
        // BinaryHeap is a max-heap so we negate: largest negated-Y = smallest Y.
        self.y
            .cmp(&other.y)
            .then(self.kind_ord.cmp(&other.kind_ord))
    }
}

// ---------------------------------------------------------------------------
// Active-segment key
// ---------------------------------------------------------------------------

/// Key for the BTreeMap of active vertical segments, ordered by X.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct ActiveKey {
    x: OrderedFloat<f64>,
    /// Break ties by node index so distinct segments at the same X coexist.
    idx: usize,
}

// ---------------------------------------------------------------------------
// LinkedPointSplitter
// ---------------------------------------------------------------------------

/// Finds all H/V crossings between path segments and inserts intersection
/// points into both `LinkedPointList`s.
///
/// Ported from `LinkedPointSplitter.ts` using a sweep-line with a priority
/// queue and an active-segment BTreeMap instead of RBTree.
pub struct LinkedPointSplitter;

impl LinkedPointSplitter {
    /// Find all crossings between horizontal and vertical segments,
    /// inserting intersection points into both lists.
    ///
    /// `h_starts` — start indices of horizontal segments in `h_list`.
    /// `v_starts` — start indices of vertical segments in `v_list`.
    pub fn split(
        h_list: &mut LinkedPointList,
        h_starts: &[LinkedPointIndex],
        v_list: &mut LinkedPointList,
        v_starts: &[LinkedPointIndex],
    ) {
        if v_starts.is_empty() || h_starts.is_empty() {
            return;
        }

        // Build priority-queue (min-heap via Reverse).
        // BinaryHeap is a max-heap; wrapping in Reverse gives min-heap.
        let mut queue: BinaryHeap<Reverse<Event>> = BinaryHeap::new();

        // Enqueue vertical segments: low endpoint fires before high endpoint.
        for &start in v_starts {
            let p = v_list.point(start);
            let next_idx = match v_list.next(start) {
                Some(idx) => idx,
                None => continue,
            };
            let next_p = v_list.point(next_idx);
            let low_y = p.y().min(next_p.y());
            let high_y = p.y().max(next_p.y());

            queue.push(Reverse(Event {
                y: OrderedFloat(low_y),
                kind_ord: 0, // fires before horizontal at same Y
                kind: EventKind::VertLow,
                start,
                is_vertical: true,
            }));
            queue.push(Reverse(Event {
                y: OrderedFloat(high_y),
                kind_ord: 2, // fires after horizontal at same Y
                kind: EventKind::VertHigh,
                start,
                is_vertical: true,
            }));
        }

        // Enqueue horizontal segments at their Y coordinate.
        for &start in h_starts {
            let p = h_list.point(start);
            queue.push(Reverse(Event {
                y: OrderedFloat(p.y()),
                kind_ord: 1,
                kind: EventKind::Horizontal,
                start,
                is_vertical: false,
            }));
        }

        // Active vertical segments indexed by X.
        let mut tree: BTreeMap<ActiveKey, LinkedPointIndex> = BTreeMap::new();

        while let Some(Reverse(event)) = queue.pop() {
            match event.kind {
                EventKind::VertLow => {
                    let p = v_list.point(event.start);
                    let key = ActiveKey {
                        x: OrderedFloat(p.x()),
                        idx: event.start.0,
                    };
                    tree.insert(key, event.start);
                }
                EventKind::VertHigh => {
                    let p = v_list.point(event.start);
                    let key = ActiveKey {
                        x: OrderedFloat(p.x()),
                        idx: event.start.0,
                    };
                    tree.remove(&key);
                }
                EventKind::Horizontal => {
                    Self::intersect_with_tree(
                        h_list,
                        event.start,
                        v_list,
                        &tree,
                    );
                }
            }
        }
    }

    /// Query active vertical segments that cross the horizontal segment
    /// starting at `h_start`, inserting crossing points into both lists.
    fn intersect_with_tree(
        h_list: &mut LinkedPointList,
        h_start: LinkedPointIndex,
        v_list: &mut LinkedPointList,
        tree: &BTreeMap<ActiveKey, LinkedPointIndex>,
    ) {
        let h_p = h_list.point(h_start);
        let h_next_idx = match h_list.next(h_start) {
            Some(idx) => idx,
            None => return,
        };
        let h_next = h_list.point(h_next_idx);

        let y = h_p.y();
        let x_left = h_p.x().min(h_next.x());
        let x_right = h_p.x().max(h_next.x());
        let x_aligned = h_p.x() < h_next.x();

        let eps = GeomConstants::DISTANCE_EPSILON;

        // Collect vertical segment start indices whose X falls strictly
        // inside [x_left, x_right].
        let lo_key = ActiveKey {
            x: OrderedFloat(x_left),
            idx: 0,
        };
        let hi_key = ActiveKey {
            x: OrderedFloat(x_right),
            idx: usize::MAX,
        };

        // Gather candidates first to avoid borrowing issues.
        let candidates: Vec<(f64, LinkedPointIndex)> = tree
            .range(lo_key..=hi_key)
            .filter_map(|(k, &v_start)| {
                let vx = k.x.into_inner();
                if vx > x_left + eps && vx < x_right - eps {
                    Some((vx, v_start))
                } else {
                    None
                }
            })
            .collect();

        // Process in the correct order (left-to-right or right-to-left).
        let ordered: Vec<(f64, LinkedPointIndex)> = if x_aligned {
            candidates
        } else {
            let mut rev = candidates;
            rev.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
            rev
        };

        // Walk h_start forward, inserting crossing points.
        // We track a mutable cursor into the horizontal list.
        let mut h_cursor = h_start;
        for (vx, v_start) in ordered {
            let crossing = Point::new(vx, y);

            // Try to split the vertical segment.
            Self::try_split_vertical(v_list, v_start, crossing);

            // Try to split the horizontal segment at `h_cursor`.
            let (new_cursor, did_split) =
                Self::try_split_horizontal(h_list, h_cursor, crossing, x_aligned);
            if did_split {
                h_cursor = new_cursor;
            }
        }
    }

    /// Insert `point` into the vertical segment after `v_start` if it lies
    /// strictly between the segment's low and high Y endpoints.
    fn try_split_vertical(
        v_list: &mut LinkedPointList,
        v_start: LinkedPointIndex,
        point: Point,
    ) {
        let v_p = v_list.point(v_start);
        let v_next_idx = match v_list.next(v_start) {
            Some(idx) => idx,
            None => return,
        };
        let v_next = v_list.point(v_next_idx);
        let low = v_p.y().min(v_next.y());
        let high = v_p.y().max(v_next.y());
        let eps = GeomConstants::DISTANCE_EPSILON;
        if low + eps < point.y() && point.y() + eps < high {
            v_list.insert_after(v_start, point);
        }
    }

    /// Insert `point` into the horizontal segment after `h_cursor` if it lies
    /// strictly between the cursor and its next point.
    /// Returns the new cursor and whether a split happened.
    fn try_split_horizontal(
        h_list: &mut LinkedPointList,
        h_cursor: LinkedPointIndex,
        point: Point,
        x_aligned: bool,
    ) -> (LinkedPointIndex, bool) {
        let h_p = h_list.point(h_cursor);
        let h_next_idx = match h_list.next(h_cursor) {
            Some(idx) => idx,
            None => return (h_cursor, false),
        };
        let h_next = h_list.point(h_next_idx);
        let eps = GeomConstants::DISTANCE_EPSILON;
        let should_split = if x_aligned {
            h_p.x() + eps < point.x() && point.x() + eps < h_next.x()
        } else {
            h_next.x() + eps < point.x() && point.x() + eps < h_p.x()
        };
        if should_split {
            let new_node = h_list.insert_after(h_cursor, point);
            (new_node, true)
        } else {
            (h_cursor, false)
        }
    }
}
