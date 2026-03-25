use std::collections::BinaryHeap;
use std::cmp::Ordering;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use super::scan_direction::ScanDirection;

/// Sweep event types for rectangular obstacle corners.
#[derive(Clone, Debug)]
pub enum SweepEvent {
    /// Opening corner: obstacle's low side enters the scan line.
    Open { point: Point, obstacle_index: usize },
    /// Closing corner: obstacle's side exits the scan line.
    Close { point: Point, obstacle_index: usize },
}

impl SweepEvent {
    pub fn point(&self) -> Point {
        match self {
            SweepEvent::Open { point, .. } => *point,
            SweepEvent::Close { point, .. } => *point,
        }
    }

    pub fn obstacle_index(&self) -> usize {
        match self {
            SweepEvent::Open { obstacle_index, .. } => *obstacle_index,
            SweepEvent::Close { obstacle_index, .. } => *obstacle_index,
        }
    }

    /// Priority: Open events before Close events at same position.
    fn type_priority(&self) -> u8 {
        match self {
            SweepEvent::Open { .. } => 0,
            SweepEvent::Close { .. } => 1,
        }
    }
}

/// Wrapper for BinaryHeap ordering (min-heap by perpendicular coordinate).
struct OrderedEvent {
    event: SweepEvent,
    perp_coord: f64,
    scan_coord: f64,
}

impl PartialEq for OrderedEvent {
    fn eq(&self, other: &Self) -> bool {
        GeomConstants::close(self.perp_coord, other.perp_coord)
            && GeomConstants::close(self.scan_coord, other.scan_coord)
            && self.event.type_priority() == other.event.type_priority()
    }
}
impl Eq for OrderedEvent {}

impl PartialOrd for OrderedEvent {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OrderedEvent {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse ordering for min-heap (BinaryHeap is max-heap)
        let perp = GeomConstants::compare(other.perp_coord, self.perp_coord);
        if perp != Ordering::Equal { return perp; }
        let tp = self.event.type_priority().cmp(&other.event.type_priority());
        if tp != Ordering::Equal { return tp.reverse(); }
        GeomConstants::compare(other.scan_coord, self.scan_coord)
    }
}

/// Priority queue of sweep events, ordered by perpendicular coordinate.
pub struct EventQueue {
    scan_direction: ScanDirection,
    heap: BinaryHeap<OrderedEvent>,
}

impl EventQueue {
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self {
            scan_direction,
            heap: BinaryHeap::new(),
        }
    }

    pub fn enqueue(&mut self, event: SweepEvent) {
        let point = event.point();
        let oe = OrderedEvent {
            perp_coord: self.scan_direction.perp_coord(point),
            scan_coord: self.scan_direction.coord(point),
            event,
        };
        self.heap.push(oe);
    }

    pub fn dequeue(&mut self) -> Option<SweepEvent> {
        self.heap.pop().map(|oe| oe.event)
    }

    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    pub fn len(&self) -> usize {
        self.heap.len()
    }
}
