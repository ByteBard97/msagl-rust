//! Priority queue of sweep events for rectilinear visibility graph generation.
//!
//! Faithful port of MSAGL TS:
//! - `EventQueue.ts` (63 lines)
//! - `BasicVertexEvent.ts`, `OpenVertexEvent.ts`, `MiscVertexEvents.ts`
//! - `BasicReflectionEvent.ts`, `HighReflectionEvent.ts`, `LowReflectionEvent.ts`
//!
//! Event hierarchy (as Rust enum):
//!   SweepEvent
//!   ├── OpenVertex    (obstacle corner entering sweep range)
//!   ├── CloseVertex   (obstacle corner leaving sweep range)
//!   ├── LowBend       (low bend at obstacle corner)
//!   ├── HighBend      (high bend at obstacle corner)
//!   ├── LowReflection (reflection off a low obstacle side)
//!   └── HighReflection(reflection off a high obstacle side)
//!
//! Comparison order (matching TS EventQueue.Compare):
//!   1. Perpendicular coordinate (smaller first)
//!   2. Reflection events before non-reflection events
//!   3. Scan coordinate (smaller first)

use std::collections::BinaryHeap;
use std::cmp::Ordering;
use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use super::scan_direction::ScanDirection;

/// Full sweep event hierarchy for the rectilinear visibility sweep line.
///
/// Mirrors the TS EventQueue.ts hierarchy:
///   SweepEvent
///   ├── VertexEvent  (OpenVertex, CloseVertex, LowBend, HighBend)
///   └── ReflectionEvent (LowReflection, HighReflection)
///
/// Vertex events carry a `vertex_key` that identifies the PolylinePoint in
/// the obstacle's boundary polyline. This matches the TS where each vertex
/// event holds a `PolylinePoint` giving both the point and traversal position.
#[derive(Clone, Debug)]
pub enum SweepEvent {
    /// Obstacle corner entering sweep range (low side of obstacle).
    OpenVertex { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    /// Obstacle corner leaving sweep range (high side of obstacle).
    CloseVertex { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    /// Low bend at obstacle corner (between open and close).
    LowBend { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    /// High bend at obstacle corner (between open and close).
    HighBend { site: Point, obstacle_index: usize, vertex_key: PolylinePointKey },
    /// Reflection event off a low obstacle side.
    /// Matches TS: `LowReflectionEvent extends BasicReflectionEvent`
    ///
    /// - `initial_obstacle`: TS `BasicReflectionEvent.InitialObstacle`
    /// - `reflecting_obstacle`: TS `BasicReflectionEvent.ReflectingObstacle`
    /// - `low_side_obstacle`: obstacle owning the target LowObstacleSide
    /// - `prev_event_index`: TS `BasicReflectionEvent.PreviousSite` (stored by index)
    LowReflection {
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
        low_side_obstacle: Option<usize>,
        prev_event_index: Option<usize>,
    },
    /// Reflection event off a high obstacle side.
    /// Matches TS: `HighReflectionEvent extends BasicReflectionEvent`
    HighReflection {
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
        high_side_obstacle: Option<usize>,
        prev_event_index: Option<usize>,
    },
}

impl SweepEvent {
    /// The spatial site of this event.
    /// Matches TS: `SweepEvent.Site`
    pub fn site(&self) -> Point {
        match self {
            Self::OpenVertex { site, .. }
            | Self::CloseVertex { site, .. }
            | Self::LowBend { site, .. }
            | Self::HighBend { site, .. }
            | Self::LowReflection { site, .. }
            | Self::HighReflection { site, .. } => *site,
        }
    }

    /// The obstacle index for vertex events; `None` for reflection events.
    pub fn obstacle_index(&self) -> Option<usize> {
        match self {
            Self::OpenVertex { obstacle_index, .. }
            | Self::CloseVertex { obstacle_index, .. }
            | Self::LowBend { obstacle_index, .. }
            | Self::HighBend { obstacle_index, .. } => Some(*obstacle_index),
            Self::LowReflection { .. } | Self::HighReflection { .. } => None,
        }
    }

    /// The polyline point key for vertex events; `None` for reflection events.
    pub fn vertex_key(&self) -> Option<PolylinePointKey> {
        match self {
            Self::OpenVertex { vertex_key, .. }
            | Self::CloseVertex { vertex_key, .. }
            | Self::LowBend { vertex_key, .. }
            | Self::HighBend { vertex_key, .. } => Some(*vertex_key),
            Self::LowReflection { .. } | Self::HighReflection { .. } => None,
        }
    }

    /// Returns `true` if this is a reflection event.
    /// Matches TS: `lhs instanceof BasicReflectionEvent`
    pub fn is_reflection(&self) -> bool {
        matches!(self, Self::LowReflection { .. } | Self::HighReflection { .. })
    }

    /// Returns `true` if this is a vertex event (Open, Close, LowBend, HighBend).
    pub fn is_vertex_event(&self) -> bool {
        !self.is_reflection()
    }

    /// For reflection events: the initial obstacle that started the chain.
    /// Matches TS: `BasicReflectionEvent.InitialObstacle`
    pub fn initial_obstacle(&self) -> Option<usize> {
        match self {
            Self::LowReflection { initial_obstacle, .. }
            | Self::HighReflection { initial_obstacle, .. } => Some(*initial_obstacle),
            _ => None,
        }
    }

    /// For reflection events: the obstacle being reflected off.
    /// Matches TS: `BasicReflectionEvent.ReflectingObstacle`
    pub fn reflecting_obstacle(&self) -> Option<usize> {
        match self {
            Self::LowReflection { reflecting_obstacle, .. }
            | Self::HighReflection { reflecting_obstacle, .. } => Some(*reflecting_obstacle),
            _ => None,
        }
    }

    /// For reflection events: index of the previous reflection event in the chain.
    /// Matches TS: `BasicReflectionEvent.PreviousSite`
    pub fn prev_event_index(&self) -> Option<usize> {
        match self {
            Self::LowReflection { prev_event_index, .. }
            | Self::HighReflection { prev_event_index, .. } => *prev_event_index,
            _ => None,
        }
    }

    /// Event type priority for sub-ordering within the same perpendicular coordinate.
    ///
    /// The TS only distinguishes reflection vs non-reflection, but we refine
    /// the ordering for non-reflection events for determinism:
    ///   reflection(0) < open(1) < bend(2) < close(3)
    fn type_priority(&self) -> u8 {
        match self {
            Self::LowReflection { .. } | Self::HighReflection { .. } => 0,
            Self::OpenVertex { .. } => 1,
            Self::LowBend { .. } | Self::HighBend { .. } => 2,
            Self::CloseVertex { .. } => 3,
        }
    }

    /// Check if this reflection event forms a staircase step with the given target.
    /// Matches TS: `BasicReflectionEvent.IsStaircaseStep(reflectionTarget)`
    ///   returns `this.InitialObstacle === reflectionTarget`
    pub fn is_staircase_step(&self, reflection_target: usize) -> bool {
        match self {
            Self::LowReflection { initial_obstacle, .. }
            | Self::HighReflection { initial_obstacle, .. } => {
                *initial_obstacle == reflection_target
            }
            _ => false,
        }
    }

    /// Create a LowReflection from a previous reflection event.
    /// Matches TS: `new LowReflectionEvent(previousSite, targetSide, site)`
    /// where `previousSite.ReflectingObstacle` becomes `initial_obstacle`.
    pub fn new_low_reflection(
        site: Point,
        prev_reflecting_obstacle: usize,
        low_side_obstacle: usize,
        prev_event_index: Option<usize>,
    ) -> Self {
        Self::LowReflection {
            site,
            initial_obstacle: prev_reflecting_obstacle,
            reflecting_obstacle: low_side_obstacle,
            low_side_obstacle: Some(low_side_obstacle),
            prev_event_index,
        }
    }

    /// Create a HighReflection from a previous reflection event.
    /// Matches TS: `new HighReflectionEvent(previousSite, targetSide, site)`
    pub fn new_high_reflection(
        site: Point,
        prev_reflecting_obstacle: usize,
        high_side_obstacle: usize,
        prev_event_index: Option<usize>,
    ) -> Self {
        Self::HighReflection {
            site,
            initial_obstacle: prev_reflecting_obstacle,
            reflecting_obstacle: high_side_obstacle,
            high_side_obstacle: Some(high_side_obstacle),
            prev_event_index,
        }
    }

    /// Create a BasicReflectionEvent (used by StoreLookaheadSite).
    /// Matches TS: `new BasicReflectionEvent(initialObstacle, reflectingObstacle, site)`
    pub fn new_basic_reflection(
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
    ) -> Self {
        // TS base class is neither Low nor High specifically.
        // We use LowReflection as the concrete variant with no side obstacle.
        Self::LowReflection {
            site,
            initial_obstacle,
            reflecting_obstacle,
            low_side_obstacle: None,
            prev_event_index: None,
        }
    }
}

/// Wrapper for BinaryHeap ordering (min-heap by perpendicular coordinate).
///
/// Comparison order matches TS `EventQueue.Compare()`:
///   1. Perpendicular coordinate (smaller first)
///   2. Reflection events before non-reflection events
///   3. Scan coordinate (smaller first)
struct OrderedEvent {
    event: SweepEvent,
    perp_coord: f64,
    scan_coord: f64,
}

impl PartialEq for OrderedEvent {
    fn eq(&self, other: &Self) -> bool {
        GeomConstants::close(self.perp_coord, other.perp_coord)
            && self.event.type_priority() == other.event.type_priority()
            && GeomConstants::close(self.scan_coord, other.scan_coord)
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
        // Reverse ordering for min-heap (BinaryHeap is max-heap by default).
        // Primary: smallest perp_coord first.
        let perp = GeomConstants::compare(other.perp_coord, self.perp_coord);
        if perp != Ordering::Equal {
            return perp;
        }
        // Secondary: event type priority (reflection < open < bend < close).
        // TS only distinguishes reflection vs non-reflection; we refine with
        // type_priority for determinism. Lower type_priority = higher priority.
        // Reversed for min-heap.
        let tp = self.event.type_priority().cmp(&other.event.type_priority());
        if tp != Ordering::Equal {
            return tp.reverse();
        }
        // Tertiary: smaller scan_coord first (reversed for min-heap).
        GeomConstants::compare(other.scan_coord, self.scan_coord)
    }
}

/// Priority queue of sweep events, ordered by perpendicular coordinate.
///
/// Faithful port of TS `EventQueue` (EventQueue.ts, 63 lines).
pub struct EventQueue {
    scan_direction: ScanDirection,
    heap: BinaryHeap<OrderedEvent>,
}

impl EventQueue {
    /// Create a new event queue with the given scan direction.
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self {
            scan_direction,
            heap: BinaryHeap::new(),
        }
    }

    /// Reset the queue for a new scan direction.
    /// Matches TS: `EventQueue.Reset(scanDir)`
    pub fn reset(&mut self, scan_direction: ScanDirection) {
        debug_assert!(self.heap.is_empty(), "Stray events in EventQueue.Reset");
        self.scan_direction = scan_direction;
    }

    /// Enqueue a sweep event.
    /// Matches TS: `EventQueue.Enqueue(evt)`
    pub fn enqueue(&mut self, event: SweepEvent) {
        let site = event.site();
        let oe = OrderedEvent {
            perp_coord: self.scan_direction.perp_coord(site),
            scan_coord: self.scan_direction.coord(site),
            event,
        };
        self.heap.push(oe);
    }

    /// Dequeue the highest-priority event (smallest perpendicular coordinate).
    /// Matches TS: `EventQueue.Dequeue()`
    pub fn dequeue(&mut self) -> Option<SweepEvent> {
        self.heap.pop().map(|oe| oe.event)
    }

    /// Check if the queue is empty.
    pub fn is_empty(&self) -> bool {
        self.heap.is_empty()
    }

    /// Number of events in the queue.
    /// Matches TS: `EventQueue.Count`
    pub fn len(&self) -> usize {
        self.heap.len()
    }

    /// The current scan direction.
    pub fn scan_direction(&self) -> ScanDirection {
        self.scan_direction
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dk() -> PolylinePointKey { PolylinePointKey::default() }

    #[test]
    fn events_dequeued_in_perp_coord_order() {
        let sd = ScanDirection::horizontal();
        let mut q = EventQueue::new(sd);

        q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 10.0), obstacle_index: 0, vertex_key: dk() });
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 5.0), obstacle_index: 1, vertex_key: dk() });
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(0.0, 15.0), obstacle_index: 2, vertex_key: dk() });

        // For horizontal scan, perp_coord = y. Should come out 5, 10, 15.
        let e1 = q.dequeue().unwrap();
        let e2 = q.dequeue().unwrap();
        let e3 = q.dequeue().unwrap();
        assert!(e1.site().y() < e2.site().y());
        assert!(e2.site().y() < e3.site().y());
    }

    #[test]
    fn reflections_before_vertex_events_at_same_coord() {
        let sd = ScanDirection::horizontal();
        let mut q = EventQueue::new(sd);

        // Vertex event at y=5
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(3.0, 5.0), obstacle_index: 0, vertex_key: dk() });
        // Reflection event at y=5
        q.enqueue(SweepEvent::new_basic_reflection(Point::new(3.0, 5.0), 1, 2));

        let first = q.dequeue().unwrap();
        let second = q.dequeue().unwrap();
        assert!(first.is_reflection(), "Reflection should come first");
        assert!(!second.is_reflection(), "Vertex event should come second");
    }

    #[test]
    fn scan_coord_tiebreaker_at_same_perp_and_type() {
        let sd = ScanDirection::horizontal();
        let mut q = EventQueue::new(sd);

        // Two open vertex events at same y=5, different x
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(10.0, 5.0), obstacle_index: 0, vertex_key: dk() });
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(3.0, 5.0), obstacle_index: 1, vertex_key: dk() });

        let first = q.dequeue().unwrap();
        let second = q.dequeue().unwrap();
        // For horizontal scan, scan coord = x. Smaller x should come first.
        assert!(first.site().x() < second.site().x());
    }

    #[test]
    fn staircase_step_detection() {
        let evt = SweepEvent::new_basic_reflection(Point::new(0.0, 0.0), 5, 10);
        assert!(evt.is_staircase_step(5));
        assert!(!evt.is_staircase_step(10));
    }

    #[test]
    fn reset_clears_direction() {
        let mut q = EventQueue::new(ScanDirection::horizontal());
        q.reset(ScanDirection::vertical());
        assert!(q.scan_direction().is_vertical());
    }

    #[test]
    fn reflection_constructors() {
        let low = SweepEvent::new_low_reflection(Point::new(1.0, 2.0), 3, 4, Some(0));
        assert!(low.is_reflection());
        assert_eq!(low.initial_obstacle(), Some(3));
        assert_eq!(low.reflecting_obstacle(), Some(4));
        assert_eq!(low.prev_event_index(), Some(0));

        let high = SweepEvent::new_high_reflection(Point::new(5.0, 6.0), 7, 8, None);
        assert!(high.is_reflection());
        assert_eq!(high.initial_obstacle(), Some(7));
        assert_eq!(high.reflecting_obstacle(), Some(8));
        assert_eq!(high.prev_event_index(), None);
    }

    #[test]
    fn vertical_scan_direction_ordering() {
        let sd = ScanDirection::vertical();
        let mut q = EventQueue::new(sd);

        // For vertical scan: perp_coord = x, scan_coord = y
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(10.0, 0.0), obstacle_index: 0, vertex_key: dk() });
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(3.0, 0.0), obstacle_index: 1, vertex_key: dk() });

        let first = q.dequeue().unwrap();
        let second = q.dequeue().unwrap();
        // Smaller x (perp) should come first
        assert!(first.site().x() < second.site().x());
    }

    #[test]
    fn all_event_types_ordered_correctly() {
        let sd = ScanDirection::horizontal();
        let mut q = EventQueue::new(sd);
        let y = 5.0;

        // Enqueue events at same perp coord (y=5), same scan coord (x=3)
        q.enqueue(SweepEvent::CloseVertex { site: Point::new(3.0, y), obstacle_index: 0, vertex_key: dk() });
        q.enqueue(SweepEvent::HighBend { site: Point::new(3.0, y), obstacle_index: 0, vertex_key: dk() });
        q.enqueue(SweepEvent::OpenVertex { site: Point::new(3.0, y), obstacle_index: 0, vertex_key: dk() });
        q.enqueue(SweepEvent::LowBend { site: Point::new(3.0, y), obstacle_index: 0, vertex_key: dk() });
        q.enqueue(SweepEvent::new_basic_reflection(Point::new(3.0, y), 0, 1));

        // Reflection should come first, rest ordered by scan coord (same here)
        let first = q.dequeue().unwrap();
        assert!(first.is_reflection(), "Reflection should be first");

        // Remaining 4 are all non-reflection at same coord — order is stable
        for _ in 0..4 {
            let evt = q.dequeue().unwrap();
            assert!(!evt.is_reflection());
        }
        assert!(q.is_empty());
    }

    #[test]
    fn empty_queue() {
        let mut q = EventQueue::new(ScanDirection::horizontal());
        assert!(q.is_empty());
        assert_eq!(q.len(), 0);
        assert!(q.dequeue().is_none());
    }
}
