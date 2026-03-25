use std::collections::BinaryHeap;
use std::cmp::Ordering;
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use super::scan_direction::ScanDirection;

/// Full sweep event hierarchy for the rectilinear visibility sweep line.
///
/// Mirrors the TS EventQueue.ts hierarchy:
///   SweepEvent
///   ├── VertexEvent  (OpenVertex, CloseVertex, LowBend, HighBend)
///   └── ReflectionEvent (LowReflection, HighReflection)
#[derive(Clone, Debug)]
pub enum SweepEvent {
    /// Obstacle corner entering sweep range (low side of obstacle).
    OpenVertex { site: Point, obstacle_index: usize },
    /// Obstacle corner leaving sweep range (high side of obstacle).
    CloseVertex { site: Point, obstacle_index: usize },
    /// Low bend at obstacle corner (between open and close).
    LowBend { site: Point, obstacle_index: usize },
    /// High bend at obstacle corner (between open and close).
    HighBend { site: Point, obstacle_index: usize },
    /// Reflection event off a low obstacle side.
    LowReflection {
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
        prev_event_index: Option<usize>,
    },
    /// Reflection event off a high obstacle side.
    HighReflection {
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
        prev_event_index: Option<usize>,
    },
}

impl SweepEvent {
    /// The spatial site of this event.
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

    /// Returns `true` if this is a reflection event.
    pub fn is_reflection(&self) -> bool {
        matches!(self, Self::LowReflection { .. } | Self::HighReflection { .. })
    }

    /// Event type priority (lower = processed first at the same coordinate).
    ///
    /// Matches TS EventQueue.ts ordering:
    ///   reflection events (0) → open (1) → bend (2) → close (3)
    fn type_priority(&self) -> u8 {
        match self {
            Self::LowReflection { .. } | Self::HighReflection { .. } => 0,
            Self::OpenVertex { .. } => 1,
            Self::LowBend { .. } | Self::HighBend { .. } => 2,
            Self::CloseVertex { .. } => 3,
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
        // Reverse ordering for min-heap (BinaryHeap is max-heap by default).
        // Primary: smallest perp_coord first.
        let perp = GeomConstants::compare(other.perp_coord, self.perp_coord);
        if perp != Ordering::Equal {
            return perp;
        }
        // Secondary: lower type_priority value first (reflection < open < bend < close).
        let tp = self.event.type_priority().cmp(&other.event.type_priority());
        if tp != Ordering::Equal {
            return tp.reverse();
        }
        // Tertiary: smaller scan_coord first.
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
        let site = event.site();
        let oe = OrderedEvent {
            perp_coord: self.scan_direction.perp_coord(site),
            scan_coord: self.scan_direction.coord(site),
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
