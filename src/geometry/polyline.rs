use slotmap::SlotMap;
use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;

#[derive(Clone, Copy, Debug)]
pub struct PolylinePoint {
    pub key: PolylinePointKey,
    pub point: Point,
}

#[derive(Clone, Debug)]
pub struct Polyline {
    nodes: SlotMap<PolylinePointKey, PolylinePointData>,
    start: Option<PolylinePointKey>,
    end: Option<PolylinePointKey>,
    closed: bool,
}

#[derive(Clone, Debug)]
struct PolylinePointData {
    point: Point,
    next: Option<PolylinePointKey>,
    prev: Option<PolylinePointKey>,
}

impl Polyline {
    pub fn new() -> Self {
        Self {
            nodes: SlotMap::with_key(),
            start: None,
            end: None,
            closed: false,
        }
    }
    pub fn is_closed(&self) -> bool { self.closed }
    pub fn points(&self) -> impl Iterator<Item = Point> + '_ {
        PointIter { poly: self, current: self.start, done: false }
    }
}

impl Default for Polyline {
    fn default() -> Self { Self::new() }
}

struct PointIter<'a> {
    poly: &'a Polyline,
    current: Option<PolylinePointKey>,
    done: bool,
}

impl<'a> Iterator for PointIter<'a> {
    type Item = Point;
    fn next(&mut self) -> Option<Point> {
        if self.done { return None; }
        let key = self.current?;
        let data = &self.poly.nodes[key];
        let point = data.point;
        self.current = data.next;
        if self.current.is_none() && self.poly.closed {
            self.done = true;
        }
        Some(point)
    }
}
