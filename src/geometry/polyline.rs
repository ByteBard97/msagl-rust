// src/geometry/polyline.rs
use slotmap::SlotMap;

use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;

#[derive(Clone, Debug)]
struct PolylinePointData {
    point: Point,
    next: Option<PolylinePointKey>,
    prev: Option<PolylinePointKey>,
}

/// Public view of a polyline point (key + point value).
#[derive(Clone, Copy, Debug)]
pub struct PolylinePoint {
    pub key: PolylinePointKey,
    pub point: Point,
}

/// A doubly-linked list of points backed by a SlotMap.
/// Supports both open and closed polylines.
#[derive(Clone, Debug)]
pub struct Polyline {
    nodes: SlotMap<PolylinePointKey, PolylinePointData>,
    start: Option<PolylinePointKey>,
    end: Option<PolylinePointKey>,
    closed: bool,
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

    pub fn from_points(pts: &[Point]) -> Self {
        let mut poly = Self::new();
        for &p in pts {
            poly.add_point(p);
        }
        poly
    }

    /// Append a point to the end. Returns the key.
    pub fn add_point(&mut self, p: Point) -> PolylinePointKey {
        let key = self.nodes.insert(PolylinePointData {
            point: p,
            next: None,
            prev: self.end,
        });
        if let Some(end_key) = self.end {
            self.nodes[end_key].next = Some(key);
        } else {
            self.start = Some(key);
        }
        self.end = Some(key);
        key
    }

    /// Prepend a point to the start. Returns the key.
    pub fn prepend_point(&mut self, p: Point) -> PolylinePointKey {
        let key = self.nodes.insert(PolylinePointData {
            point: p,
            next: self.start,
            prev: None,
        });
        if let Some(start_key) = self.start {
            self.nodes[start_key].prev = Some(key);
        } else {
            self.end = Some(key);
        }
        self.start = Some(key);
        key
    }

    pub fn count(&self) -> usize {
        self.nodes.len()
    }
    pub fn is_closed(&self) -> bool {
        self.closed
    }
    pub fn set_closed(&mut self, closed: bool) {
        self.closed = closed;
    }
    pub fn start_key(&self) -> Option<PolylinePointKey> {
        self.start
    }
    pub fn end_key(&self) -> Option<PolylinePointKey> {
        self.end
    }

    /// First point. Panics if empty.
    pub fn start(&self) -> Point {
        self.nodes[self.start.expect("empty polyline")].point
    }

    /// Last point. Panics if empty.
    pub fn end(&self) -> Point {
        self.nodes[self.end.expect("empty polyline")].point
    }

    pub fn point_at(&self, key: PolylinePointKey) -> Point {
        self.nodes[key].point
    }

    pub fn set_point_at(&mut self, key: PolylinePointKey, p: Point) {
        self.nodes[key].point = p;
    }

    /// Next key. For closed polylines, wraps from end to start.
    pub fn next_key(&self, key: PolylinePointKey) -> Option<PolylinePointKey> {
        let next = self.nodes[key].next;
        if next.is_some() {
            next
        } else if self.closed {
            self.start
        } else {
            None
        }
    }

    /// Previous key. For closed polylines, wraps from start to end.
    pub fn prev_key(&self, key: PolylinePointKey) -> Option<PolylinePointKey> {
        let prev = self.nodes[key].prev;
        if prev.is_some() {
            prev
        } else if self.closed {
            self.end
        } else {
            None
        }
    }

    /// Iterate over points in order.
    pub fn points(&self) -> PointIter<'_> {
        PointIter {
            poly: self,
            current: self.start,
            done: false,
        }
    }

    /// Iterate over (key, point) pairs.
    pub fn polyline_points(&self) -> PolylinePointIter<'_> {
        PolylinePointIter {
            poly: self,
            current: self.start,
            done: false,
        }
    }

    pub fn bounding_box(&self) -> Rectangle {
        let mut bb = Rectangle::empty();
        for p in self.points() {
            bb.add_point(p);
        }
        bb
    }

    pub fn length(&self) -> f64 {
        let mut total = 0.0;
        let mut current = self.start;
        while let Some(key) = current {
            let next = self.nodes[key].next;
            if let Some(next_key) = next {
                let a = self.nodes[key].point;
                let b = self.nodes[next_key].point;
                total += (b - a).length();
            }
            current = next;
        }
        if self.closed {
            if let (Some(s), Some(e)) = (self.start, self.end) {
                total += (self.nodes[s].point - self.nodes[e].point).length();
            }
        }
        total
    }

    pub fn translate(&mut self, delta: Point) {
        for data in self.nodes.values_mut() {
            data.point = data.point + delta;
        }
    }

    pub fn remove_start_point(&mut self) {
        let start_key = self.start.expect("empty polyline");
        let next = self.nodes[start_key].next;
        self.nodes.remove(start_key);
        self.start = next;
        if let Some(next_key) = next {
            self.nodes[next_key].prev = None;
        } else {
            self.end = None;
        }
    }

    pub fn remove_end_point(&mut self) {
        let end_key = self.end.expect("empty polyline");
        let prev = self.nodes[end_key].prev;
        self.nodes.remove(end_key);
        self.end = prev;
        if let Some(prev_key) = prev {
            self.nodes[prev_key].next = None;
        } else {
            self.start = None;
        }
    }
}

impl Default for Polyline {
    fn default() -> Self {
        Self::new()
    }
}

pub struct PointIter<'a> {
    poly: &'a Polyline,
    current: Option<PolylinePointKey>,
    done: bool,
}

impl<'a> Iterator for PointIter<'a> {
    type Item = Point;
    fn next(&mut self) -> Option<Point> {
        if self.done {
            return None;
        }
        let key = self.current?;
        let data = &self.poly.nodes[key];
        let point = data.point;
        let next = data.next;
        if next.is_none() && self.poly.closed {
            self.done = true;
        }
        self.current = next;
        Some(point)
    }
}

pub struct PolylinePointIter<'a> {
    poly: &'a Polyline,
    current: Option<PolylinePointKey>,
    done: bool,
}

impl<'a> Iterator for PolylinePointIter<'a> {
    type Item = PolylinePoint;
    fn next(&mut self) -> Option<PolylinePoint> {
        if self.done {
            return None;
        }
        let key = self.current?;
        let data = &self.poly.nodes[key];
        let pp = PolylinePoint {
            key,
            point: data.point,
        };
        let next = data.next;
        if next.is_none() && self.poly.closed {
            self.done = true;
        }
        self.current = next;
        Some(pp)
    }
}
