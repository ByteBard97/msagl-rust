//! FreeSpaceFinder: sweep-line algorithm to compute movement bounds for axis edges.
//!
//! Faithfully ported from FreeSpaceFinder.ts (498 lines). Uses a sweep-line that
//! processes obstacle vertex events and axis-edge low/high-point events to compute
//! how far each axis edge can move perpendicular to its direction without hitting
//! an obstacle. Also discovers right-neighbor relationships between axis edges.

use std::collections::{BTreeMap, BinaryHeap};

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use crate::routing::scan_direction::Direction;

use super::axis_edge::{AxisEdge, AxisEdgeId};
use super::free_space_finder_types::*;

/// Find free space around axis edges by constraining them with obstacles.
///
/// Public entry point. Internally runs the full sweep-line algorithm
/// from FreeSpaceFinder.ts.
pub fn find_free_space(
    axis_edges: &mut [AxisEdge],
    obstacles: &[Rectangle],
    direction: Direction,
) {
    // In the TS, `AxisEdgesToObstaclesTheyOriginatedFrom` maps axis edges to
    // the obstacle they came from. Our axis edges come from path refinement,
    // not obstacle boundaries, so no edge originates from any obstacle.
    let obstacle_origin: Vec<Option<ObstacleId>> = vec![None; axis_edges.len()];
    let mut finder = FreeSpaceFinderState::new(direction, obstacles, &obstacle_origin, axis_edges);
    finder.find_free_space(axis_edges);
}

/// Internal sweep-line state.
struct FreeSpaceFinderState<'a> {
    sweep_direction: Point,
    direction_perp: Point,
    sweep_pole: Direction,
    x_projection_is_north: bool,
    obstacle_polys: Vec<ObstaclePolyline>,
    obstacle_origin: &'a [Option<ObstacleId>],
    event_queue: BinaryHeap<EventEntry>,
    event_seq: u64,
    z: f64,
    left_sides: BTreeMap<OrderedFloat<f64>, Vec<ObstacleSide>>,
    right_sides: BTreeMap<OrderedFloat<f64>, Vec<ObstacleSide>>,
    edge_containers: BTreeMap<OrderedFloat<f64>, AxisEdgesContainer>,
}

impl<'a> FreeSpaceFinderState<'a> {
    fn new(
        direction: Direction,
        obstacles: &[Rectangle],
        obstacle_origin: &'a [Option<ObstacleId>],
        _axis_edges: &[AxisEdge],
    ) -> Self {
        let sweep_direction = match direction {
            Direction::North => Point::new(0.0, 1.0),
            Direction::East => Point::new(1.0, 0.0),
        };
        // TS: DirectionPerp = CompassVector(direction).Right.ToPoint()
        let direction_perp = match direction {
            Direction::North => Point::new(1.0, 0.0),  // Right of North = East
            Direction::East => Point::new(0.0, -1.0),  // Right of East = South
        };
        let obstacle_polys: Vec<ObstaclePolyline> = obstacles
            .iter()
            .enumerate()
            .map(|(i, r)| ObstaclePolyline::from_rect(i, r))
            .collect();

        Self {
            sweep_direction,
            direction_perp,
            sweep_pole: direction,
            x_projection_is_north: direction == Direction::North,
            obstacle_polys,
            obstacle_origin,
            event_queue: BinaryHeap::new(),
            event_seq: 0,
            z: f64::NEG_INFINITY,
            left_sides: BTreeMap::new(),
            right_sides: BTreeMap::new(),
            edge_containers: BTreeMap::new(),
        }
    }

    fn x_projection(&self, p: Point) -> f64 {
        if self.x_projection_is_north { p.x() } else { -p.y() }
    }

    fn get_z(&self, point: Point) -> f64 {
        point.dot(self.sweep_direction)
    }

    fn enqueue_event(&mut self, event: SweepEvent) {
        let site = event.site();
        self.event_seq += 1;
        self.event_queue.push(EventEntry {
            z: OrderedFloat(site.dot(self.sweep_direction)),
            perp: OrderedFloat(site.dot(self.direction_perp)),
            seq: self.event_seq,
            event,
        });
    }

    // -- TS: FindFreeSpace --
    fn find_free_space(&mut self, axis_edges: &mut [AxisEdge]) {
        self.init_queue_of_events(axis_edges);
        while let Some(entry) = self.event_queue.pop() {
            self.process_event(entry.event, axis_edges);
        }
    }

    // -- TS: InitTheQueueOfEvents --
    fn init_queue_of_events(&mut self, axis_edges: &[AxisEdge]) {
        let sd = self.sweep_direction;
        let dp = self.direction_perp;
        let obstacle_events: Vec<_> = self.obstacle_polys.iter().map(|poly| {
            let idx = get_lowest_vertex(poly, sd, dp);
            SweepEvent::LowestVertex {
                site: poly.vertex(idx),
                obstacle_id: poly.obstacle_id,
                vertex_idx: idx,
            }
        }).collect();
        for event in obstacle_events {
            self.enqueue_event(event);
        }
        for (ae_id, ae) in axis_edges.iter().enumerate() {
            if ae.direction == self.sweep_pole {
                self.enqueue_event(SweepEvent::AxisEdgeLow {
                    site: ae.source, axis_edge_id: ae_id,
                });
                self.enqueue_event(SweepEvent::AxisEdgeHigh {
                    site: ae.target, axis_edge_id: ae_id,
                });
            }
        }
    }

    // -- TS: ProcessEvent --
    fn process_event(&mut self, event: SweepEvent, axis_edges: &mut [AxisEdge]) {
        match event {
            SweepEvent::LowestVertex { obstacle_id, vertex_idx, .. } => {
                self.z = self.get_z(self.obstacle_polys[obstacle_id].vertex(vertex_idx));
                self.process_lowest_vertex(obstacle_id, vertex_idx, axis_edges);
            }
            SweepEvent::LeftVertex { obstacle_id, vertex_idx, .. } => {
                self.z = self.get_z(self.obstacle_polys[obstacle_id].vertex(vertex_idx));
                self.process_left_vertex(obstacle_id, vertex_idx, axis_edges);
            }
            SweepEvent::RightVertex { obstacle_id, vertex_idx, .. } => {
                self.z = self.get_z(self.obstacle_polys[obstacle_id].vertex(vertex_idx));
                self.process_right_vertex(obstacle_id, vertex_idx, axis_edges);
            }
            SweepEvent::AxisEdgeLow { site, axis_edge_id } => {
                self.z = self.get_z(site);
                self.process_low_edge_event(axis_edge_id, axis_edges);
            }
            SweepEvent::AxisEdgeHigh { site, axis_edge_id } => {
                self.z = self.get_z(site);
                self.process_high_edge_event(axis_edge_id, axis_edges);
            }
        }
    }

    // -- TS: ProcessLowEdgeEvent --
    fn process_low_edge_event(&mut self, ae_id: AxisEdgeId, axis_edges: &mut [AxisEdge]) {
        let source = axis_edges[ae_id].source;
        let key = OrderedFloat(self.x_projection(source));
        self.edge_containers
            .entry(key)
            .or_insert_with(|| AxisEdgesContainer::new(source))
            .add_edge(ae_id);

        let container_edges: Vec<AxisEdgeId> =
            self.edge_containers[&key].edges.iter().copied().collect();

        // Previous container (left neighbor).
        if let Some((_, prev)) = self.edge_containers.range(..key).next_back() {
            let prev_edges: Vec<AxisEdgeId> = prev.edges.iter().copied().collect();
            for &pe in &prev_edges {
                for &ce in &container_edges {
                    if self.projections_overlap(&axis_edges[pe], &axis_edges[ce]) {
                        axis_edges[pe].right_neighbors.push(ce);
                    }
                }
            }
        }
        // Next container (right neighbor).
        if let Some((_, next)) = self.edge_containers
            .range((std::ops::Bound::Excluded(key), std::ops::Bound::Unbounded))
            .next()
        {
            let next_edges: Vec<AxisEdgeId> = next.edges.iter().copied().collect();
            for &ce in &container_edges {
                for &ne in &next_edges {
                    if self.projections_overlap(&axis_edges[ce], &axis_edges[ne]) {
                        axis_edges[ce].right_neighbors.push(ne);
                    }
                }
            }
        }
        self.constrain_edge_at_z(ae_id, axis_edges[ae_id].source, axis_edges);
    }

    // -- TS: ProcessHighEdgeEvent --
    fn process_high_edge_event(&mut self, ae_id: AxisEdgeId, axis_edges: &mut [AxisEdge]) {
        // Remove edge from container.
        let source = axis_edges[ae_id].source;
        let key = OrderedFloat(self.x_projection(source));
        if let Some(container) = self.edge_containers.get_mut(&key) {
            container.remove_edge(ae_id);
            if container.is_empty() {
                self.edge_containers.remove(&key);
            }
        }
        self.constrain_edge_at_z(ae_id, axis_edges[ae_id].target, axis_edges);
    }

    // -- TS: ProjectionsOfEdgesOverlap --
    fn projections_overlap(&self, left: &AxisEdge, right: &AxisEdge) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        if self.sweep_pole == Direction::North {
            !(left.target.y() < right.source.y() - eps
                || right.target.y() < left.source.y() - eps)
        } else {
            !(left.target.x() < right.source.x() - eps
                || right.target.x() < left.source.x() - eps)
        }
    }

    // -- TS: ConstraintEdgeWithObstaclesAtZ --
    fn constrain_edge_at_z(&self, ae_id: AxisEdgeId, point: Point, ae: &mut [AxisEdge]) {
        // From right: find left-side that the point is to the left of.
        if let Some(side) = self.find_left_side_from_right(point) {
            if !self.not_restricting(ae_id, side.obstacle_id) {
                let x = self.intersect_side_sweep(&side);
                ae[ae_id].bound_from_right(x.dot(self.direction_perp));
            }
        }
        // From left: find right-side that the point is to the right of.
        if let Some(side) = self.find_right_side_from_left(point) {
            if !self.not_restricting(ae_id, side.obstacle_id) {
                let x = self.intersect_side_sweep(&side);
                ae[ae_id].bound_from_left(x.dot(self.direction_perp));
            }
        }
    }

    // -- TS: GetActiveSideFromRight (searches LeftObstacleSideTree) --
    fn find_left_side_from_right(&self, point: Point) -> Option<ObstacleSide> {
        for sides in self.left_sides.values() {
            for side in sides {
                if point_to_left_or_on(point, side.start, side.end) {
                    return Some(side.clone());
                }
            }
        }
        None
    }

    // -- TS: GetActiveSideFromLeft (searches RightObstacleSideTree) --
    fn find_right_side_from_left(&self, point: Point) -> Option<ObstacleSide> {
        for sides in self.right_sides.values().rev() {
            for side in sides.iter().rev() {
                if point_to_right_or_on(point, side.start, side.end) {
                    return Some(side.clone());
                }
            }
        }
        None
    }

    // -- TS: NotRestricting --
    fn not_restricting(&self, ae_id: AxisEdgeId, obstacle_id: ObstacleId) -> bool {
        self.obstacle_origin.get(ae_id).and_then(|o| *o) == Some(obstacle_id)
    }

    // -- TS: ObstacleSideComparer.IntersectionOfSideAndSweepLine --
    fn intersect_side_sweep(&self, side: &ObstacleSide) -> Point {
        let dir = side.direction();
        let den = dir.dot(self.sweep_direction);
        if den.abs() < GeomConstants::DISTANCE_EPSILON {
            return side.start;
        }
        let t = (self.z - side.start.dot(self.sweep_direction)) / den;
        side.start + dir * t
    }

    // -- Obstacle vertex processing --

    fn process_lowest_vertex(
        &mut self, oid: ObstacleId, vidx: VertexIdx, ae: &mut [AxisEdge],
    ) {
        let next = self.obstacle_polys[oid].next_vertex(vidx);
        let prev = self.obstacle_polys[oid].prev_vertex(vidx);
        self.do_left_vertex(oid, vidx, next, ae);
        self.do_right_vertex(oid, vidx, prev, ae);
    }

    fn process_left_vertex(&mut self, oid: ObstacleId, vidx: VertexIdx, ae: &mut [AxisEdge]) {
        let next = self.obstacle_polys[oid].next_vertex(vidx);
        self.do_left_vertex(oid, vidx, next, ae);
    }

    fn process_right_vertex(&mut self, oid: ObstacleId, vidx: VertexIdx, ae: &mut [AxisEdge]) {
        let next = self.obstacle_polys[oid].prev_vertex(vidx);
        self.do_right_vertex(oid, vidx, next, ae);
    }

    // -- TS: ProcessLeftVertex body --
    fn do_left_vertex(
        &mut self, oid: ObstacleId, vidx: VertexIdx, next_idx: VertexIdx, ae: &mut [AxisEdge],
    ) {
        let poly = self.obstacle_polys[oid].clone();
        let site = poly.vertex(vidx);
        let prev_idx = poly.prev_vertex(vidx);
        let prev_site = poly.vertex(prev_idx);

        // ProcessPrevSegmentForLeftVertex
        if (site - prev_site).dot(self.sweep_direction) > GeomConstants::DISTANCE_EPSILON {
            self.remove_left_side(prev_site, site, oid);
        }

        let next_pt = poly.vertex(next_idx);
        let delta = next_pt - site;
        let delta_x = delta.dot(self.direction_perp);
        let delta_z = delta.dot(self.sweep_direction);

        if delta_z <= GeomConstants::DISTANCE_EPSILON {
            if delta_x < 0.0 && delta_z >= 0.0 {
                self.enqueue_event(SweepEvent::LeftVertex {
                    site: next_pt, obstacle_id: oid, vertex_idx: next_idx,
                });
            }
        } else {
            self.insert_left_side(site, next_pt, oid);
            self.enqueue_event(SweepEvent::LeftVertex {
                site: next_pt, obstacle_id: oid, vertex_idx: next_idx,
            });
        }
        // RestrictEdgeFromTheLeftOfEvent
        self.restrict_from_left(site, oid, ae);
    }

    // -- TS: ProcessRightVertex body --
    fn do_right_vertex(
        &mut self, oid: ObstacleId, vidx: VertexIdx, next_idx: VertexIdx, ae: &mut [AxisEdge],
    ) {
        let poly = self.obstacle_polys[oid].clone();
        let site = poly.vertex(vidx);
        let next_on_poly = poly.next_vertex(vidx);
        let prev_site = poly.vertex(next_on_poly);

        // ProcessPrevSegmentForRightVertex
        if (site - prev_site).dot(self.sweep_direction) > GeomConstants::DISTANCE_EPSILON {
            self.remove_right_side(prev_site, site, oid);
        }

        let next_pt = poly.vertex(next_idx);
        let delta = next_pt - site;
        let delta_x = delta.dot(self.direction_perp);
        let delta_z = delta.dot(self.sweep_direction);

        if delta_z <= GeomConstants::DISTANCE_EPSILON {
            if delta_x > 0.0 && delta_z >= 0.0 {
                self.enqueue_event(SweepEvent::RightVertex {
                    site: next_pt, obstacle_id: oid, vertex_idx: next_idx,
                });
            } else {
                self.restrict_from_right(site, oid, ae);
            }
        } else {
            self.insert_right_side(site, next_pt, oid);
            self.enqueue_event(SweepEvent::RightVertex {
                site: next_pt, obstacle_id: oid, vertex_idx: next_idx,
            });
            self.restrict_from_right(site, oid, ae);
        }
    }

    // -- TS: RestrictEdgeFromTheLeftOfEvent --
    fn restrict_from_left(&self, site: Point, oid: ObstacleId, ae: &mut [AxisEdge]) {
        let site_x = OrderedFloat(self.x_projection(site));
        if let Some((_, container)) = self.edge_containers.range(..=site_x).next_back() {
            let edges: Vec<AxisEdgeId> = container.edges.iter().copied().collect();
            for id in edges {
                if !self.not_restricting(id, oid) {
                    ae[id].bound_from_right(site.dot(self.direction_perp));
                }
            }
        }
    }

    // -- TS: RestrictEdgeContainerToTheRightOfEvent --
    fn restrict_from_right(&self, site: Point, oid: ObstacleId, ae: &mut [AxisEdge]) {
        let site_x = OrderedFloat(self.x_projection(site));
        if let Some((_, container)) = self.edge_containers.range(site_x..).next() {
            let edges: Vec<AxisEdgeId> = container.edges.iter().copied().collect();
            for id in edges {
                if !self.not_restricting(id, oid) {
                    ae[id].bound_from_left(site.dot(self.direction_perp));
                }
            }
        }
    }

    // -- Obstacle side management --

    fn insert_left_side(&mut self, start: Point, end: Point, oid: ObstacleId) {
        let key = OrderedFloat(self.side_perp(start, end));
        self.left_sides.entry(key).or_default().push(ObstacleSide { start, end, obstacle_id: oid });
    }

    fn remove_left_side(&mut self, start: Point, end: Point, oid: ObstacleId) {
        let key = OrderedFloat(self.side_perp(start, end));
        if let Some(sides) = self.left_sides.get_mut(&key) {
            sides.retain(|s| !(s.obstacle_id == oid && s.start.close_to(start)));
            if sides.is_empty() { self.left_sides.remove(&key); }
        }
    }

    fn insert_right_side(&mut self, start: Point, end: Point, oid: ObstacleId) {
        let key = OrderedFloat(self.side_perp(start, end));
        self.right_sides.entry(key).or_default().push(ObstacleSide { start, end, obstacle_id: oid });
    }

    fn remove_right_side(&mut self, start: Point, end: Point, oid: ObstacleId) {
        let key = OrderedFloat(self.side_perp(start, end));
        if let Some(sides) = self.right_sides.get_mut(&key) {
            sides.retain(|s| !(s.obstacle_id == oid && s.start.close_to(start)));
            if sides.is_empty() { self.right_sides.remove(&key); }
        }
    }

    fn side_perp(&self, start: Point, end: Point) -> f64 {
        let dir = end - start;
        let den = dir.dot(self.sweep_direction);
        if den.abs() < GeomConstants::DISTANCE_EPSILON {
            return start.dot(self.direction_perp);
        }
        let t = (self.z - start.dot(self.sweep_direction)) / den;
        (start + dir * t).dot(self.direction_perp)
    }
}

// -- TS: PointToTheLeftOfLineOrOnLineLocal --
fn point_to_left_or_on(a: Point, line0: Point, line1: Point) -> bool {
    Point::signed_doubled_triangle_area(a, line0, line1) > -GeomConstants::INTERSECTION_EPSILON
}

// -- TS: PointToTheRightOfLineOrOnLineLocal --
fn point_to_right_or_on(a: Point, line0: Point, line1: Point) -> bool {
    Point::signed_doubled_triangle_area(line0, line1, a) < GeomConstants::INTERSECTION_EPSILON
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_edge_no_obstacles() {
        let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
        find_free_space(&mut edges, &[], Direction::North);
        assert_eq!(edges[0].left_bound, f64::NEG_INFINITY);
        assert_eq!(edges[0].right_bound, f64::INFINITY);
    }

    #[test]
    fn edge_bounded_by_obstacle_on_left() {
        let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
        let obs = vec![Rectangle::new(0.0, 0.0, 3.0, 10.0)];
        find_free_space(&mut edges, &obs, Direction::North);
        assert!(edges[0].left_bound >= 3.0 - GeomConstants::DISTANCE_EPSILON);
    }

    #[test]
    fn edge_bounded_by_obstacle_on_right() {
        let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
        let obs = vec![Rectangle::new(7.0, 0.0, 10.0, 10.0)];
        find_free_space(&mut edges, &obs, Direction::North);
        assert!(edges[0].right_bound <= 7.0 + GeomConstants::DISTANCE_EPSILON);
    }

    #[test]
    fn edge_bounded_both_sides() {
        let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
        let obs = vec![
            Rectangle::new(0.0, 0.0, 3.0, 10.0),
            Rectangle::new(7.0, 0.0, 10.0, 10.0),
        ];
        find_free_space(&mut edges, &obs, Direction::North);
        assert!(edges[0].left_bound >= 3.0 - GeomConstants::DISTANCE_EPSILON);
        assert!(edges[0].right_bound <= 7.0 + GeomConstants::DISTANCE_EPSILON);
    }

    #[test]
    fn right_neighbors_discovered() {
        let mut edges = vec![
            AxisEdge::new(Point::new(3.0, 0.0), Point::new(3.0, 10.0)),
            AxisEdge::new(Point::new(7.0, 0.0), Point::new(7.0, 10.0)),
        ];
        find_free_space(&mut edges, &[], Direction::North);
        assert!(edges[0].right_neighbors.contains(&1),
            "Edge 0 should have edge 1 as right neighbor, got: {:?}", edges[0].right_neighbors);
    }

    #[test]
    fn non_overlapping_projections_no_neighbors() {
        let mut edges = vec![
            AxisEdge::new(Point::new(3.0, 0.0), Point::new(3.0, 5.0)),
            AxisEdge::new(Point::new(7.0, 6.0), Point::new(7.0, 10.0)),
        ];
        find_free_space(&mut edges, &[], Direction::North);
        assert!(edges[0].right_neighbors.is_empty());
    }

    #[test]
    fn east_direction_sweep() {
        let mut edges = vec![AxisEdge::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0))];
        let obs = vec![Rectangle::new(0.0, 7.0, 10.0, 10.0)];
        find_free_space(&mut edges, &obs, Direction::East);
        assert!(edges[0].left_bound > f64::NEG_INFINITY || edges[0].right_bound < f64::INFINITY);
    }

    #[test]
    fn east_direction_right_neighbors() {
        let mut edges = vec![
            AxisEdge::new(Point::new(0.0, 5.0), Point::new(10.0, 5.0)),
            AxisEdge::new(Point::new(0.0, 3.0), Point::new(10.0, 3.0)),
        ];
        find_free_space(&mut edges, &[], Direction::East);
        assert!(edges[0].right_neighbors.contains(&1),
            "East: edge at y=5 should have edge at y=3 as right neighbor, got: {:?}",
            edges[0].right_neighbors);
    }

    #[test]
    fn obstacle_not_overlapping_along_direction() {
        let mut edges = vec![AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0))];
        let obs = vec![Rectangle::new(3.0, 15.0, 7.0, 20.0)];
        find_free_space(&mut edges, &obs, Direction::North);
        assert_eq!(edges[0].left_bound, f64::NEG_INFINITY);
        assert_eq!(edges[0].right_bound, f64::INFINITY);
    }

    #[test]
    fn multiple_edges_multiple_obstacles() {
        let mut edges = vec![
            AxisEdge::new(Point::new(2.0, 0.0), Point::new(2.0, 10.0)),
            AxisEdge::new(Point::new(5.0, 0.0), Point::new(5.0, 10.0)),
            AxisEdge::new(Point::new(8.0, 0.0), Point::new(8.0, 10.0)),
        ];
        let obs = vec![
            Rectangle::new(-1.0, 0.0, 0.0, 10.0),
            Rectangle::new(10.0, 0.0, 11.0, 10.0),
        ];
        find_free_space(&mut edges, &obs, Direction::North);
        assert!(edges[0].right_neighbors.contains(&1));
        assert!(edges[1].right_neighbors.contains(&2));
    }
}
