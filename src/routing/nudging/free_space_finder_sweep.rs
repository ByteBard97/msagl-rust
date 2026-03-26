//! Sweep-line implementation for FreeSpaceFinder.
//!
//! Contains the FreeSpaceFinder struct and all sweep-line processing logic.
//! This is the faithful port of the C#/TS FreeSpaceFinder : LineSweeperBase.

use std::collections::{BTreeMap, BinaryHeap};

use ordered_float::OrderedFloat;

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use crate::routing::scan_direction::Direction;

use super::axis_edge::{AxisEdge, AxisEdgeId};
use super::free_space_finder_types::{
    AxisEdgesContainer, EventEntry, ObstacleId, ObstaclePolyline, ObstacleSide, SweepEvent,
    VertexIdx, get_lowest_vertex,
};

/// Snapshot of axis edge data for read access during sweep.
#[derive(Clone)]
struct EdgeSnapshot {
    source: Point,
    target: Point,
    direction: Direction,
}

/// Key for obstacle sides in BTreeMap, incorporating position and a unique id
/// to allow multiple sides at the same position.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct SideKey {
    perp: OrderedFloat<f64>,
    obstacle_id: OrderedFloat<f64>,
    vertex_idx: OrderedFloat<f64>,
}

impl SideKey {
    fn new(perp: f64, obstacle_id: usize, vertex_idx: usize) -> Self {
        Self {
            perp: OrderedFloat(perp),
            obstacle_id: OrderedFloat(obstacle_id as f64),
            vertex_idx: OrderedFloat(vertex_idx as f64),
        }
    }
}

const AREA_COMPARISON_EPSILON: f64 = GeomConstants::INTERSECTION_EPSILON;

/// The sweep-line state for finding free space.
///
/// Corresponds to C# `FreeSpaceFinder : LineSweeperBase`.
pub(super) struct FreeSpaceFinder {
    // --- LineSweeperBase fields ---
    sweep_dir: Point,
    dir_perp: Point,
    z: f64,
    event_queue: BinaryHeap<EventEntry>,
    event_seq: u64,

    /// Active left obstacle sides: C# LeftObstacleSideTree.
    left_side_tree: BTreeMap<SideKey, ObstacleSide>,

    /// Active right obstacle sides: C# RightObstacleSideTree.
    right_side_tree: BTreeMap<SideKey, ObstacleSide>,

    // --- FreeSpaceFinder-specific fields ---
    /// Active axis edge containers: C# edgeContainersTree.
    edge_containers: BTreeMap<OrderedFloat<f64>, AxisEdgesContainer>,

    /// The sweep pole direction (North or East).
    sweep_pole: Direction,

    /// For North sweep: x_projection = p.x; for East: x_projection = -p.y.
    x_projection_is_x: bool,

    /// Obstacle polylines (rectangles as 4-vertex closed polylines).
    polylines: Vec<ObstaclePolyline>,

    /// Accumulated left bound updates: (axis_edge_id, bound_value).
    left_bounds: Vec<(AxisEdgeId, f64)>,

    /// Accumulated right bound updates: (axis_edge_id, bound_value).
    right_bounds: Vec<(AxisEdgeId, f64)>,

    /// Accumulated right-neighbor pairs: (left_edge_id, right_edge_id).
    neighbor_pairs: Vec<(AxisEdgeId, AxisEdgeId)>,

    /// Snapshot of axis edge data needed during sweep.
    edge_data: Vec<EdgeSnapshot>,
}

impl FreeSpaceFinder {
    pub fn new(axis_edges: &[AxisEdge], obstacles: &[Rectangle], direction: Direction) -> Self {
        let sweep_dir = match direction {
            Direction::North => Point::new(0.0, 1.0),
            Direction::East => Point::new(1.0, 0.0),
        };
        let dir_perp = match direction {
            Direction::North => Point::new(1.0, 0.0),
            Direction::East => Point::new(0.0, -1.0),
        };
        let x_projection_is_x = direction == Direction::North;

        let polylines: Vec<ObstaclePolyline> = obstacles
            .iter()
            .enumerate()
            .map(|(i, r)| ObstaclePolyline::from_rect(i, r))
            .collect();

        let edge_data: Vec<EdgeSnapshot> = axis_edges
            .iter()
            .map(|ae| EdgeSnapshot {
                source: ae.source,
                target: ae.target,
                direction: ae.direction,
            })
            .collect();

        Self {
            sweep_dir,
            dir_perp,
            z: f64::NEG_INFINITY,
            event_queue: BinaryHeap::new(),
            event_seq: 0,
            left_side_tree: BTreeMap::new(),
            right_side_tree: BTreeMap::new(),
            edge_containers: BTreeMap::new(),
            sweep_pole: direction,
            x_projection_is_x,
            polylines,
            left_bounds: Vec::new(),
            right_bounds: Vec::new(),
            neighbor_pairs: Vec::new(),
            edge_data,
        }
    }

    /// C# xProjection: perpendicular projection.
    fn x_projection(&self, p: Point) -> f64 {
        if self.x_projection_is_x { p.x() } else { -p.y() }
    }

    /// C# GetZ(point): sweep position.
    fn get_z(&self, point: Point) -> f64 {
        self.sweep_dir.dot(point)
    }

    /// Enqueue a sweep event.
    fn enqueue_event(&mut self, event: SweepEvent) {
        let site = event.site();
        let z = self.get_z(site);
        let perp = self.dir_perp.dot(site);
        self.event_seq += 1;
        self.event_queue.push(EventEntry {
            z: OrderedFloat(z),
            perp: OrderedFloat(perp),
            seq: self.event_seq,
            event,
        });
    }

    /// C# FindFreeSpace(): main entry point.
    pub fn find_free_space(&mut self) {
        self.init_queue_of_events();
        self.process_events();
    }

    /// C# InitTheQueueOfEvents.
    fn init_queue_of_events(&mut self) {
        for poly in &self.polylines.clone() {
            let lowest = get_lowest_vertex(poly, self.sweep_dir, self.dir_perp);
            self.enqueue_event(SweepEvent::LowestVertex {
                site: poly.vertex(lowest),
                obstacle_id: poly.obstacle_id,
                vertex_idx: lowest,
            });
        }

        let edge_events: Vec<(AxisEdgeId, Point, Point)> = self
            .edge_data
            .iter()
            .enumerate()
            .filter(|(_, ae)| ae.direction == self.sweep_pole)
            .map(|(id, ae)| (id, ae.source, ae.target))
            .collect();
        for (ae_id, source, target) in edge_events {
            self.enqueue_event(SweepEvent::AxisEdgeLow {
                site: source,
                axis_edge_id: ae_id,
            });
            self.enqueue_event(SweepEvent::AxisEdgeHigh {
                site: target,
                axis_edge_id: ae_id,
            });
        }
    }

    /// C# ProcessEvents.
    fn process_events(&mut self) {
        while let Some(entry) = self.event_queue.pop() {
            self.process_event(entry.event);
        }
    }

    /// C# ProcessEvent: dispatch by event type.
    fn process_event(&mut self, event: SweepEvent) {
        match event {
            SweepEvent::LowestVertex { site, obstacle_id, vertex_idx } => {
                self.z = self.get_z(site);
                let poly = self.polylines[obstacle_id].clone();
                let next_vtx = poly.next_vertex(vertex_idx);
                let prev_vtx = poly.prev_vertex(vertex_idx);
                self.process_left_vertex(site, obstacle_id, vertex_idx, next_vtx);
                self.process_right_vertex(site, obstacle_id, vertex_idx, prev_vtx);
            }
            SweepEvent::LeftVertex { site, obstacle_id, vertex_idx } => {
                self.z = self.get_z(site);
                let poly = self.polylines[obstacle_id].clone();
                let next_vtx = poly.next_vertex(vertex_idx);
                self.process_left_vertex(site, obstacle_id, vertex_idx, next_vtx);
            }
            SweepEvent::RightVertex { site, obstacle_id, vertex_idx } => {
                self.z = self.get_z(site);
                let poly = self.polylines[obstacle_id].clone();
                let prev_vtx = poly.prev_vertex(vertex_idx);
                self.process_right_vertex(site, obstacle_id, vertex_idx, prev_vtx);
            }
            SweepEvent::AxisEdgeLow { site, axis_edge_id } => {
                self.z = self.get_z(site);
                self.process_low_edge_event(axis_edge_id);
            }
            SweepEvent::AxisEdgeHigh { site, axis_edge_id } => {
                self.z = self.get_z(site);
                self.process_high_edge_event(axis_edge_id);
            }
        }
    }

    /// C# ProcessLeftVertex.
    fn process_left_vertex(
        &mut self,
        site: Point,
        obstacle_id: ObstacleId,
        vertex_idx: VertexIdx,
        next_vertex_idx: VertexIdx,
    ) {
        let poly = &self.polylines[obstacle_id];
        let prev_vtx_idx = poly.prev_vertex(vertex_idx);
        let prev_site = poly.vertex(prev_vtx_idx);

        // C# ProcessPrevSegmentForLeftVertex.
        let delta = site - prev_site;
        let delta_z = delta.dot(self.sweep_dir);
        if delta_z > GeomConstants::DISTANCE_EPSILON {
            self.remove_left_side(obstacle_id, prev_vtx_idx);
        }

        let next_site = self.polylines[obstacle_id].vertex(next_vertex_idx);
        let delta_next = next_site - site;
        let delta_x = delta_next.dot(self.dir_perp);
        let delta_z_next = delta_next.dot(self.sweep_dir);

        if delta_z_next <= GeomConstants::DISTANCE_EPSILON {
            if delta_x < 0.0 && delta_z_next >= 0.0 {
                self.enqueue_event(SweepEvent::LeftVertex {
                    site: next_site,
                    obstacle_id,
                    vertex_idx: next_vertex_idx,
                });
            }
        } else {
            self.insert_left_side(obstacle_id, vertex_idx);
            self.enqueue_event(SweepEvent::LeftVertex {
                site: next_site,
                obstacle_id,
                vertex_idx: next_vertex_idx,
            });
        }

        self.restrict_edge_from_left_of_event(site);
    }

    /// C# ProcessRightVertex.
    fn process_right_vertex(
        &mut self,
        site: Point,
        obstacle_id: ObstacleId,
        vertex_idx: VertexIdx,
        next_vertex_idx: VertexIdx,
    ) {
        let poly = &self.polylines[obstacle_id];
        let prev_vtx_idx = poly.next_vertex(vertex_idx);
        let prev_site = poly.vertex(prev_vtx_idx);

        // C# ProcessPrevSegmentForRightVertex.
        let delta = site - prev_site;
        let delta_z = delta.dot(self.sweep_dir);
        if delta_z > GeomConstants::DISTANCE_EPSILON {
            self.remove_right_side(obstacle_id, prev_vtx_idx);
        }

        let next_site = self.polylines[obstacle_id].vertex(next_vertex_idx);
        let delta_next = next_site - site;
        let delta_x = delta_next.dot(self.dir_perp);
        let delta_z_next = delta_next.dot(self.sweep_dir);

        if delta_z_next <= GeomConstants::DISTANCE_EPSILON {
            if delta_x > 0.0 && delta_z_next >= 0.0 {
                self.enqueue_event(SweepEvent::RightVertex {
                    site: next_site,
                    obstacle_id,
                    vertex_idx: next_vertex_idx,
                });
            } else {
                self.restrict_edge_container_to_right_of_event(site);
            }
        } else {
            self.insert_right_side(obstacle_id, vertex_idx);
            self.enqueue_event(SweepEvent::RightVertex {
                site: next_site,
                obstacle_id,
                vertex_idx: next_vertex_idx,
            });
            self.restrict_edge_container_to_right_of_event(site);
        }
    }

    /// C# ProcessLowEdgeEvent.
    fn process_low_edge_event(&mut self, ae_id: AxisEdgeId) {
        let source = self.edge_data[ae_id].source;
        let prj = self.x_projection(source);
        let key = OrderedFloat(prj);
        let container_key = self.get_or_create_container_key(source, key);

        self.edge_containers.get_mut(&container_key).unwrap().add_edge(ae_id);

        // Right-neighbor discovery with prev container.
        if let Some((&prev_key, _)) = self.edge_containers.range(..container_key).next_back() {
            let prev_edges: Vec<AxisEdgeId> =
                self.edge_containers[&prev_key].edges.iter().copied().collect();
            let cur_edges: Vec<AxisEdgeId> =
                self.edge_containers[&container_key].edges.iter().copied().collect();
            for prev_edge in &prev_edges {
                for cur_edge in &cur_edges {
                    self.try_add_right_neighbor(*prev_edge, *cur_edge);
                }
            }
        }

        // Right-neighbor discovery with next container.
        if let Some((&next_key, _)) = self
            .edge_containers
            .range((std::ops::Bound::Excluded(container_key), std::ops::Bound::Unbounded))
            .next()
        {
            let cur_edges: Vec<AxisEdgeId> =
                self.edge_containers[&container_key].edges.iter().copied().collect();
            let next_edges: Vec<AxisEdgeId> =
                self.edge_containers[&next_key].edges.iter().copied().collect();
            for cur_edge in &cur_edges {
                for next_edge in &next_edges {
                    self.try_add_right_neighbor(*cur_edge, *next_edge);
                }
            }
        }

        self.constraint_edge_with_obstacles_at_z(ae_id, source);
    }

    /// C# ProcessHighEdgeEvent.
    fn process_high_edge_event(&mut self, ae_id: AxisEdgeId) {
        let target = self.edge_data[ae_id].target;
        let source = self.edge_data[ae_id].source;
        self.remove_edge_from_container(ae_id, source);
        self.constraint_edge_with_obstacles_at_z(ae_id, target);
    }

    /// C# GetOrCreateAxisEdgesContainer.
    fn get_or_create_container_key(
        &mut self,
        source: Point,
        _key: OrderedFloat<f64>,
    ) -> OrderedFloat<f64> {
        let prj = self.x_projection(source);
        let half_eps = GeomConstants::DISTANCE_EPSILON / 2.0;

        // C#: FindFirst(cont => xProjection(cont.Source) >= prj - eps/2)
        // Then check if xProjection(ret.Source) <= prj + eps/2
        if let Some((&k, container)) = self
            .edge_containers
            .range(OrderedFloat(prj - half_eps)..)
            .next()
        {
            let container_prj = self.x_projection(container.source);
            if container_prj <= prj + half_eps {
                return k;
            }
        }

        let key = OrderedFloat(prj);
        self.edge_containers.insert(key, AxisEdgesContainer::new(source));
        key
    }

    /// C# RemoveEdge.
    fn remove_edge_from_container(&mut self, ae_id: AxisEdgeId, source: Point) {
        let prj = self.x_projection(source);
        let half_eps = GeomConstants::DISTANCE_EPSILON / 2.0;

        let found_key = self
            .edge_containers
            .range(OrderedFloat(prj - half_eps)..)
            .next()
            .and_then(|(&k, container)| {
                let container_prj = self.x_projection(container.source);
                if container_prj <= prj + half_eps {
                    Some(k)
                } else {
                    None
                }
            });

        if let Some(key) = found_key {
            self.edge_containers.get_mut(&key).unwrap().remove_edge(ae_id);
            if self.edge_containers[&key].is_empty() {
                self.edge_containers.remove(&key);
            }
        }
    }

    fn try_add_right_neighbor(&mut self, left_id: AxisEdgeId, right_id: AxisEdgeId) {
        if self.projections_of_edges_overlap(left_id, right_id) {
            self.neighbor_pairs.push((left_id, right_id));
        }
    }

    /// C# ProjectionsOfEdgesOverlap.
    fn projections_of_edges_overlap(&self, left_id: AxisEdgeId, right_id: AxisEdgeId) -> bool {
        let left = &self.edge_data[left_id];
        let right = &self.edge_data[right_id];
        let eps = GeomConstants::DISTANCE_EPSILON;
        match self.sweep_pole {
            Direction::North => {
                !(left.target.y() < right.source.y() - eps
                    || right.target.y() < left.source.y() - eps)
            }
            Direction::East => {
                !(left.target.x() < right.source.x() - eps
                    || right.target.x() < left.source.x() - eps)
            }
        }
    }

    /// C# ConstraintEdgeWithObstaclesAtZ.
    fn constraint_edge_with_obstacles_at_z(&mut self, ae_id: AxisEdgeId, point: Point) {
        self.constraint_edge_from_left(ae_id, point);
        self.constraint_edge_from_right(ae_id, point);
    }

    /// C# ConstraintEdgeWithObstaclesAtZFromLeft.
    fn constraint_edge_from_left(&mut self, ae_id: AxisEdgeId, point: Point) {
        let found = self.get_active_right_side_from_left(point);
        if let Some(side) = found {
            let intersection = self.intersection_of_side_and_sweep_line(&side);
            let x = intersection.dot(self.dir_perp);
            self.left_bounds.push((ae_id, x));
        }
    }

    /// C# ConstraintEdgeWithObstaclesAtZFromRight.
    fn constraint_edge_from_right(&mut self, ae_id: AxisEdgeId, point: Point) {
        let found = self.get_active_left_side_from_right(point);
        if let Some(side) = found {
            let intersection = self.intersection_of_side_and_sweep_line(&side);
            let x = intersection.dot(self.dir_perp);
            self.right_bounds.push((ae_id, x));
        }
    }

    /// C# GetActiveSideFromLeft: find last right-side where point is to the right.
    fn get_active_right_side_from_left(&self, point: Point) -> Option<ObstacleSide> {
        let mut result = None;
        for (_, side) in self.right_side_tree.iter() {
            if point_to_right_of_line_or_on_line(point, side.start, side.end) {
                result = Some(side.clone());
            }
        }
        result
    }

    /// C# GetActiveSideFromRight: find first left-side where point is to the left.
    fn get_active_left_side_from_right(&self, point: Point) -> Option<ObstacleSide> {
        for (_, side) in self.left_side_tree.iter() {
            if point_to_left_of_line_or_on_line(point, side.start, side.end) {
                return Some(side.clone());
            }
        }
        None
    }

    /// C# ObstacleSideComparer.IntersectionOfSideAndSweepLine.
    fn intersection_of_side_and_sweep_line(&self, side: &ObstacleSide) -> Point {
        let dir = side.direction();
        let den = dir.dot(self.sweep_dir);
        if den.abs() < GeomConstants::DISTANCE_EPSILON {
            return side.start;
        }
        let t = (self.z - side.start.dot(self.sweep_dir)) / den;
        side.start + dir * t
    }

    /// C# RestrictEdgeFromTheLeftOfEvent.
    fn restrict_edge_from_left_of_event(&mut self, site: Point) {
        let site_x = self.x_projection(site);
        // C# GetContainerNodeToTheLeftOfEvent: findLast(container => xProj(source) <= siteX)
        let container_key = self
            .edge_containers
            .range(..=OrderedFloat(site_x + GeomConstants::DISTANCE_EPSILON))
            .next_back()
            .map(|(&k, _)| k);

        if let Some(key) = container_key {
            let edges: Vec<AxisEdgeId> =
                self.edge_containers[&key].edges.iter().copied().collect();
            let perp_val = self.dir_perp.dot(site);
            for edge_id in edges {
                self.right_bounds.push((edge_id, perp_val));
            }
        }
    }

    /// C# RestrictEdgeContainerToTheRightOfEvent.
    fn restrict_edge_container_to_right_of_event(&mut self, site: Point) {
        let site_x = self.x_projection(site);
        let container_key = self
            .edge_containers
            .range(OrderedFloat(site_x - GeomConstants::DISTANCE_EPSILON)..)
            .next()
            .map(|(&k, _)| k);

        if let Some(key) = container_key {
            let edges: Vec<AxisEdgeId> =
                self.edge_containers[&key].edges.iter().copied().collect();
            let perp_val = self.dir_perp.dot(site);
            for edge_id in edges {
                self.left_bounds.push((edge_id, perp_val));
            }
        }
    }

    // --- Obstacle side tree operations ---

    fn insert_left_side(&mut self, obstacle_id: ObstacleId, vertex_idx: VertexIdx) {
        let poly = &self.polylines[obstacle_id];
        let start = poly.vertex(vertex_idx);
        let end = poly.vertex(poly.next_vertex(vertex_idx));
        let side = ObstacleSide { start, end, obstacle_id };
        let intersection = self.intersection_of_side_and_sweep_line(&side);
        let perp = self.dir_perp.dot(intersection);
        self.left_side_tree.insert(SideKey::new(perp, obstacle_id, vertex_idx), side);
    }

    fn remove_left_side(&mut self, obstacle_id: ObstacleId, vertex_idx: VertexIdx) {
        let poly = &self.polylines[obstacle_id];
        let start = poly.vertex(vertex_idx);
        let end = poly.vertex(poly.next_vertex(vertex_idx));
        let side = ObstacleSide { start, end, obstacle_id };
        let intersection = self.intersection_of_side_and_sweep_line(&side);
        let perp = self.dir_perp.dot(intersection);
        self.left_side_tree.remove(&SideKey::new(perp, obstacle_id, vertex_idx));
    }

    fn insert_right_side(&mut self, obstacle_id: ObstacleId, vertex_idx: VertexIdx) {
        let poly = &self.polylines[obstacle_id];
        let start = poly.vertex(vertex_idx);
        let end = poly.vertex(poly.prev_vertex(vertex_idx));
        let side = ObstacleSide { start, end, obstacle_id };
        let intersection = self.intersection_of_side_and_sweep_line(&side);
        let perp = self.dir_perp.dot(intersection);
        self.right_side_tree.insert(SideKey::new(perp, obstacle_id, vertex_idx), side);
    }

    fn remove_right_side(&mut self, obstacle_id: ObstacleId, vertex_idx: VertexIdx) {
        let poly = &self.polylines[obstacle_id];
        let start = poly.vertex(vertex_idx);
        let end = poly.vertex(poly.prev_vertex(vertex_idx));
        let side = ObstacleSide { start, end, obstacle_id };
        let intersection = self.intersection_of_side_and_sweep_line(&side);
        let perp = self.dir_perp.dot(intersection);
        self.right_side_tree.remove(&SideKey::new(perp, obstacle_id, vertex_idx));
    }

    /// Apply accumulated bounds and neighbor relationships back to axis edges.
    pub fn apply(&self, axis_edges: &mut [AxisEdge]) {
        for &(ae_id, bound) in &self.left_bounds {
            axis_edges[ae_id].bound_from_left(bound);
        }
        for &(ae_id, bound) in &self.right_bounds {
            axis_edges[ae_id].bound_from_right(bound);
        }
        for &(left_id, right_id) in &self.neighbor_pairs {
            if !axis_edges[left_id].right_neighbors.contains(&right_id) {
                axis_edges[left_id].right_neighbors.push(right_id);
            }
        }
    }
}

/// C# PointToTheLeftOfLineOrOnLineLocal.
fn point_to_left_of_line_or_on_line(a: Point, line_start: Point, line_end: Point) -> bool {
    Point::signed_doubled_triangle_area(a, line_start, line_end) > -AREA_COMPARISON_EPSILON
}

/// C# PointToTheRightOfLineOrOnLineLocal.
fn point_to_right_of_line_or_on_line(a: Point, line_start: Point, line_end: Point) -> bool {
    Point::signed_doubled_triangle_area(line_start, line_end, a) < AREA_COMPARISON_EPSILON
}
