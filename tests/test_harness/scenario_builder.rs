//! Builder pattern for creating routing test scenarios.
//!
//! Mirrors the C# test helpers from `RectilinearEdgeRouterWrapper.cs`.

#![allow(dead_code)]

use msagl_rust::routing::rectilinear_edge_router::{RectilinearEdgeRouter, RoutingResult};
use msagl_rust::routing::shape::Shape;
use msagl_rust::routing::port::FloatingPort;
use msagl_rust::routing::edge_geometry::EdgeGeometry;
use msagl_rust::geometry::point::Point;

/// Default padding around obstacles (matches C# default).
const DEFAULT_PADDING: f64 = 1.0;
/// Default separation between parallel edges.
const DEFAULT_EDGE_SEPARATION: f64 = 1.0;
/// Default bend penalty as percentage of source-target Manhattan distance.
const DEFAULT_BEND_PENALTY: f64 = 4.0;

/// Builder for routing test scenarios.
///
/// Add shapes and edges, then call `run()` to execute routing.
pub struct ScenarioBuilder {
    shapes: Vec<Shape>,
    edges: Vec<(FloatingPort, FloatingPort)>,
    padding: f64,
    edge_separation: f64,
    bend_penalty: f64,
}

impl ScenarioBuilder {
    pub fn new() -> Self {
        Self {
            shapes: Vec::new(),
            edges: Vec::new(),
            padding: DEFAULT_PADDING,
            edge_separation: DEFAULT_EDGE_SEPARATION,
            bend_penalty: DEFAULT_BEND_PENALTY,
        }
    }

    /// Add a rectangle obstacle centered at (center_x, center_y).
    /// Returns the obstacle index.
    pub fn add_rectangle(
        &mut self,
        center_x: f64,
        center_y: f64,
        width: f64,
        height: f64,
    ) -> usize {
        let idx = self.shapes.len();
        self.shapes
            .push(Shape::rectangle_centered(center_x, center_y, width, height));
        idx
    }

    /// Add obstacle from bottom-left / top-right corners.
    /// Matches C# `PolylineFromRectanglePoints` pattern.
    pub fn add_rectangle_bl(
        &mut self,
        left: f64,
        bottom: f64,
        right: f64,
        top: f64,
    ) -> usize {
        let idx = self.shapes.len();
        self.shapes
            .push(Shape::rectangle(left, bottom, right - left, top - bottom));
        idx
    }

    /// Create a port at the center of an obstacle.
    pub fn center_port(&self, obstacle_index: usize) -> FloatingPort {
        let bb = self.shapes[obstacle_index].bounding_box();
        FloatingPort::new(obstacle_index, bb.center())
    }

    /// Create a port at an absolute position on an obstacle.
    pub fn port_at(&self, obstacle_index: usize, location: Point) -> FloatingPort {
        FloatingPort::new(obstacle_index, location)
    }

    /// Add a route between two ports.
    pub fn route(&mut self, source: FloatingPort, target: FloatingPort) -> &mut Self {
        self.edges.push((source, target));
        self
    }

    /// Add a route between the centers of two obstacles.
    pub fn route_between(&mut self, source_idx: usize, target_idx: usize) -> &mut Self {
        let src = self.center_port(source_idx);
        let tgt = self.center_port(target_idx);
        self.edges.push((src, tgt));
        self
    }

    /// Set obstacle padding.
    pub fn padding(&mut self, p: f64) -> &mut Self {
        self.padding = p;
        self
    }

    /// Set edge separation.
    pub fn edge_separation(&mut self, s: f64) -> &mut Self {
        self.edge_separation = s;
        self
    }

    /// Set bend penalty as percentage.
    pub fn bend_penalty(&mut self, p: f64) -> &mut Self {
        self.bend_penalty = p;
        self
    }

    /// Access the shapes slice (for verification).
    pub fn shapes(&self) -> &[Shape] {
        &self.shapes
    }

    /// Access the configured padding (for verification).
    pub fn get_padding(&self) -> f64 {
        self.padding
    }

    /// Execute routing and return the result.
    pub fn run(&self) -> RoutingResult {
        let mut router = RectilinearEdgeRouter::new(&self.shapes)
            .padding(self.padding)
            .edge_separation(self.edge_separation)
            .bend_penalty_as_percentage(self.bend_penalty);

        for (src, tgt) in &self.edges {
            router.add_edge(EdgeGeometry::new(src.clone(), tgt.clone()));
        }

        router.run()
    }
}

impl Default for ScenarioBuilder {
    fn default() -> Self {
        Self::new()
    }
}
