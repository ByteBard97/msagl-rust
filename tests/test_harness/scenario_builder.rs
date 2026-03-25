/// Test harness for rectilinear routing scenarios.
///
/// ScenarioBuilder accumulates rectangular obstacles and edge requests, then
/// runs the router and returns the RoutingResult for verification.
use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, RoutingResult, Shape};

/// Default padding around obstacles used in rectilinear scenario tests.
const SCENARIO_PADDING: f64 = 1.0;
/// Default edge separation used in rectilinear scenario tests.
const SCENARIO_EDGE_SEPARATION: f64 = 1.0;

/// Pending edge: pair of obstacle indices to route between.
struct PendingEdge {
    src_obstacle: usize,
    src_offset: Point,
    tgt_obstacle: usize,
    tgt_offset: Point,
}

pub struct ScenarioBuilder {
    shapes: Vec<Shape>,
    edges: Vec<PendingEdge>,
    padding: f64,
    edge_separation: f64,
}

impl ScenarioBuilder {
    /// Create a new scenario with default routing parameters matching
    /// the C# RectilinearVerifier defaults (padding=1.0, separation=1.0).
    pub fn new() -> Self {
        Self {
            shapes: Vec::new(),
            edges: Vec::new(),
            padding: SCENARIO_PADDING,
            edge_separation: SCENARIO_EDGE_SEPARATION,
        }
    }

    /// Override obstacle padding.
    pub fn with_padding(mut self, p: f64) -> Self {
        self.padding = p;
        self
    }

    /// Override edge separation.
    pub fn with_edge_separation(mut self, s: f64) -> Self {
        self.edge_separation = s;
        self
    }

    /// Add a rectangle by center coordinates. Returns the obstacle index.
    pub fn add_rectangle(&mut self, cx: f64, cy: f64, width: f64, height: f64) -> usize {
        let idx = self.shapes.len();
        self.shapes.push(Shape::rectangle_centered(cx, cy, width, height));
        idx
    }

    /// Add a rectangle by bottom-left corner. Returns the obstacle index.
    pub fn add_rectangle_bl(&mut self, x: f64, y: f64, width: f64, height: f64) -> usize {
        let idx = self.shapes.len();
        self.shapes.push(Shape::rectangle(x, y, width, height));
        idx
    }

    /// Schedule an edge from the center of obstacle `src` to the center of `tgt`.
    pub fn route_between(&mut self, src: usize, tgt: usize) {
        self.route_between_offsets(src, Point::new(0.0, 0.0), tgt, Point::new(0.0, 0.0));
    }

    /// Schedule an edge with explicit offsets from each obstacle center.
    pub fn route_between_offsets(
        &mut self,
        src: usize,
        src_offset: Point,
        tgt: usize,
        tgt_offset: Point,
    ) {
        self.edges.push(PendingEdge {
            src_obstacle: src,
            src_offset,
            tgt_obstacle: tgt,
            tgt_offset,
        });
    }

    /// Return a slice of all shapes registered so far.
    pub fn shapes(&self) -> &[Shape] {
        &self.shapes
    }

    /// Run the router and return the result.
    ///
    /// Pending edges are converted to floating ports placed at each obstacle's
    /// bounding-box center (plus any requested offset).
    pub fn run(self) -> RoutingResult {
        let mut router = RectilinearEdgeRouter::new(&self.shapes)
            .padding(self.padding)
            .edge_separation(self.edge_separation);

        for e in &self.edges {
            let src_center = obstacle_center(&self.shapes, e.src_obstacle);
            let tgt_center = obstacle_center(&self.shapes, e.tgt_obstacle);
            let src_loc = Point::new(src_center.x() + e.src_offset.x(), src_center.y() + e.src_offset.y());
            let tgt_loc = Point::new(tgt_center.x() + e.tgt_offset.x(), tgt_center.y() + e.tgt_offset.y());
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(e.src_obstacle, src_loc),
                FloatingPort::new(e.tgt_obstacle, tgt_loc),
            ));
        }

        router.run()
    }
}

impl Default for ScenarioBuilder {
    fn default() -> Self {
        Self::new()
    }
}

fn obstacle_center(shapes: &[Shape], idx: usize) -> Point {
    let bb = shapes[idx].bounding_box();
    Point::new(
        (bb.left() + bb.right()) / 2.0,
        (bb.bottom() + bb.top()) / 2.0,
    )
}
