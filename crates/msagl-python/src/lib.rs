//! Python bindings for msagl-rust rectilinear edge router.
//!
//! Exposes native Python classes (`Obstacle`, `Port`, `Router`, etc.)
//! wrapping the core `msagl_rust::RectilinearEdgeRouter`.

use pyo3::prelude::*;

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

/// A rectangular obstacle on the canvas.
#[pyclass]
#[derive(Clone, Debug)]
struct Obstacle {
    #[pyo3(get)]
    x: f64,
    #[pyo3(get)]
    y: f64,
    #[pyo3(get)]
    width: f64,
    #[pyo3(get)]
    height: f64,
    /// Internal index assigned when obstacles are added to a Router.
    index: Option<usize>,
}

#[pymethods]
impl Obstacle {
    #[new]
    fn new(x: f64, y: f64, width: f64, height: f64) -> Self {
        Self {
            x,
            y,
            width,
            height,
            index: None,
        }
    }

    /// Create a port at absolute canvas coordinates (x, y) on this obstacle.
    ///
    /// The obstacle must have been passed to a `Router` before calling this,
    /// so that it has a valid index. Coordinates are absolute canvas position,
    /// not relative to the obstacle.
    fn port(&self, x: f64, y: f64) -> PyResult<Port> {
        let idx = self.index.ok_or_else(|| {
            pyo3::exceptions::PyValueError::new_err(
                "Obstacle has no index — pass it to a Router first",
            )
        })?;
        Ok(Port {
            obstacle_index: idx,
            x,
            y,
        })
    }

    fn __repr__(&self) -> String {
        format!(
            "Obstacle(x={}, y={}, width={}, height={})",
            self.x, self.y, self.width, self.height,
        )
    }
}

/// A port (connection point) on an obstacle.
///
/// Created via `Obstacle.port(x, y)`. The x/y coordinates are absolute
/// canvas position, not relative to the obstacle.
#[pyclass]
#[derive(Clone, Debug)]
struct Port {
    #[pyo3(get)]
    obstacle_index: usize,
    #[pyo3(get)]
    x: f64,
    #[pyo3(get)]
    y: f64,
}

#[pymethods]
impl Port {
    fn __repr__(&self) -> String {
        format!(
            "Port(obstacle_index={}, x={}, y={})",
            self.obstacle_index, self.x, self.y,
        )
    }
}

/// A routed path consisting of a sequence of (x, y) waypoints.
#[pyclass]
#[derive(Clone, Debug)]
struct Path {
    #[pyo3(get)]
    points: Vec<(f64, f64)>,
}

#[pymethods]
impl Path {
    fn __repr__(&self) -> String {
        format!("Path(points={})", format_points(&self.points))
    }
}

/// The result of running the router, containing all routed paths.
#[pyclass]
#[derive(Clone, Debug)]
struct RoutingResult {
    #[pyo3(get)]
    paths: Vec<Path>,
}

#[pymethods]
impl RoutingResult {
    fn __repr__(&self) -> String {
        format!("RoutingResult(paths=<{} paths>)", self.paths.len())
    }
}

/// Rectilinear edge router.
///
/// Takes a list of rectangular obstacles and routes edges between ports
/// on those obstacles, producing orthogonal (axis-aligned) paths with
/// guaranteed minimum edge separation.
#[pyclass]
struct Router {
    obstacles: Vec<Obstacle>,
    edges: Vec<(Port, Port)>,
    padding: f64,
    edge_separation: f64,
    bend_penalty: f64,
}

/// Default padding around obstacles (pixels).
const DEFAULT_PADDING: f64 = 4.0;
/// Default separation between parallel edges.
const DEFAULT_EDGE_SEPARATION: f64 = 8.0;
/// Default bend penalty as a percentage of source-target Manhattan distance.
const DEFAULT_BEND_PENALTY: f64 = 4.0;

#[pymethods]
impl Router {
    #[new]
    #[pyo3(signature = (obstacles, *, padding=DEFAULT_PADDING, edge_separation=DEFAULT_EDGE_SEPARATION, bend_penalty=DEFAULT_BEND_PENALTY))]
    fn new(
        obstacles: Vec<Obstacle>,
        padding: f64,
        edge_separation: f64,
        bend_penalty: f64,
    ) -> Self {
        let obstacles: Vec<Obstacle> = obstacles
            .into_iter()
            .enumerate()
            .map(|(i, mut o)| {
                o.index = Some(i);
                o
            })
            .collect();

        Self {
            obstacles,
            edges: Vec::new(),
            padding,
            edge_separation,
            bend_penalty,
        }
    }

    /// Add an edge to be routed between two ports.
    fn add_edge(&mut self, source: Port, target: Port) {
        self.edges.push((source, target));
    }

    /// Run the router and return a `RoutingResult` containing all paths.
    fn route(&self) -> RoutingResult {
        let shapes: Vec<Shape> = self
            .obstacles
            .iter()
            .map(|o| Shape::rectangle(o.x, o.y, o.width, o.height))
            .collect();

        let mut router = RectilinearEdgeRouter::new(&shapes)
            .padding(self.padding)
            .edge_separation(self.edge_separation)
            .bend_penalty_as_percentage(self.bend_penalty);

        for (src, tgt) in &self.edges {
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(src.obstacle_index, Point::new(src.x, src.y)),
                FloatingPort::new(tgt.obstacle_index, Point::new(tgt.x, tgt.y)),
            ));
        }

        let result = router.run();

        let paths = result
            .edges
            .iter()
            .map(|edge| {
                let points = edge.points.iter().map(|p| (p.x(), p.y())).collect();
                Path { points }
            })
            .collect();

        RoutingResult { paths }
    }

    fn __repr__(&self) -> String {
        format!(
            "Router(obstacles={}, edges={}, padding={}, edge_separation={})",
            self.obstacles.len(),
            self.edges.len(),
            self.padding,
            self.edge_separation,
        )
    }
}

/// Format a list of point tuples for display.
fn format_points(points: &[(f64, f64)]) -> String {
    let inner: Vec<String> = points.iter().map(|(x, y)| format!("({x}, {y})")).collect();
    format!("[{}]", inner.join(", "))
}

/// The msagl Python module.
#[pymodule]
fn msagl(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Obstacle>()?;
    m.add_class::<Port>()?;
    m.add_class::<Path>()?;
    m.add_class::<RoutingResult>()?;
    m.add_class::<Router>()?;
    Ok(())
}
