//! WASM bindings for msagl-rust rectilinear edge router.
//!
//! Provides a typed JavaScript/TypeScript API using `tsify-next` and
//! `serde-wasm-bindgen`. Input and output are native JS objects — no JSON
//! string serialization required.

use wasm_bindgen::prelude::*;
use serde::{Deserialize, Serialize};
use tsify_next::Tsify;

use msagl_rust::{
    EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape,
};

// ---------------------------------------------------------------------------
// Default values for optional RoutingInput fields
// ---------------------------------------------------------------------------

fn default_padding() -> f64 {
    4.0
}

fn default_edge_separation() -> f64 {
    8.0
}

fn default_bend_penalty() -> f64 {
    4.0
}

// ---------------------------------------------------------------------------
// Input types (from_wasm_abi — JS passes these IN)
// ---------------------------------------------------------------------------

/// Top-level routing request passed from JavaScript.
#[derive(Tsify, Deserialize)]
#[tsify(from_wasm_abi)]
#[serde(rename_all = "camelCase")]
pub struct RoutingInput {
    /// Rectangular obstacles that edges must route around.
    pub obstacles: Vec<ObstacleInput>,
    /// Edges to route between obstacle ports.
    pub edges: Vec<EdgeInput>,
    /// Padding around each obstacle (default: 4.0).
    #[serde(default = "default_padding")]
    pub padding: f64,
    /// Minimum separation between parallel edge segments (default: 8.0).
    #[serde(default = "default_edge_separation")]
    pub edge_separation: f64,
    /// Bend penalty as a percentage of source-target Manhattan distance (default: 4.0).
    #[serde(default = "default_bend_penalty")]
    pub bend_penalty: f64,
}

/// A rectangular obstacle. (x, y) is the left-bottom corner.
#[derive(Tsify, Deserialize)]
#[tsify(from_wasm_abi)]
pub struct ObstacleInput {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

/// An edge to route, defined by source/target points and their obstacle indices.
#[derive(Tsify, Deserialize)]
#[tsify(from_wasm_abi)]
#[serde(rename_all = "camelCase")]
pub struct EdgeInput {
    /// Port location for the source end.
    pub source: PointInput,
    /// Port location for the target end.
    pub target: PointInput,
    /// Index into the `obstacles` array for the source port.
    pub source_obstacle: usize,
    /// Index into the `obstacles` array for the target port.
    pub target_obstacle: usize,
}

/// A 2D point (input side).
#[derive(Tsify, Deserialize)]
#[tsify(from_wasm_abi)]
pub struct PointInput {
    pub x: f64,
    pub y: f64,
}

// ---------------------------------------------------------------------------
// Output types (into_wasm_abi — returned TO JS)
// ---------------------------------------------------------------------------

/// Result of routing all edges.
#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct RoutingOutput {
    /// One path per input edge, in the same order.
    pub paths: Vec<PathOutput>,
}

/// A single routed path (sequence of rectilinear waypoints).
#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct PathOutput {
    /// Waypoints from source through bends to target.
    pub points: Vec<PointOutput>,
}

/// A 2D point (output side). Uses plain `f64` — the core library's
/// `OrderedFloat` wrapper is stripped here for JS compatibility.
#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct PointOutput {
    pub x: f64,
    pub y: f64,
}

// ---------------------------------------------------------------------------
// WASM entry points
// ---------------------------------------------------------------------------

/// Initialise the WASM module. Call once before `route_edges`.
///
/// Sets up the panic hook so Rust panics appear as readable JS console errors
/// instead of opaque "unreachable" messages.
#[wasm_bindgen]
pub fn init() {
    console_error_panic_hook::set_once();
}

/// Route edges around rectangular obstacles.
///
/// Accepts a typed `RoutingInput` object and returns a `RoutingOutput` with
/// one path per input edge.
#[wasm_bindgen]
pub fn route_edges(input: RoutingInput) -> Result<RoutingOutput, JsError> {
    // 1. Build shapes from obstacle descriptions
    let shapes: Vec<Shape> = input
        .obstacles
        .iter()
        .map(|o| Shape::rectangle(o.x, o.y, o.width, o.height))
        .collect();

    // 2. Configure the router
    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(input.padding)
        .edge_separation(input.edge_separation)
        .bend_penalty_as_percentage(input.bend_penalty);

    // 3. Add edges
    for edge in &input.edges {
        if edge.source_obstacle >= shapes.len() {
            return Err(JsError::new(&format!(
                "sourceObstacle index {} out of range ({} obstacles)",
                edge.source_obstacle,
                shapes.len()
            )));
        }
        if edge.target_obstacle >= shapes.len() {
            return Err(JsError::new(&format!(
                "targetObstacle index {} out of range ({} obstacles)",
                edge.target_obstacle,
                shapes.len()
            )));
        }

        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(
                edge.source_obstacle,
                Point::new(edge.source.x, edge.source.y),
            ),
            FloatingPort::new(
                edge.target_obstacle,
                Point::new(edge.target.x, edge.target.y),
            ),
        ));
    }

    // 4. Run the router
    let result = router.run();

    // 5. Convert output — strip OrderedFloat, return plain f64
    let paths = result
        .edges
        .into_iter()
        .map(|routed| PathOutput {
            points: routed
                .points
                .iter()
                .map(|p| PointOutput {
                    x: p.x(),
                    y: p.y(),
                })
                .collect(),
        })
        .collect();

    Ok(RoutingOutput { paths })
}
