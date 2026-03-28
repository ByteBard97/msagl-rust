//! WASM bindings for msagl-rust rectilinear edge router.
//!
//! Provides a typed JavaScript/TypeScript API using `tsify-next` and
//! `serde-wasm-bindgen`. Input and output are native JS objects — no JSON
//! string serialization required.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use tsify_next::Tsify;
use wasm_bindgen::prelude::*;

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

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
#[serde(rename_all = "camelCase")]
pub struct PathOutput {
    /// Waypoints from source through bends to target.
    pub points: Vec<PointOutput>,
    /// True when the path search failed and this is a straight-line fallback.
    pub is_fallback: bool,
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
                .map(|p| PointOutput { x: p.x(), y: p.y() })
                .collect(),
            is_fallback: routed.is_fallback,
        })
        .collect();

    Ok(RoutingOutput { paths })
}

// ---------------------------------------------------------------------------
// Stateful Router struct
// ---------------------------------------------------------------------------

/// Serialisable path returned by `Router::get_paths`.
#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct StatefulPathOutput {
    /// Edge handle that this path belongs to.
    edge_id: usize,
    /// Waypoints (source → bends → target).
    points: Vec<PointOutput>,
    /// True when routing failed and this is a straight-line fallback.
    is_fallback: bool,
}

/// Stored bounds for one obstacle — used to compute port centers on demand.
#[derive(Clone, Copy)]
struct ObstacleBounds {
    x: f64,
    y: f64,
    w: f64,
    h: f64,
}

impl ObstacleBounds {
    fn center(self) -> Point {
        Point::new(self.x + self.w / 2.0, self.y + self.h / 2.0)
    }
}

/// Stored edge — src/tgt obstacle handles so we can look up current centers.
struct StoredEdge {
    src_obstacle_id: usize,
    tgt_obstacle_id: usize,
}

/// Long-lived stateful router exposed to JavaScript.
///
/// Maintains a persistent `RectilinearEdgeRouter` and re-routes only when
/// `route()` is called, rather than rebuilding everything on every call.
///
/// # Handle semantics
/// `add_obstacle` and `add_edge` return opaque integer handles.  Handles are
/// monotonically increasing and are never reused.  Internally the struct
/// maintains `id_to_slot` which maps a live handle to its current slot index
/// inside the `RectilinearEdgeRouter` obstacle list.  Slot indices shift
/// whenever an obstacle is removed, so `id_to_slot` is updated after every
/// removal.
#[wasm_bindgen]
pub struct Router {
    inner: RectilinearEdgeRouter,
    /// Bounds keyed by obstacle handle — needed to compute port centers.
    obstacle_bounds: HashMap<usize, ObstacleBounds>,
    /// Maps obstacle handle → slot index in `inner`'s obstacle list.
    id_to_slot: HashMap<usize, usize>,
    next_obstacle_id: usize,
    /// Maps edge handle → slot index in `inner`'s edge list.
    edge_id_to_slot: HashMap<usize, usize>,
    /// Stored edges (src/tgt handles) keyed by edge handle.
    stored_edges: HashMap<usize, StoredEdge>,
    next_edge_id: usize,
    /// Most recent routing result — populated by `route()`, read by `get_paths()`.
    last_paths: Vec<StatefulPathOutput>,
}

#[wasm_bindgen]
impl Router {
    /// Create a new stateful router with the given padding and edge separation.
    #[wasm_bindgen(constructor)]
    pub fn new(padding: f64, edge_separation: f64) -> Router {
        // Construct with an empty obstacle list; obstacles are added incrementally.
        let inner = RectilinearEdgeRouter::new(&[])
            .padding(padding)
            .edge_separation(edge_separation);
        Router {
            inner,
            obstacle_bounds: HashMap::new(),
            id_to_slot: HashMap::new(),
            next_obstacle_id: 0,
            edge_id_to_slot: HashMap::new(),
            stored_edges: HashMap::new(),
            next_edge_id: 0,
            last_paths: Vec::new(),
        }
    }

    /// Add a rectangular obstacle. Returns an opaque integer handle.
    ///
    /// The handle can be used later with `move_obstacle`, `remove_obstacle`,
    /// and `add_edge`.
    pub fn add_obstacle(&mut self, x: f64, y: f64, width: f64, height: f64) -> usize {
        let id = self.next_obstacle_id;
        self.next_obstacle_id += 1;

        // The slot is at the end of the current obstacle list.
        let slot = self.id_to_slot.len(); // == current obstacle count before adding
        self.id_to_slot.insert(id, slot);
        self.obstacle_bounds.insert(id, ObstacleBounds { x, y, w: width, h: height });

        self.inner.add_obstacle(Shape::rectangle(x, y, width, height));
        id
    }

    /// Move / resize an obstacle identified by `id`.
    ///
    /// The visibility graph is rebuilt immediately by the underlying router.
    pub fn move_obstacle(&mut self, id: usize, x: f64, y: f64, width: f64, height: f64) {
        let slot = match self.id_to_slot.get(&id).copied() {
            Some(s) => s,
            None => return, // unknown handle — silently ignore
        };
        self.obstacle_bounds.insert(id, ObstacleBounds { x, y, w: width, h: height });
        self.inner.update_obstacle(slot, Shape::rectangle(x, y, width, height));
    }

    /// Remove an obstacle identified by `id`.
    ///
    /// After removal, `id_to_slot` is updated for all slots whose indices were
    /// shifted down by the removal, preserving correctness for future calls.
    pub fn remove_obstacle(&mut self, id: usize) {
        let slot = match self.id_to_slot.remove(&id) {
            Some(s) => s,
            None => return, // unknown handle — silently ignore
        };
        self.obstacle_bounds.remove(&id);
        self.inner.remove_obstacle(slot);

        // Every obstacle whose slot was greater than the removed slot has been
        // shifted down by 1 in the inner list — update id_to_slot accordingly.
        for s in self.id_to_slot.values_mut() {
            if *s > slot {
                *s -= 1;
            }
        }
    }

    /// Add a directed edge between two obstacle handles. Returns an edge handle.
    ///
    /// Port locations are computed as the centres of the source and target
    /// obstacles at the time `add_edge` is called.  If either obstacle handle
    /// is unknown the call is a no-op and returns `usize::MAX`.
    pub fn add_edge(&mut self, src_obstacle_id: usize, tgt_obstacle_id: usize) -> usize {
        let src_bounds = match self.obstacle_bounds.get(&src_obstacle_id).copied() {
            Some(b) => b,
            None => return usize::MAX,
        };
        let tgt_bounds = match self.obstacle_bounds.get(&tgt_obstacle_id).copied() {
            Some(b) => b,
            None => return usize::MAX,
        };
        let src_slot = match self.id_to_slot.get(&src_obstacle_id).copied() {
            Some(s) => s,
            None => return usize::MAX,
        };
        let tgt_slot = match self.id_to_slot.get(&tgt_obstacle_id).copied() {
            Some(s) => s,
            None => return usize::MAX,
        };

        let id = self.next_edge_id;
        self.next_edge_id += 1;

        // Edge slot is the current edge count before adding.
        let slot = self.inner.edge_count();
        self.edge_id_to_slot.insert(id, slot);
        self.stored_edges.insert(id, StoredEdge { src_obstacle_id, tgt_obstacle_id });

        self.inner.add_edge(EdgeGeometry::new(
            FloatingPort::new(src_slot, src_bounds.center()),
            FloatingPort::new(tgt_slot, tgt_bounds.center()),
        ));
        id
    }

    /// Remove an edge by handle.
    pub fn remove_edge(&mut self, id: usize) {
        let slot = match self.edge_id_to_slot.remove(&id) {
            Some(s) => s,
            None => return,
        };
        self.stored_edges.remove(&id);
        self.inner.remove_edge_geometry(slot);

        // Update slots for all edges that were after the removed one.
        for s in self.edge_id_to_slot.values_mut() {
            if *s > slot {
                *s -= 1;
            }
        }
    }

    /// Route all registered edges.
    ///
    /// Stores the result internally. Call `get_paths()` to retrieve it.
    /// Keeping `route` and `get_paths` separate allows the caller to trigger
    /// routing once and inspect results without re-running.
    pub fn route(&mut self) {
        // Re-add all edges with fresh port locations (obstacle centres may have
        // moved since the edge was first registered).
        //
        // We clear all edges from `inner`, rebuild them from `stored_edges`, and
        // reset `edge_id_to_slot` so the slot mapping stays consistent.
        //
        // Collect the data we need first to avoid conflicting borrows.
        let edge_data: Vec<(usize, usize, usize)> = {
            // Sort by current slot so rebuild order matches the original insertion order.
            let mut entries: Vec<(usize, usize)> = self
                .edge_id_to_slot
                .iter()
                .map(|(&eid, &slot)| (eid, slot))
                .collect();
            entries.sort_unstable_by_key(|(_, slot)| *slot);
            entries
                .into_iter()
                .filter_map(|(eid, _)| {
                    let se = self.stored_edges.get(&eid)?;
                    Some((eid, se.src_obstacle_id, se.tgt_obstacle_id))
                })
                .collect()
        };

        // Clear existing edges from the inner router.
        // `remove_edge_geometry` shifts slots; easiest to drain from the end.
        let edge_count = self.inner.edge_count();
        for i in (0..edge_count).rev() {
            self.inner.remove_edge_geometry(i);
        }
        self.edge_id_to_slot.clear();

        // Re-add edges with current port centres.
        for (eid, src_id, tgt_id) in &edge_data {
            let src_bounds = match self.obstacle_bounds.get(src_id).copied() {
                Some(b) => b,
                None => continue,
            };
            let tgt_bounds = match self.obstacle_bounds.get(tgt_id).copied() {
                Some(b) => b,
                None => continue,
            };
            let src_slot = match self.id_to_slot.get(src_id).copied() {
                Some(s) => s,
                None => continue,
            };
            let tgt_slot = match self.id_to_slot.get(tgt_id).copied() {
                Some(s) => s,
                None => continue,
            };

            let slot = self.inner.edge_count();
            self.edge_id_to_slot.insert(*eid, slot);

            self.inner.add_edge(EdgeGeometry::new(
                FloatingPort::new(src_slot, src_bounds.center()),
                FloatingPort::new(tgt_slot, tgt_bounds.center()),
            ));
        }

        let result = self.inner.run();

        // Map each result back to the edge handle using the slot order.
        let edge_handles_in_order: Vec<usize> = {
            let mut v: Vec<(usize, usize)> = self
                .edge_id_to_slot
                .iter()
                .map(|(&eid, &slot)| (slot, eid))
                .collect();
            v.sort_unstable_by_key(|(slot, _)| *slot);
            v.into_iter().map(|(_, eid)| eid).collect()
        };

        self.last_paths = result
            .edges
            .into_iter()
            .enumerate()
            .map(|(i, routed)| StatefulPathOutput {
                edge_id: edge_handles_in_order.get(i).copied().unwrap_or(usize::MAX),
                points: routed
                    .points
                    .iter()
                    .map(|p| PointOutput { x: p.x(), y: p.y() })
                    .collect(),
                is_fallback: routed.is_fallback,
            })
            .collect();
    }

    /// Return the routed paths from the most recent `route()` call as a JS value.
    ///
    /// Returns an array of `{ edgeId, points, isFallback }` objects.
    pub fn get_paths(&self) -> JsValue {
        serde_wasm_bindgen::to_value(&self.last_paths).unwrap_or(JsValue::NULL)
    }
}
