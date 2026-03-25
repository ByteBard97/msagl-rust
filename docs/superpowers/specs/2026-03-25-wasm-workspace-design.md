# msagl-rust WASM + Workspace Design

## Goal

Set up msagl-rust as a Cargo workspace with a thin WASM wrapper crate, so the rectilinear edge router can be used from any browser app via npm, while keeping the core algorithm free of WASM dependencies.

## Architecture

```
msagl-rust/                       ← Cargo workspace root
  Cargo.toml                      ← workspace definition
  src/                            ← core library (pure Rust, no WASM deps)
  crates/
    msagl-wasm/
      Cargo.toml                  ← depends on msagl-rust, adds wasm-bindgen + serde
      src/lib.rs                  ← ~80 lines of glue: JSON in → route → JSON out
      pkg-web/                    ← wasm-pack output for bundlers (gitignored)
      pkg-node/                   ← wasm-pack output for Node (gitignored)
```

## Workspace Cargo.toml

```toml
# msagl-rust/Cargo.toml (root)
[workspace]
members = [".", "crates/msagl-wasm"]

[package]
name = "msagl-rust"
version = "0.1.0"
edition = "2021"
# ... existing fields ...

[dependencies]
ordered-float = "5"
slotmap = "1"
kurbo = "0.13"
rstar = "0.12"

[dev-dependencies]
approx = "0.5"
```

## WASM Crate

```toml
# crates/msagl-wasm/Cargo.toml
[package]
name = "msagl-wasm"
version = "0.1.0"
edition = "2021"

[lib]
crate-type = ["cdylib", "rlib"]

[dependencies]
msagl-rust = { path = "../.." }
wasm-bindgen = "0.2"
serde = { version = "1", features = ["derive"] }
serde_json = "1"
js-sys = "0.3"
console_error_panic_hook = "0.1"

[dev-dependencies]
wasm-bindgen-test = "0.3"

[package.metadata.wasm-pack.profile.release]
wasm-opt = ["-Os"]

[profile.release]
lto = true
opt-level = "s"
```

## WASM API (crates/msagl-wasm/src/lib.rs)

The WASM wrapper exposes a single function that takes JSON input and returns JSON output. This is the simplest possible API — no complex type marshaling, just strings.

```rust
use wasm_bindgen::prelude::*;
use serde::{Deserialize, Serialize};

#[wasm_bindgen(start)]
pub fn init() {
    console_error_panic_hook::set_once();
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RoutingInput {
    pub obstacles: Vec<ObstacleInput>,
    pub edges: Vec<EdgeInput>,
    #[serde(default = "default_padding")]
    pub padding: f64,
    #[serde(default = "default_edge_separation")]
    pub edge_separation: f64,
    #[serde(default = "default_bend_penalty")]
    pub bend_penalty: f64,
    #[serde(default)]
    pub corner_radius: f64,
}

#[derive(Deserialize)]
pub struct ObstacleInput {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

#[derive(Deserialize)]
pub struct EdgeInput {
    pub source: PointInput,
    pub target: PointInput,
    pub source_obstacle: usize,
    pub target_obstacle: usize,
}

#[derive(Deserialize, Serialize)]
pub struct PointInput {
    pub x: f64,
    pub y: f64,
}

#[derive(Serialize)]
pub struct RoutingOutput {
    pub paths: Vec<PathOutput>,
}

#[derive(Serialize)]
pub struct PathOutput {
    pub points: Vec<PointInput>,
}

fn default_padding() -> f64 { 4.0 }
fn default_edge_separation() -> f64 { 8.0 }
fn default_bend_penalty() -> f64 { 4.0 }

#[wasm_bindgen]
pub fn route_edges(input_json: &str) -> String {
    let input: RoutingInput = match serde_json::from_str(input_json) {
        Ok(i) => i,
        Err(e) => return format!(r#"{{"error":"{}"}}"#, e),
    };

    // Build shapes
    let shapes: Vec<msagl_rust::Shape> = input.obstacles.iter()
        .map(|o| msagl_rust::Shape::rectangle(o.x, o.y, o.width, o.height))
        .collect();

    // Build router
    let mut router = msagl_rust::RectilinearEdgeRouter::new(&shapes)
        .padding(input.padding)
        .edge_separation(input.edge_separation)
        .bend_penalty_as_percentage(input.bend_penalty);

    // Add edges
    for edge in &input.edges {
        router.add_edge(msagl_rust::EdgeGeometry::new(
            msagl_rust::FloatingPort::new(
                edge.source_obstacle,
                msagl_rust::Point::new(edge.source.x, edge.source.y),
            ),
            msagl_rust::FloatingPort::new(
                edge.target_obstacle,
                msagl_rust::Point::new(edge.target.x, edge.target.y),
            ),
        ));
    }

    // Route
    let result = router.run();

    // Serialize output
    let output = RoutingOutput {
        paths: result.edges.iter().map(|e| PathOutput {
            points: e.points.iter().map(|p| PointInput { x: p.x(), y: p.y() }).collect(),
        }).collect(),
    };

    serde_json::to_string(&output).unwrap_or_else(|e| format!(r#"{{"error":"{}"}}"#, e))
}
```

## TypeScript Consumer Usage

```typescript
// In SignalCanvasFrontend or any web app
import init, { route_edges } from 'msagl-wasm';

await init(); // load WASM module

const result = JSON.parse(route_edges(JSON.stringify({
  obstacles: [
    { x: 0, y: 0, width: 100, height: 50 },
    { x: 300, y: 0, width: 100, height: 50 },
  ],
  edges: [{
    source: { x: 50, y: 25 },
    target: { x: 350, y: 25 },
    sourceObstacle: 0,
    targetObstacle: 1,
  }],
  padding: 4.0,
  edgeSeparation: 8.0,
})));

// result.paths[0].points = [{x: 50, y: 25}, ..., {x: 350, y: 25}]
```

## Build Commands

```bash
# Build WASM for browser bundlers (Vite, Webpack)
cd crates/msagl-wasm && wasm-pack build --target bundler --out-dir pkg-web

# Build WASM for Node.js
cd crates/msagl-wasm && wasm-pack build --target nodejs --out-dir pkg-node

# Run core Rust tests
cargo test

# Run WASM tests (in browser via headless Chrome)
cd crates/msagl-wasm && wasm-pack test --headless --chrome

# Publish to npm
cd crates/msagl-wasm && wasm-pack publish
```

## Frontend Integration (SignalCanvasFrontend)

Add to `package.json`:
```json
{
  "scripts": {
    "wasm:build": "cd ../msagl-rust/crates/msagl-wasm && wasm-pack build --target bundler --out-dir pkg-web"
  },
  "dependencies": {
    "msagl-wasm": "file:../msagl-rust/crates/msagl-wasm/pkg-web"
  }
}
```

Vite config (already has `vite-plugin-wasm`):
```typescript
import wasm from 'vite-plugin-wasm';
export default defineConfig({
  plugins: [wasm()],
});
```

## Future: Python Wheel

When needed, add `crates/msagl-python/` using PyO3 + maturin. Same pattern — thin wrapper, JSON API. No changes to core crate.

## Dependency Check

All current dependencies are WASM-compatible:
- `ordered-float` — pure Rust, no std issues
- `slotmap` — pure Rust
- `kurbo` — pure Rust, geometry only
- `rstar` — pure Rust, R-tree implementation

No blockers for WASM compilation.

## What NOT to Do

- Don't add `serde` to the core crate — keep it dependency-light
- Don't add `wasm-bindgen` feature flags to the core crate — that's the wrapper's job
- Don't use `wasm-opt = false` — use `wasm-opt = ["-Os"]` for smaller output
- Don't publish the core crate and WASM crate with the same name
