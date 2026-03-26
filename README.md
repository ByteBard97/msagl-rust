<div align="center">

# msagl-rust

**Rectilinear edge routing for orthogonal graph layouts**

[![CI](https://github.com/ByteBard97/msagl-rust/actions/workflows/ci.yml/badge.svg)](https://github.com/ByteBard97/msagl-rust/actions)
[![Crates.io](https://img.shields.io/crates/v/msagl-rust.svg)](https://crates.io/crates/msagl-rust)
[![docs.rs](https://docs.rs/msagl-rust/badge.svg)](https://docs.rs/msagl-rust)
[![npm](https://img.shields.io/npm/v/msagl-wasm.svg)](https://www.npmjs.com/package/msagl-wasm)
[![PyPI](https://img.shields.io/pypi/v/msagl.svg)](https://pypi.org/project/msagl/)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE-MIT)

[API Docs](https://docs.rs/msagl-rust) | [Crate](https://crates.io/crates/msagl-rust) | [npm](https://www.npmjs.com/package/msagl-wasm) | [PyPI](https://pypi.org/project/msagl/) | [MSAGL (original)](https://github.com/microsoft/automatic-graph-layout)

</div>

---

A Rust port of the **rectilinear edge router** from Microsoft's [Automatic Graph Layout (MSAGL)](https://github.com/microsoft/automatic-graph-layout) library. Routes edges as clean, orthogonal polylines around rectangular obstacles with guaranteed minimum edge separation.

Works natively in Rust, in the browser via WASM, and in Python via PyO3.

## Features

- Orthogonal (rectilinear) edge routing with obstacle avoidance
- Guaranteed minimum separation between parallel edge segments
- Constraint-based optimization (QPSC) for edge positioning
- Arc corner fitting for polished output
- WASM bindings with full TypeScript type generation
- Python bindings with native classes (not JSON)
- No allocator, no GC, no runtime -- pure Rust with zero `unsafe`

## Quick Start -- Rust

```toml
[dependencies]
msagl-rust = "0.1"
```

```rust
use msagl_rust::{RectilinearEdgeRouter, Shape, FloatingPort, EdgeGeometry, Point};

// Define rectangular obstacles
let shapes = vec![
    Shape::rectangle(0.0, 0.0, 100.0, 50.0),
    Shape::rectangle(300.0, 0.0, 100.0, 50.0),
];

// Create router with custom settings
let mut router = RectilinearEdgeRouter::new(&shapes)
    .padding(4.0)
    .edge_separation(8.0);

// Add an edge connecting two shapes
router.add_edge(EdgeGeometry::new(
    FloatingPort::new(0, Point::new(104.0, 25.0)),
    FloatingPort::new(1, Point::new(296.0, 25.0)),
));

// Route all edges
let result = router.run();

for edge in &result.edges {
    for pt in &edge.points {
        println!("({}, {})", pt.x, pt.y);
    }
}
```

## Quick Start -- TypeScript (WASM)

```bash
npm install msagl-wasm
```

```typescript
import init, { route_edges } from 'msagl-wasm';

await init();

const result = route_edges({
  obstacles: [
    { x: 0, y: 0, width: 100, height: 50 },
    { x: 300, y: 0, width: 100, height: 50 },
  ],
  edges: [{
    source: { x: 104, y: 25 },
    target: { x: 296, y: 25 },
    sourceObstacle: 0,
    targetObstacle: 1,
  }],
  padding: 4.0,
  edgeSeparation: 8.0,
});

// result.paths[0].points = [{ x: 104, y: 25 }, ..., { x: 296, y: 25 }]
```

## Quick Start -- Python

```bash
pip install msagl
```

```python
import msagl

obstacles = [
    msagl.Obstacle(x=0, y=0, width=100, height=50),
    msagl.Obstacle(x=300, y=0, width=100, height=50),
]

router = msagl.Router(obstacles, padding=4.0, edge_separation=8.0)
router.add_edge(
    source=obstacles[0].port(x=104, y=25),
    target=obstacles[1].port(x=296, y=25),
)
result = router.route()

for path in result.paths:
    print(path.points)  # [(104.0, 25.0), ..., (296.0, 25.0)]
```

## Benchmarks

<!-- BENCHMARK TABLE -->

> Run `scripts/run-benchmarks.sh` to generate benchmark results for your hardware.

## Architecture

The router implements a six-stage pipeline, ported faithfully from MSAGL's C# and TypeScript implementations:

| Stage | Description |
|-------|-------------|
| 1. Obstacle Tree | R-tree spatial index of padded node rectangles |
| 2. Visibility Graph | Sweep-line construction of legal routing corridors |
| 3. Port Splicing | Connect edge endpoints into the visibility graph |
| 4. Shortest Path | Direction-aware A* on the visibility graph |
| 5. Nudging | QPSC constraint solver for minimum edge separation |
| 6. Finalization | Arc fitting, arrowhead trimming, curve output |

### Module Map

- **`geometry/`** -- `Point` (OrderedFloat-backed), `Rectangle`, `Polyline` (SlotMap doubly-linked list), `Curve`
- **`projection_solver/`** -- Full QPSC port, validated against 80/86 C# golden-baseline fixtures
- **`visibility/`** -- Visibility graph with HashMap point-to-vertex lookup
- **`routing/`** -- Obstacle tree, sweep-line graph generator, A* path search, port splicing, nudging pipeline, top-level router

## Algorithm

The rectilinear edge router is based on the algorithms described in:

> Wybrow, M., Marriott, K., & Stuckey, P. J. (2010). *Orthogonal Connector Routing.* Journal of Graph Algorithms and Applications, 14(2), 169--199.

This is the primary paper referenced in the MSAGL C# source for both the path search (`SsstRectilinearPath`) and nudging (`Nudger`) stages.

The nudging stage uses a QPSC (Quadratic Programming with Separation Constraints) solver described in:

> Dwyer, T., Koren, Y., & Marriott, K. (2006). *IPSep-CoLa: An incremental procedure for separation constraint layout of graphs.* IEEE Transactions on Visualization and Computer Graphics, 12(5), 821--828.

## Dependencies

| Crate | Purpose |
|-------|---------|
| [`ordered-float`](https://crates.io/crates/ordered-float) | Hashable/orderable `f64` for point-keyed maps |
| [`slotmap`](https://crates.io/crates/slotmap) | Arena allocation for polyline doubly-linked lists |
| [`kurbo`](https://crates.io/crates/kurbo) | Arc and line segment geometry |
| [`rstar`](https://crates.io/crates/rstar) | R-tree spatial indexing for obstacle queries |

## Testing

```bash
cargo test
```

The test suite includes 698+ tests:

- Geometry unit tests (points, rectangles, polylines, curves)
- Projection solver tests (unit + 86 golden-baseline fixtures from the C# test suite)
- Routing tests (visibility graph, path search, port splicing, nudging)
- End-to-end integration tests

## Examples

```bash
cargo run --example basic          # 2 rectangles, 1 edge
cargo run --example multi_edge     # 4 rectangles, 6 edges
cargo run --example custom_params  # custom padding, separation, bend penalty
```

## Attribution

This crate is a port of the rectilinear edge routing module from [Microsoft Automatic Graph Layout (MSAGL)](https://github.com/microsoft/automatic-graph-layout), originally developed by Lev Nachmanson, Sergey Pupyrev, Tim Dwyer, Ted Hart, and Roman Prutkin at Microsoft Research.

The TypeScript port ([msagl-js](https://github.com/microsoft/msagljs)) was used as the primary structural reference, with the C# source as the authoritative algorithmic reference.

Thank you to the MSAGL team for open-sourcing a world-class graph layout library under the MIT license.

## License

MIT License -- same as the original MSAGL. See [LICENSE-MIT](LICENSE-MIT).
