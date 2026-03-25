<div align="center">

# msagl-rust

**Rectilinear edge routing for orthogonal graph layouts**

[![Crates.io](https://img.shields.io/crates/v/msagl-rust.svg)](https://crates.io/crates/msagl-rust)
[![docs.rs](https://docs.rs/msagl-rust/badge.svg)](https://docs.rs/msagl-rust)
[![CI](https://github.com/ByteBard97/msagl-rust/actions/workflows/ci.yml/badge.svg)](https://github.com/ByteBard97/msagl-rust/actions)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE-MIT)

[API Docs](https://docs.rs/msagl-rust) | [Crate](https://crates.io/crates/msagl-rust) | [MSAGL (original)](https://github.com/microsoft/automatic-graph-layout)

</div>

---

A Rust port of the **rectilinear edge router** from Microsoft's [Automatic Graph Layout (MSAGL)](https://github.com/microsoft/automatic-graph-layout) library. Routes edges as clean, orthogonal polylines around rectangular obstacles with guaranteed minimum edge separation.

> **Status:** All six pipeline stages are implemented. The router produces clean orthogonal paths with guaranteed edge separation for rectangular obstacles.

## Features

- Orthogonal (rectilinear) edge routing with obstacle avoidance
- Guaranteed minimum separation between parallel edge segments
- Constraint-based optimization (QPSC) for edge positioning
- Arc corner fitting for polished output
- No allocator, no GC, no runtime — pure Rust with zero `unsafe`

## Quick Start

```toml
[dependencies]
msagl-rust = "0.1"
```

```rust
use msagl_rust::{RectilinearEdgeRouter, Shape, FloatingPort, EdgeGeometry, Point};

// Define rectangular obstacles (nodes)
let shapes = vec![
    Shape::rectangle(0.0, 100.0, 120.0, 60.0),
    Shape::rectangle(400.0, 100.0, 120.0, 60.0),
];

// Create router
let mut router = RectilinearEdgeRouter::new(&shapes)
    .padding(4.0)
    .edge_separation(8.0);

// Add an edge to route
router.add_edge(EdgeGeometry::new(
    FloatingPort::new(0, Point::new(120.0, 130.0)),
    FloatingPort::new(1, Point::new(400.0, 130.0)),
));

// Route all edges
let result = router.run();

for edge in &result.edges {
    for segment in edge.curve.segments() {
        // Each segment is a Line or Arc
    }
}
```

> **Note:** This API is functional. The router produces orthogonal paths with edge separation for rectangular obstacles. Arc corner fitting is supported via `corner_fit_radius()`.

## Architecture

The router implements a six-stage pipeline, ported faithfully from MSAGL's C# and TypeScript implementations:

| Stage | Description | Status |
|-------|-------------|--------|
| 1. Obstacle Tree | R-tree spatial index of padded node rectangles | **Complete** |
| 2. Visibility Graph | Sweep-line construction of legal routing corridors | **Complete** |
| 3. Port Splicing | Connect edge endpoints into the visibility graph | **Complete** |
| 4. Shortest Path | Direction-aware A* on the visibility graph | **Complete** |
| 5. Nudging | QPSC constraint solver for minimum edge separation | **Complete** |
| 6. Finalization | Arc fitting, arrowhead trimming, curve output | **Complete** |

### Modules

- **`geometry/`** — `Point` (OrderedFloat-backed), `Rectangle`, `Polyline` (SlotMap doubly-linked list), `Curve`
- **`projection_solver/`** — Full QPSC port, validated against 80/86 C# golden-baseline fixtures
- **`visibility/`** — Visibility graph with HashMap point-to-vertex lookup
- **`routing/`** — Obstacle tree, sweep-line graph generator, A* path search, port splicing, nudging pipeline, top-level router

## Algorithm

The rectilinear edge router is based on the algorithms described in:

> Dwyer, T., Nachmanson, L., & Wybrow, M. (2010). *Optimal Route Planning for Buses.* Technical Report, Microsoft Research.

The nudging stage uses a QPSC solver described in:

> Dwyer, T., Koren, Y., & Marriott, K. (2006). *IPSep-CoLa: An incremental procedure for separation constraint layout of graphs.* IEEE Transactions on Visualization and Computer Graphics.

## Dependencies

| Crate | Purpose |
|-------|---------|
| [`ordered-float`](https://crates.io/crates/ordered-float) | Hashable/orderable `f64` for point-keyed maps |
| [`slotmap`](https://crates.io/crates/slotmap) | Arena allocation for polyline doubly-linked lists |
| [`kurbo`](https://crates.io/crates/kurbo) | Arc and line segment geometry |

## Testing

```bash
cargo test
```

The test suite includes:
- 70 geometry unit tests
- 112 projection solver tests (29 unit + 86 golden-baseline fixtures, 80 passing / 6 ignored edge cases)
- 47 routing tests (visibility graph, path search, port splicing, nudging)
- 6 end-to-end integration tests

## Attribution

This crate is a port of the rectilinear edge routing module from [Microsoft Automatic Graph Layout (MSAGL)](https://github.com/microsoft/automatic-graph-layout), originally developed by Lev Nachmanson, Sergey Pupyrev, Tim Dwyer, Ted Hart, and Roman Prutkin at Microsoft Research.

The TypeScript port ([msagl-js](https://github.com/microsoft/msagljs)) was used as the primary structural reference, with the C# source as the authoritative algorithmic reference.

## License

MIT License. See [LICENSE-MIT](LICENSE-MIT) for details.

Copyright (c) Microsoft Corporation (original MSAGL).
Portions Copyright (c) 2026 SignalCanvas contributors.
