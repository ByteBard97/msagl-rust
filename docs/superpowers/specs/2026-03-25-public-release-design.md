# msagl-rust Public Release Design

## Goal

Publish msagl-rust as a polished open-source library with Rust + WASM + Python bindings, benchmarks, and clean presentation. Portfolio-quality, ready for HN/Reddit.

## Architecture

```
msagl-rust/                       <- Cargo workspace root
  Cargo.toml                      <- workspace definition
  src/                            <- core library (pure Rust, no WASM deps)
  crates/
    msagl-wasm/
      Cargo.toml                  <- depends on msagl-rust, adds wasm-bindgen + serde
      src/lib.rs                  <- typed JS/TS API via serde-wasm-bindgen + tsify
      pkg-web/                    <- wasm-pack output (gitignored)
    msagl-python/
      Cargo.toml                  <- depends on msagl-rust, adds pyo3
      src/lib.rs                  <- native Python classes
      msagl.pyi                   <- type stubs
      pyproject.toml              <- maturin config
  benches/
    routing.rs                    <- criterion benchmarks
  .github/
    workflows/
      ci.yml                      <- test + lint + fmt + WASM + Python
      release.yml                 <- publish to crates.io + npm + PyPI
  examples/                       <- runnable examples
```

## Workstream 1: Complete the Port

Prerequisite for everything else. Handled by separate agent.

- Finish faithful port remediation (PRD items)
- Port 242 routing tests from RectilinearTests.cs
- Fix code review violations (NaN panics, magic numbers, DRY)
- Zero `#[allow(dead_code)]` without justifying comment
- `cargo clippy -- -D warnings` clean

## Workstream 2: WASM Bindings (crates/msagl-wasm)

### Cargo.toml

```toml
[package]
name = "msagl-wasm"
version = "0.1.0"
edition = "2021"
description = "WASM bindings for msagl-rust rectilinear edge router"
license = "MIT"

[lib]
crate-type = ["cdylib", "rlib"]

[dependencies]
msagl-rust = { path = "../.." }
wasm-bindgen = "0.2"
serde = { version = "1", features = ["derive"] }
serde-wasm-bindgen = "0.6"
tsify-next = { version = "0.5", features = ["js"] }
console_error_panic_hook = "0.1"

[dev-dependencies]
wasm-bindgen-test = "0.3"

```

Note: `[profile.release]` must go in the workspace root Cargo.toml, not in member crates (silently ignored otherwise).

Add to workspace root:
```toml
[profile.release]
lto = true
opt-level = "s"
```

### API Design

Use `tsify-next` to auto-generate TypeScript types. Use `serde-wasm-bindgen` to pass native JS objects (not JSON strings).

```rust
use wasm_bindgen::prelude::*;
use serde::{Deserialize, Serialize};
use tsify_next::Tsify;

#[derive(Tsify, Deserialize)]
#[tsify(from_wasm_abi)]
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
}

#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct RoutingOutput {
    pub paths: Vec<PathOutput>,
}

#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct PathOutput {
    pub points: Vec<PointOutput>,
}

#[derive(Tsify, Serialize)]
#[tsify(into_wasm_abi)]
pub struct PointOutput {
    pub x: f64,
    pub y: f64,
}

#[wasm_bindgen]
pub fn route_edges(input: RoutingInput) -> Result<RoutingOutput, JsError> {
    // typed in, typed out — no JSON strings
    // Convert msagl_rust::Point (OrderedFloat) to plain f64 PointOutput for serialization
}
```

### TypeScript consumer usage

```typescript
import init, { route_edges } from 'msagl-wasm';

await init();

const result = route_edges({
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
});
// result.paths[0].points = [{x: 50, y: 25}, ..., {x: 350, y: 25}]
```

### Build commands

```bash
cd crates/msagl-wasm && wasm-pack build --target bundler --out-dir pkg-web
cd crates/msagl-wasm && wasm-pack test --headless --chrome
```

## Workstream 3: Python Bindings (crates/msagl-python)

### Cargo.toml

```toml
[package]
name = "msagl-python"
version = "0.1.0"
edition = "2021"
description = "Python bindings for msagl-rust rectilinear edge router"
license = "MIT"

[lib]
name = "msagl"
crate-type = ["cdylib"]

[dependencies]
msagl-rust = { path = "../.." }
pyo3 = { version = "0.23", features = ["extension-module"] }
```

### pyproject.toml

```toml
[build-system]
requires = ["maturin>=1.5,<2"]
build-backend = "maturin"

[project]
name = "msagl"
version = "0.1.0"
description = "Rectilinear edge router for orthogonal graph layouts"
license = { text = "MIT" }
requires-python = ">=3.9"
classifiers = [
    "Programming Language :: Rust",
    "Programming Language :: Python :: Implementation :: CPython",
]

[tool.maturin]
features = ["pyo3/extension-module"]
```

### API Design

Native Python classes, not JSON.

```python
import msagl

obstacles = [
    msagl.Obstacle(x=0, y=0, width=100, height=50),
    msagl.Obstacle(x=300, y=0, width=100, height=50),
]

router = msagl.Router(obstacles, padding=4.0, edge_separation=8.0)
router.add_edge(
    source=obstacles[0].port(x=50, y=25),
    target=obstacles[1].port(x=350, y=25),
)
result = router.route()

for path in result.paths:
    print(path.points)  # [(50.0, 25.0), ..., (350.0, 25.0)]
```

### Type stubs (msagl.pyi)

```python
from typing import List, Tuple

class Obstacle:
    x: float
    y: float
    width: float
    height: float
    def __init__(self, x: float, y: float, width: float, height: float) -> None: ...

class Port:
    """A port on an obstacle. Created via Obstacle.port(x, y) — coordinates are absolute canvas position."""
    obstacle: int
    x: float
    y: float

class Path:
    points: List[Tuple[float, float]]

class RoutingResult:
    paths: List[Path]

class Router:
    def __init__(self, obstacles: List[Obstacle], *, padding: float = 4.0,
                 edge_separation: float = 8.0, bend_penalty: float = 4.0) -> None: ...
    def add_edge(self, source: Port, target: Port) -> None: ...
    def route(self) -> RoutingResult: ...

class Obstacle:
    # ... existing fields ...
    def port(self, x: float, y: float) -> Port:
        """Create a port at absolute canvas coordinates (x, y) on this obstacle."""
        ...
```

### Build commands

```bash
cd crates/msagl-python && maturin develop  # local install
cd crates/msagl-python && maturin build --release  # wheel
cd crates/msagl-python && pytest  # tests
```

## Workstream 4: Benchmarks

### Setup

- `benches/routing.rs` using `criterion`
- Three scales: small (5 obstacles / 4 edges), medium (50 / 80), large (200 / 500)
- Same scenarios run through: native Rust, WASM (Node), Python, MSAGL-JS (TypeScript)
- Run on M4 MacBook (primary) and Linux (secondary column)

### Methodology

- **Separate init from routing**: WASM `init()` and Python import are one-time costs — measure routing calls only
- **Correctness assertion**: every benchmark run verifies no fallback paths (source→target only) were produced
- **Confidence intervals**: report `X ns ± Y ns` from criterion, not bare numbers
- **Reproducibility**: `scripts/run-benchmarks.sh` regenerates all numbers and the README chart
- **No cherry-picking**: use the same scenarios for all four targets

### Presentation

- Table in README with times and confidence intervals
- Bar chart SVG committed to repo (generated by benchmark script)
- Benchmark scenarios as example JSON files for reproducibility

## Workstream 5: Documentation

### README rewrite

1. One-line description + badges (CI, crates.io, docs.rs, npm, PyPI)
2. Feature bullets
3. Quickstart for Rust, TypeScript, Python — three code blocks, same problem
4. Benchmark table with chart
5. Architecture overview + pipeline stages table
6. Algorithm description with paper citations
7. Attribution to Microsoft/MSAGL

### Other docs

- `CONTRIBUTING.md` — how to build, test, submit PRs
- `CHANGELOG.md` — initial release entry
- API doc comments on every public type and function (`cargo doc` must be clean)
- `examples/` directory with 2-3 runnable Rust examples

## Workstream 6: CI/CD

### ci.yml

Runs on every PR and push to main:
- Matrix: Rust stable, Ubuntu + macOS + Windows
- `cargo test --all`
- `cargo clippy --all -- -D warnings`
- `cargo fmt --check`
- `wasm-pack test --headless --firefox` (Ubuntu only, no extra driver setup needed)
- `maturin build` + `pytest` (Ubuntu only)

### release.yml

Triggered by git tag `v*`:
- `cargo publish` (crates.io)
- `wasm-pack publish` (npm)
- `maturin publish` (PyPI)

## Workstream 7: Repo Hygiene (Last)

### Remove before publishing

- `CLAUDE.md`
- `docs/PRD-faithful-port-remediation.md`
- `docs/solver-failure-triage.md`
- `docs/routing-test-failures.md`
- `docs/code-review-2026-03-25.md`
- `docs/superpowers/` (entire directory, including this spec)
- All `Co-Authored-By: Claude` references

### Git history rewrite

- Squash feature branch into clean, logical commits on main
- Remove co-author tags
- Clean commit messages that tell the library's story

### Add

- `.github/ISSUE_TEMPLATE/` (bug report + feature request)
- `rustfmt.toml` (if needed)

## Order of Operations

1. **Port completion** (ongoing, separate agent) — prerequisite
2. **WASM + Python + CI/CD** — can parallelize now, independent of port
3. **Benchmarks** — need bindings done first
4. **Documentation** — can start alongside bindings, finish after benchmarks
5. **Repo hygiene** — absolute last step before publishing

## What NOT to Do

- Don't add `serde` to the core crate — keep it dependency-light
- Don't use JSON string APIs for WASM or Python — use typed bindings
- Don't cherry-pick benchmark scenarios — use realistic workloads
- Don't publish with any `#[allow(dead_code)]` that isn't justified
- Don't leave internal working documents in the public repo
