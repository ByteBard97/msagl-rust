# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0] - 2026-03-25

### Added

- Rectilinear (orthogonal) edge routing with obstacle avoidance
- Six-stage routing pipeline: obstacle tree, visibility graph, port splicing, shortest path, nudging, finalization
- Constraint-based optimization (QPSC) for guaranteed minimum edge separation
- Arc corner fitting for polished curve output
- WASM bindings via `wasm-bindgen` + `tsify-next` (`msagl-wasm` crate)
- Python bindings via PyO3 + maturin (`msagl-python` crate)
- Criterion benchmarks for small/medium/large routing scenarios
- 698+ tests across geometry, projection solver, routing, and integration
- Runnable examples in `examples/`

[0.1.0]: https://github.com/ByteBard97/msagl-rust/releases/tag/v0.1.0
