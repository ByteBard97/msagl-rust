# Contributing to msagl-rust

Thanks for your interest in contributing! This document covers the basics.

## Building

```bash
# Build the core library
cargo build

# Run all tests
cargo test

# Lint
cargo clippy -- -D warnings

# Format check
cargo fmt --check
```

## WASM Bindings

```bash
cd crates/msagl-wasm
wasm-pack build --target bundler
```

## Python Bindings

```bash
cd crates/msagl-python
maturin develop
```

## Benchmarks

```bash
cargo bench
# Or generate the full benchmark report:
scripts/run-benchmarks.sh
```

## Submitting a Pull Request

1. Fork the repo and create a feature branch from `main`.
2. Make your changes. Keep commits focused and atomic.
3. Ensure all checks pass before pushing:
   ```bash
   cargo fmt
   cargo clippy -- -D warnings
   cargo test
   ```
4. Open a PR with a clear description of what changed and why.

## Code Style

- Run `cargo fmt` before committing.
- No `unsafe` code without prior discussion.
- No `#[allow(dead_code)]` without a justifying comment.
- Keep files under 500 lines (700 absolute max).
- Use constants instead of magic numbers.

## Reporting Issues

Use the [GitHub issue tracker](https://github.com/ByteBard97/msagl-rust/issues). Bug reports should include:

- Minimal reproduction (obstacle positions, edge endpoints)
- Expected vs. actual output
- Rust version (`rustc --version`)

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
