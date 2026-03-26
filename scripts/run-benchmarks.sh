#!/usr/bin/env bash
# run-benchmarks.sh — Run Rust, WASM, and Python benchmarks and print a summary.
#
# Usage: ./scripts/run-benchmarks.sh [--rust] [--wasm] [--python] [--all]
#
# With no flags, runs all three. Individual flags run only the specified targets.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---------------------------------------------------------------------------
# Parse flags
# ---------------------------------------------------------------------------
RUN_RUST=false
RUN_WASM=false
RUN_PYTHON=false

if [ $# -eq 0 ]; then
    RUN_RUST=true
    RUN_WASM=true
    RUN_PYTHON=true
fi

for arg in "$@"; do
    case "$arg" in
        --rust)   RUN_RUST=true ;;
        --wasm)   RUN_WASM=true ;;
        --python) RUN_PYTHON=true ;;
        --all)
            RUN_RUST=true
            RUN_WASM=true
            RUN_PYTHON=true
            ;;
        *)
            echo "Unknown flag: $arg"
            echo "Usage: $0 [--rust] [--wasm] [--python] [--all]"
            exit 1
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Separator
# ---------------------------------------------------------------------------
separator() {
    echo ""
    echo "================================================================"
    echo " $1"
    echo "================================================================"
    echo ""
}

# ---------------------------------------------------------------------------
# Rust benchmarks (criterion)
# ---------------------------------------------------------------------------
if [ "$RUN_RUST" = true ]; then
    separator "Rust Native Benchmarks (criterion)"
    cd "$ROOT"
    cargo bench --bench routing 2>&1 | tail -20
fi

# ---------------------------------------------------------------------------
# WASM benchmarks (Node.js)
# ---------------------------------------------------------------------------
if [ "$RUN_WASM" = true ]; then
    separator "WASM Benchmarks (Node.js)"

    PKG_DIR="$ROOT/crates/msagl-wasm/pkg-web"
    if [ ! -d "$PKG_DIR" ]; then
        echo "WASM package not found at $PKG_DIR"
        echo "Building with wasm-pack..."
        cd "$ROOT/crates/msagl-wasm"
        wasm-pack build --target web --out-dir pkg-web
        cd "$ROOT"
    fi

    node "$ROOT/benches/bench_wasm.mjs"
fi

# ---------------------------------------------------------------------------
# Python benchmarks
# ---------------------------------------------------------------------------
if [ "$RUN_PYTHON" = true ]; then
    separator "Python Benchmarks"

    # Check if msagl module is importable
    if ! python3 -c "import msagl" 2>/dev/null; then
        echo "msagl Python module not found. Building with maturin..."
        cd "$ROOT/crates/msagl-python"
        maturin develop --release
        cd "$ROOT"
    fi

    python3 "$ROOT/benches/bench_python.py"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
separator "Benchmark Summary"
echo "Targets run:"
[ "$RUN_RUST" = true ]   && echo "  - Rust native (criterion)"
[ "$RUN_WASM" = true ]   && echo "  - WASM (Node.js)"
[ "$RUN_PYTHON" = true ] && echo "  - Python (maturin)"
echo ""
echo "Scenario files: benches/scenarios/{small,medium,large}.json"
echo "Criterion HTML reports: target/criterion/"
echo ""
echo "Done."
