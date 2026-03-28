/**
 * WASM benchmark script for msagl-rust rectilinear edge routing.
 *
 * Loads the WASM module from crates/msagl-wasm/pkg-web/ and runs
 * each scenario (small/medium/large) 100 times, reporting mean +/- stddev.
 *
 * Usage: node benches/bench_wasm.mjs
 */

import { readFileSync } from "fs";
import { join, dirname } from "path";
import { fileURLToPath } from "url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const ROOT = join(__dirname, "..");

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const ITERATIONS = 100;
const SCENARIO_NAMES = ["small", "medium", "large"];
const PKG_DIR = join(ROOT, "crates/msagl-wasm/pkg-node");

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function loadScenario(name) {
  const path = join(__dirname, "scenarios", `${name}.json`);
  return JSON.parse(readFileSync(path, "utf-8"));
}

function mean(values) {
  return values.reduce((a, b) => a + b, 0) / values.length;
}

function stddev(values) {
  const m = mean(values);
  const variance =
    values.reduce((sum, v) => sum + (v - m) ** 2, 0) / values.length;
  return Math.sqrt(variance);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

async function main() {
  // Dynamically import the WASM module
  let wasmModule;
  try {
    wasmModule = await import(join(PKG_DIR, "msagl_wasm.js"));
  } catch (err) {
    console.error(
      "ERROR: Could not load WASM module from crates/msagl-wasm/pkg-web/"
    );
    console.error(
      "Build it first: cd crates/msagl-wasm && wasm-pack build --target web --out-dir pkg-web"
    );
    console.error(`Details: ${err.message}`);
    process.exit(1);
  }

  // Initialise the WASM module
  if (typeof wasmModule.default === "function") {
    await wasmModule.default();
  } else if (wasmModule.init) {
    wasmModule.init();
  }

  const { route_edges } = wasmModule;
  if (!route_edges) {
    console.error("ERROR: route_edges not found in WASM module exports");
    process.exit(1);
  }

  console.log("WASM Benchmark Results");
  console.log("=".repeat(60));
  console.log();

  for (const name of SCENARIO_NAMES) {
    const scenario = loadScenario(name);
    const numObstacles = scenario.obstacles.length;
    const numEdges = scenario.edges.length;

    // Correctness check (single run)
    const result = route_edges(scenario);
    if (!result || !result.paths) {
      console.error(`ERROR: ${name} — route_edges returned no paths`);
      process.exit(1);
    }
    if (result.paths.length !== numEdges) {
      console.error(
        `ERROR: ${name} — expected ${numEdges} paths, got ${result.paths.length}`
      );
      process.exit(1);
    }

    let fallbacks = 0;
    for (let i = 0; i < result.paths.length; i++) {
      if (result.paths[i].points.length <= 2) {
        fallbacks++;
      }
    }
    if (fallbacks > 0) {
      console.warn(
        `  WARN: ${name} — ${fallbacks}/${numEdges} edges have <= 2 waypoints (fallback paths)`
      );
    }

    // Benchmark: run ITERATIONS times
    const timings = [];
    for (let iter = 0; iter < ITERATIONS; iter++) {
      const start = performance.now();
      route_edges(scenario);
      const elapsed = performance.now() - start;
      timings.push(elapsed);
    }

    const m = mean(timings);
    const sd = stddev(timings);

    console.log(
      `${name.padEnd(8)} (${numObstacles} obstacles, ${numEdges} edges): ` +
        `${m.toFixed(3)} ± ${sd.toFixed(3)} ms  (${ITERATIONS} iterations)`
    );
  }

  console.log();
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
