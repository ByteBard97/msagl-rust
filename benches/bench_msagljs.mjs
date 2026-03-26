/**
 * MSAGL-JS benchmark script — runs the ORIGINAL TypeScript RectilinearEdgeRouter
 * on the same scenarios as our Rust benchmarks, for direct comparison.
 *
 * Prerequisites (one-time build of the MSAGL-JS reference):
 *
 *   MSAGL_JS=<SignalCanvas>/SignalCanvasFrontend/reference/msagl-js
 *
 *   1. Install dependencies:
 *      cd $MSAGL_JS && npm install --legacy-peer-deps
 *
 *   2. Build core module as CJS (two config patches needed):
 *      - modules/core/package.json: change "type" from "module" to "commonjs"
 *      - modules/core/tsconfig.prod.json: add "module": "commonjs",
 *        "skipLibCheck": true, "typeRoots": ["../../node_modules/@types"]
 *      Then: cd $MSAGL_JS/modules/core && npx tsc --build tsconfig.prod.json
 *
 * Usage:
 *   node benches/bench_msagljs.mjs
 *
 * Output format matches bench_wasm.mjs for direct comparison.
 */

import { readFileSync } from "fs";
import { join, dirname } from "path";
import { fileURLToPath } from "url";
import { createRequire } from "module";

const __dirname = dirname(fileURLToPath(import.meta.url));
const ROOT = join(__dirname, "..");
const require = createRequire(import.meta.url);

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const ITERATIONS = 100;
const SCENARIO_NAMES = ["small", "medium", "large"];
const MSAGL_CORE_DIST = join(
  ROOT,
  "../SignalCanvasFrontend/reference/msagl-js/modules/core/dist"
);

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
  // Load the built MSAGL-JS core module via createRequire (CJS output)
  let msagl;
  try {
    msagl = require(join(MSAGL_CORE_DIST, "index.js"));
  } catch (err) {
    console.error(
      "ERROR: Could not load MSAGL-JS core from dist/"
    );
    console.error(
      "Build it first:"
    );
    console.error(
      "  cd <SignalCanvas>/SignalCanvasFrontend/reference/msagl-js"
    );
    console.error("  npm install --legacy-peer-deps");
    console.error(
      "  cd modules/core && npx tsc --build tsconfig.prod.json"
    );
    console.error(`\nDetails: ${err.message}`);
    process.exit(1);
  }

  const {
    GeomGraph,
    GeomNode,
    GeomEdge,
    Graph,
    Node,
    Edge,
    CurveFactory,
    Point,
    FloatingPort,
    RectilinearEdgeRouter,
  } = msagl;

  // Verify all required exports are present
  const requiredExports = [
    ["GeomGraph", GeomGraph],
    ["GeomNode", GeomNode],
    ["GeomEdge", GeomEdge],
    ["Graph", Graph],
    ["Node", Node],
    ["Edge", Edge],
    ["CurveFactory", CurveFactory],
    ["Point", Point],
    ["FloatingPort", FloatingPort],
    ["RectilinearEdgeRouter", RectilinearEdgeRouter],
  ];
  for (const [name, val] of requiredExports) {
    if (!val) {
      console.error(`ERROR: ${name} not found in MSAGL-JS core exports`);
      process.exit(1);
    }
  }

  /**
   * Converts a scenario JSON to an MSAGL-JS GeomGraph and runs routing.
   *
   * Scenario JSON format:
   *   obstacles: [{x, y, width, height}, ...]   -- x,y = top-left corner
   *   edges: [{source: {x,y}, target: {x,y}, sourceObstacle, targetObstacle}]
   *   padding: number
   *   edgeSeparation: number
   *
   * MSAGL-JS coordinate model:
   *   CurveFactory.createRectangle(width, height, center) where center is the
   *   midpoint of the rectangle. Our JSON has x,y as the top-left corner, so
   *   we convert: center = (x + width/2, y + height/2).
   *
   *   FloatingPort(curve, location) takes the obstacle boundary curve and the
   *   specific port location on its perimeter.
   */
  function runScenario(scenario) {
    const gg = new GeomGraph(new Graph("bench"));

    // Create nodes for each obstacle
    const nodes = scenario.obstacles.map((obs, i) => {
      const node = new Node(`n${i}`);
      gg.graph.addNode(node);
      const geomNode = new GeomNode(node);

      // Our JSON: x,y = top-left corner. MSAGL-JS createRectangle wants center.
      const centerX = obs.x + obs.width / 2;
      const centerY = obs.y + obs.height / 2;
      geomNode.boundaryCurve = CurveFactory.createRectangle(
        obs.width,
        obs.height,
        new Point(centerX, centerY)
      );
      return node;
    });

    // Create edges with FloatingPort at specific source/target locations
    const geomEdges = [];
    for (const edgeDef of scenario.edges) {
      const srcNode = nodes[edgeDef.sourceObstacle];
      const tgtNode = nodes[edgeDef.targetObstacle];
      const edge = new Edge(srcNode, tgtNode);
      const geomEdge = new GeomEdge(edge);

      // Get the GeomNode boundary curves for the FloatingPort
      const srcGeom = GeomNode.getGeom(srcNode);
      const tgtGeom = GeomNode.getGeom(tgtNode);

      geomEdge.sourcePort = new FloatingPort(
        srcGeom.boundaryCurve,
        new Point(edgeDef.source.x, edgeDef.source.y)
      );
      geomEdge.targetPort = new FloatingPort(
        tgtGeom.boundaryCurve,
        new Point(edgeDef.target.x, edgeDef.target.y)
      );

      geomEdges.push(geomEdge);
    }

    // Create router with obstacles as shapes, add edges, run
    const rr = RectilinearEdgeRouter.constructorGNAN(
      gg,
      geomEdges,
      scenario.padding,
      /* cornerFitRadius */ 3
    );
    rr.edgeSeparatian = scenario.edgeSeparation;
    rr.run();

    return geomEdges;
  }

  console.log("MSAGL-JS Benchmark Results");
  console.log("=".repeat(60));
  console.log();

  for (const name of SCENARIO_NAMES) {
    const scenario = loadScenario(name);
    const numObstacles = scenario.obstacles.length;
    const numEdges = scenario.edges.length;

    // Correctness check (single run)
    let geomEdges;
    try {
      geomEdges = runScenario(scenario);
    } catch (err) {
      console.error(`ERROR: ${name} — routing failed: ${err.message}`);
      console.error(err.stack);
      process.exit(1);
    }

    if (geomEdges.length !== numEdges) {
      console.error(
        `ERROR: ${name} — expected ${numEdges} edges, got ${geomEdges.length}`
      );
      process.exit(1);
    }

    // Check that edges have routed curves (not null)
    let routed = 0;
    let unrouted = 0;
    for (let i = 0; i < geomEdges.length; i++) {
      if (geomEdges[i].curve != null) {
        routed++;
      } else {
        unrouted++;
      }
    }
    if (unrouted > 0) {
      console.warn(
        `  WARN: ${name} — ${unrouted}/${numEdges} edges have null curves (failed to route)`
      );
    }

    // Benchmark: run ITERATIONS times
    const timings = [];
    for (let iter = 0; iter < ITERATIONS; iter++) {
      const start = performance.now();
      runScenario(scenario);
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
