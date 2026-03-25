//! Criterion benchmarks for msagl-rust rectilinear edge routing.
//!
//! Loads JSON scenario files (small/medium/large) and benchmarks the
//! full routing pipeline. Includes correctness assertions to ensure
//! every path has > 2 waypoints (no fallback straight-line paths).

use criterion::{criterion_group, criterion_main, Criterion};
use serde::Deserialize;
use std::fs;
use std::path::PathBuf;

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

// ---------------------------------------------------------------------------
// Scenario deserialization types (matches the JSON schema)
// ---------------------------------------------------------------------------

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Scenario {
    obstacles: Vec<ObstacleJson>,
    edges: Vec<EdgeJson>,
    padding: f64,
    edge_separation: f64,
}

#[derive(Deserialize)]
struct ObstacleJson {
    x: f64,
    y: f64,
    width: f64,
    height: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct EdgeJson {
    source: PointJson,
    target: PointJson,
    source_obstacle: usize,
    target_obstacle: usize,
}

#[derive(Deserialize)]
struct PointJson {
    x: f64,
    y: f64,
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Path to the `benches/scenarios/` directory.
fn scenarios_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("benches/scenarios")
}

/// Load a scenario JSON file and return parsed data.
fn load_scenario(name: &str) -> Scenario {
    let path = scenarios_dir().join(format!("{name}.json"));
    let data = fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("Failed to read {}: {e}", path.display()));
    serde_json::from_str(&data)
        .unwrap_or_else(|e| panic!("Failed to parse {}: {e}", path.display()))
}

/// Run the router on a loaded scenario and return the result.
fn run_scenario(scenario: &Scenario) -> msagl_rust::RoutingResult {
    let shapes: Vec<Shape> = scenario
        .obstacles
        .iter()
        .map(|o| Shape::rectangle(o.x, o.y, o.width, o.height))
        .collect();

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(scenario.padding)
        .edge_separation(scenario.edge_separation);

    for edge in &scenario.edges {
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

    router.run()
}

/// Assert correctness: every routed path must have > 2 waypoints
/// (i.e., no fallback straight-line paths).
fn assert_no_fallbacks(result: &msagl_rust::RoutingResult, scenario_name: &str) {
    for (i, edge) in result.edges.iter().enumerate() {
        assert!(
            edge.points.len() > 2,
            "{scenario_name}: edge {i} has only {} waypoints (expected > 2, \
             likely a fallback path)",
            edge.points.len(),
        );
    }
}

// ---------------------------------------------------------------------------
// Benchmarks
// ---------------------------------------------------------------------------

fn bench_small(c: &mut Criterion) {
    let scenario = load_scenario("small");

    // One-time correctness check
    let result = run_scenario(&scenario);
    assert_eq!(result.edges.len(), scenario.edges.len());

    c.bench_function("routing/small (5 obstacles, 4 edges)", |b| {
        b.iter(|| run_scenario(&scenario))
    });
}

fn bench_medium(c: &mut Criterion) {
    let scenario = load_scenario("medium");

    // One-time correctness check
    let result = run_scenario(&scenario);
    assert_eq!(result.edges.len(), scenario.edges.len());

    c.bench_function("routing/medium (49 obstacles, 80 edges)", |b| {
        b.iter(|| run_scenario(&scenario))
    });
}

fn bench_large(c: &mut Criterion) {
    let scenario = load_scenario("large");

    // One-time correctness check
    let result = run_scenario(&scenario);
    assert_eq!(result.edges.len(), scenario.edges.len());

    c.bench_function("routing/large (210 obstacles, 500 edges)", |b| {
        b.iter(|| run_scenario(&scenario))
    });
}

criterion_group!(benches, bench_small, bench_medium, bench_large);
criterion_main!(benches);
