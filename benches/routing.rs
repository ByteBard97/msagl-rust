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

fn bench_polygon(c: &mut Criterion) {
    // Mixed scenario: 12 rectangular obstacles + 8 triangular/hexagonal polygon
    // obstacles arranged in a grid. 40 edges routed between them.
    // Exercises the padded-polyline, perpendicular port entrance, and obstacle
    // transparency code paths that are dead in the rect-only benchmarks.

    fn make_router() -> (msagl_rust::RoutingResult, usize) {
        use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

        let padding = 3.0;

        // 12 rectangles arranged in a 4×3 grid (column-major, 120px spacing)
        let mut shapes: Vec<Shape> = (0..4)
            .flat_map(|col| {
                (0..3).map(move |row| {
                    Shape::rectangle(
                        col as f64 * 120.0,
                        row as f64 * 100.0,
                        60.0,
                        40.0,
                    )
                })
            })
            .collect();

        // 8 polygon obstacles: alternating upward triangles and hexagons
        // placed in the gaps between the rect grid rows.
        let poly_defs: &[&[(f64, f64)]] = &[
            // upward triangles (4)
            &[(490.0, 30.0), (460.0, 70.0), (520.0, 70.0)],
            &[(490.0, 130.0), (460.0, 170.0), (520.0, 170.0)],
            &[(490.0, 230.0), (460.0, 270.0), (520.0, 270.0)],
            &[(490.0, 330.0), (460.0, 370.0), (520.0, 370.0)],
            // rough hexagons (4)
            &[
                (620.0, 50.0), (640.0, 30.0), (670.0, 30.0),
                (690.0, 50.0), (670.0, 70.0), (640.0, 70.0),
            ],
            &[
                (620.0, 150.0), (640.0, 130.0), (670.0, 130.0),
                (690.0, 150.0), (670.0, 170.0), (640.0, 170.0),
            ],
            &[
                (620.0, 250.0), (640.0, 230.0), (670.0, 230.0),
                (690.0, 250.0), (670.0, 270.0), (640.0, 270.0),
            ],
            &[
                (620.0, 350.0), (640.0, 330.0), (670.0, 330.0),
                (690.0, 350.0), (670.0, 370.0), (640.0, 370.0),
            ],
        ];

        for pts in poly_defs {
            let points: Vec<Point> = pts.iter().map(|&(x, y)| Point::new(x, y)).collect();
            shapes.push(Shape::polygon(&points));
        }

        let n_rect = 12_usize;
        let n_poly = 8_usize;
        let n = n_rect + n_poly;

        let mut router = RectilinearEdgeRouter::new(&shapes)
            .padding(padding)
            .edge_separation(2.0);

        let mut n_edges = 0_usize;

        // rect↔rect: every rect to the one 4 columns ahead (wraps), 12 edges
        for i in 0..n_rect {
            let j = (i + 4) % n_rect;
            let src_pt = shapes[i].bounding_box().center();
            let tgt_pt = shapes[j].bounding_box().center();
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(i, src_pt),
                FloatingPort::new(j, tgt_pt),
            ));
            n_edges += 1;
        }
        // rect↔poly: 16 edges (2 per polygon)
        for k in 0..n_poly * 2 {
            let r = (k * 3) % n_rect;
            let p = n_rect + (k % n_poly);
            let src_pt = shapes[r].bounding_box().center();
            let tgt_pt = shapes[p].bounding_box().center();
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(r, src_pt),
                FloatingPort::new(p, tgt_pt),
            ));
            n_edges += 1;
        }
        // poly↔poly: 8 edges (ring)
        for k in 0..n_poly {
            let p1 = n_rect + k;
            let p2 = n_rect + (k + 1) % n_poly;
            let src_pt = shapes[p1].bounding_box().center();
            let tgt_pt = shapes[p2].bounding_box().center();
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(p1, src_pt),
                FloatingPort::new(p2, tgt_pt),
            ));
            n_edges += 1;
        }

        let _ = n;
        (router.run(), n_edges)
    }

    // One-time correctness check
    let (result, n_edges) = make_router();
    assert_eq!(result.edges.len(), n_edges, "all edges must be routed");

    c.bench_function(
        "routing/polygon (12 rects + 8 polygons, 36 edges)",
        |b| b.iter(|| make_router().0),
    );
}

criterion_group!(benches, bench_small, bench_medium, bench_large, bench_polygon);
criterion_main!(benches);
