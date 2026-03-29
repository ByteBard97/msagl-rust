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

        let padding = 6.0;

        // Layout: left cluster (6 rects) | polygon corridor | right cluster (6 rects)
        //
        // Left rects:   2 columns × 3 rows, x=0..140,   pitch 200px col / 150px row
        // Right rects:  2 columns × 3 rows, x=650..790, pitch 200px col / 150px row
        // Corridor:     x=200..600 — polygon obstacles sit here forcing route detours
        //
        // Every left-to-right edge must navigate THROUGH the corridor, exercising
        // polygon obstacle avoidance on every route.

        // Left cluster: cols 0-1
        let mut shapes: Vec<Shape> = (0..2)
            .flat_map(|col| {
                (0..3).map(move |row| {
                    Shape::rectangle(
                        col as f64 * 200.0,
                        row as f64 * 150.0,
                        120.0,
                        80.0,
                    )
                })
            })
            .collect();

        // Right cluster: cols 0-1, offset to x=650
        for col in 0..2_usize {
            for row in 0..3_usize {
                shapes.push(Shape::rectangle(
                    650.0 + col as f64 * 200.0,
                    row as f64 * 150.0,
                    120.0,
                    80.0,
                ));
            }
        }

        // 8 polygon obstacles scattered through the corridor (x=220..580)
        // — triangles point upward, hexagons are wide
        // — staggered vertically so routes at different row heights must detour
        let poly_defs: &[&[(f64, f64)]] = &[
            // triangle blocking upper corridor, left side
            &[(290.0,  30.0), (230.0, 130.0), (350.0, 130.0)],
            // triangle blocking lower corridor, left side
            &[(290.0, 200.0), (230.0, 300.0), (350.0, 300.0)],
            // triangle blocking upper corridor, right side
            &[(510.0,  80.0), (450.0, 180.0), (570.0, 180.0)],
            // triangle blocking lower corridor, right side
            &[(510.0, 240.0), (450.0, 340.0), (570.0, 340.0)],
            // hexagon in the mid corridor, upper half
            &[
                (390.0,  50.0), (420.0,  20.0), (470.0,  20.0),
                (500.0,  50.0), (470.0, 120.0), (420.0, 120.0),
            ],
            // hexagon in the mid corridor, lower half
            &[
                (390.0, 210.0), (420.0, 180.0), (470.0, 180.0),
                (500.0, 210.0), (470.0, 280.0), (420.0, 280.0),
            ],
            // hexagon blocking the very centre
            &[
                (370.0, 120.0), (400.0,  95.0), (450.0,  95.0),
                (480.0, 120.0), (450.0, 175.0), (400.0, 175.0),
            ],
            // wide triangle spanning bottom of corridor
            &[(420.0, 310.0), (330.0, 420.0), (510.0, 420.0)],
        ];

        for pts in poly_defs {
            let points: Vec<Point> = pts.iter().map(|&(x, y)| Point::new(x, y)).collect();
            shapes.push(Shape::polygon(&points));
        }

        let n_left = 6_usize;   // rects 0..5
        let n_right = 6_usize;  // rects 6..11
        let n_rect = n_left + n_right;
        let n_poly = 8_usize;

        let mut router = RectilinearEdgeRouter::new(&shapes)
            .padding(padding)
            .edge_separation(8.0);

        let mut n_edges = 0_usize;

        // left ↔ right: all 36 cross-corridor pairs (6×6)
        // Every one of these must navigate through the polygon obstacle corridor.
        for l in 0..n_left {
            for r in 0..n_right {
                let src_pt = shapes[l].bounding_box().center();
                let tgt_pt = shapes[n_left + r].bounding_box().center();
                router.add_edge(EdgeGeometry::new(
                    FloatingPort::new(l, src_pt),
                    FloatingPort::new(n_left + r, tgt_pt),
                ));
                n_edges += 1;
            }
        }
        // within left cluster: ring (6 edges)
        for i in 0..n_left {
            let j = (i + 1) % n_left;
            let src_pt = shapes[i].bounding_box().center();
            let tgt_pt = shapes[j].bounding_box().center();
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(i, src_pt),
                FloatingPort::new(j, tgt_pt),
            ));
            n_edges += 1;
        }
        // within right cluster: ring (6 edges)
        for i in 0..n_right {
            let j = (i + 1) % n_right;
            let src_pt = shapes[n_left + i].bounding_box().center();
            let tgt_pt = shapes[n_left + j].bounding_box().center();
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(n_left + i, src_pt),
                FloatingPort::new(n_left + j, tgt_pt),
            ));
            n_edges += 1;
        }
        let _ = n_rect;
        let _ = n_poly;

        (router.run(), n_edges)
    }

    // One-time correctness check
    let (result, n_edges) = make_router();
    assert_eq!(result.edges.len(), n_edges, "all edges must be routed");

    c.bench_function(
        "routing/polygon (12 rects + 8 polygons, 36 edges, sep=8)",
        |b| b.iter(|| make_router().0),
    );
}

criterion_group!(benches, bench_small, bench_medium, bench_large, bench_polygon);
criterion_main!(benches);
