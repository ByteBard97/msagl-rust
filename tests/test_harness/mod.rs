pub mod scenario_builder;
pub mod verifier;

pub use scenario_builder::ScenarioBuilder;
pub use verifier::Verifier;

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, RoutingResult, Shape};
use serde::Deserialize;
use std::fs;

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BenchScenario {
    pub obstacles: Vec<BenchObstacle>,
    pub edges: Vec<BenchEdge>,
    pub padding: f64,
    pub edge_separation: f64,
}

#[derive(Deserialize)]
pub struct BenchObstacle {
    pub x: f64,
    pub y: f64,
    pub width: f64,
    pub height: f64,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BenchEdge {
    pub source: BenchPoint,
    pub target: BenchPoint,
    pub source_obstacle: usize,
    pub target_obstacle: usize,
}

#[derive(Deserialize)]
pub struct BenchPoint {
    pub x: f64,
    pub y: f64,
}

pub fn load_bench_scenario(name: &str) -> BenchScenario {
    let path = format!("{}/benches/scenarios/{name}.json", env!("CARGO_MANIFEST_DIR"));
    serde_json::from_str(&fs::read_to_string(path).unwrap()).unwrap()
}

pub fn run_bench_scenario(s: &BenchScenario) -> RoutingResult {
    let shapes: Vec<Shape> = s.obstacles.iter()
        .map(|o| Shape::rectangle(o.x, o.y, o.width, o.height))
        .collect();
    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(s.padding)
        .edge_separation(s.edge_separation);
    for e in &s.edges {
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(e.source_obstacle, Point::new(e.source.x, e.source.y)),
            FloatingPort::new(e.target_obstacle, Point::new(e.target.x, e.target.y)),
        ));
    }
    router.run()
}
