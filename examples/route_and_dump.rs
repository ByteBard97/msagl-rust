//! Route all benchmark scenarios and dump paths as JSON to stdout.
//!
//! Used by `benches/render_scenarios.py` to render routed SVGs.
//!
//! Output format (one JSON object per line):
//! ```json
//! {"scenario":"small","edges":[ [[x,y],[x,y],...], ... ]}
//! ```
//!
//! Usage:
//!   cargo run --example route_and_dump
//!   cargo run --example route_and_dump -- small medium   # subset

use std::env;
use std::fs;
use std::path::PathBuf;

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

fn repo_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

// ── JSON scenario types (minimal — no serde needed, hand-parsed) ──────────

/// Parse a JSON number (integer or float) from a string slice, returning the
/// value and the number of bytes consumed.
fn parse_number(s: &str) -> Option<(f64, usize)> {
    let end = s
        .find(|c: char| !c.is_ascii_digit() && c != '.' && c != '-' && c != 'e' && c != '+')
        .unwrap_or(s.len());
    if end == 0 {
        return None;
    }
    s[..end].parse::<f64>().ok().map(|v| (v, end))
}

/// Pull all `"key": value` number pairs out of a JSON object string.
fn extract_numbers(obj: &str) -> Vec<(String, f64)> {
    let mut out = Vec::new();
    let mut rest = obj;
    while let Some(q) = rest.find('"') {
        rest = &rest[q + 1..];
        let end_key = rest.find('"').unwrap_or(rest.len());
        let key = rest[..end_key].to_string();
        rest = &rest[end_key + 1..];
        // skip whitespace and colon
        let colon = rest.find(':').unwrap_or(rest.len());
        rest = &rest[colon + 1..];
        let trimmed = rest.trim_start();
        if let Some((val, n)) = parse_number(trimmed) {
            out.push((key, val));
            let offset = rest.len() - trimmed.len() + n;
            rest = &rest[offset..];
        }
    }
    out
}

struct ObstacleData {
    x: f64,
    y: f64,
    width: f64,
    height: f64,
}

struct EdgeData {
    src_x: f64,
    src_y: f64,
    src_obs: usize,
    tgt_x: f64,
    tgt_y: f64,
    tgt_obs: usize,
}

struct Scenario {
    obstacles: Vec<ObstacleData>,
    edges: Vec<EdgeData>,
    padding: f64,
    edge_separation: f64,
}

fn load_scenario(name: &str) -> Scenario {
    let path = repo_root()
        .join("benches")
        .join("scenarios")
        .join(format!("{name}.json"));
    let raw = fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));

    // Extract top-level padding and edgeSeparation
    let top_nums = extract_numbers(&raw);
    let padding = top_nums
        .iter()
        .find(|(k, _)| k == "padding")
        .map(|(_, v)| *v)
        .unwrap_or(4.0);
    let edge_separation = top_nums
        .iter()
        .find(|(k, _)| k == "edgeSeparation")
        .map(|(_, v)| *v)
        .unwrap_or(8.0);

    // Parse obstacles array: find each { "x":..,"y":..,"width":..,"height":.. }
    let obs_start = raw.find("\"obstacles\"").expect("no obstacles key");
    let arr_start = raw[obs_start..].find('[').expect("no [ after obstacles") + obs_start;
    let arr_end = find_array_end(&raw, arr_start);
    let obs_arr = &raw[arr_start + 1..arr_end];

    let obstacles = split_objects(obs_arr)
        .into_iter()
        .map(|obj| {
            let nums = extract_numbers(&obj);
            let get = |k: &str| nums.iter().find(|(key, _)| key == k).map(|(_, v)| *v).unwrap_or(0.0);
            ObstacleData {
                x: get("x"),
                y: get("y"),
                width: get("width"),
                height: get("height"),
            }
        })
        .collect();

    // Parse edges array
    let edge_start = raw.find("\"edges\"").expect("no edges key");
    let arr_start2 = raw[edge_start..].find('[').expect("no [ after edges") + edge_start;
    let arr_end2 = find_array_end(&raw, arr_start2);
    let edge_arr = &raw[arr_start2 + 1..arr_end2];

    let edges = split_objects(edge_arr)
        .into_iter()
        .map(|obj| {
            let nums = extract_numbers(&obj);
            let get = |k: &str| nums.iter().find(|(key, _)| key == k).map(|(_, v)| *v).unwrap_or(0.0);
            EdgeData {
                src_x: get("x"),     // first x is source.x
                src_y: get("y"),
                src_obs: get("sourceObstacle") as usize,
                tgt_x: {
                    // second occurrence of "x" is target.x
                    nums.iter()
                        .filter(|(k, _)| k == "x")
                        .nth(1)
                        .map(|(_, v)| *v)
                        .unwrap_or(0.0)
                },
                tgt_y: {
                    nums.iter()
                        .filter(|(k, _)| k == "y")
                        .nth(1)
                        .map(|(_, v)| *v)
                        .unwrap_or(0.0)
                },
                tgt_obs: get("targetObstacle") as usize,
            }
        })
        .collect();

    Scenario { obstacles, edges, padding, edge_separation }
}

/// Find the index of the closing `]` for an array starting at `start`.
fn find_array_end(s: &str, start: usize) -> usize {
    let mut depth = 0_i32;
    for (i, c) in s[start..].char_indices() {
        match c {
            '[' | '{' => depth += 1,
            ']' | '}' => {
                depth -= 1;
                if depth == 0 {
                    return start + i;
                }
            }
            _ => {}
        }
    }
    s.len()
}

/// Split a JSON array body (between `[` and `]`) into top-level object strings.
fn split_objects(arr: &str) -> Vec<String> {
    let mut objs = Vec::new();
    let mut depth = 0_i32;
    let mut start: Option<usize> = None;
    for (i, c) in arr.char_indices() {
        match c {
            '{' => {
                if depth == 0 {
                    start = Some(i);
                }
                depth += 1;
            }
            '}' => {
                depth -= 1;
                if depth == 0 {
                    if let Some(s) = start.take() {
                        objs.push(arr[s..=i].to_string());
                    }
                }
            }
            _ => {}
        }
    }
    objs
}

// ── Route a scenario ─────────────────────────────────────────────────────────

fn route(scenario: &Scenario) -> Vec<Vec<(f64, f64)>> {
    let shapes: Vec<Shape> = scenario
        .obstacles
        .iter()
        .map(|o| Shape::rectangle(o.x, o.y, o.width, o.height))
        .collect();

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(scenario.padding)
        .edge_separation(scenario.edge_separation);

    for e in &scenario.edges {
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(e.src_obs, Point::new(e.src_x, e.src_y)),
            FloatingPort::new(e.tgt_obs, Point::new(e.tgt_x, e.tgt_y)),
        ));
    }

    router
        .run()
        .edges
        .into_iter()
        .map(|e| e.points.into_iter().map(|p| (p.x(), p.y())).collect())
        .collect()
}

// ── Polygon scenario ─────────────────────────────────────────────────────────

fn route_polygon() -> Vec<Vec<(f64, f64)>> {
    let mut shapes: Vec<Shape> = (0..4)
        .flat_map(|col| {
            (0..3).map(move |row| {
                Shape::rectangle(col as f64 * 120.0, row as f64 * 100.0, 60.0, 40.0)
            })
        })
        .collect();

    let poly_defs: &[&[(f64, f64)]] = &[
        &[(490.0, 30.0), (460.0, 70.0), (520.0, 70.0)],
        &[(490.0, 130.0), (460.0, 170.0), (520.0, 170.0)],
        &[(490.0, 230.0), (460.0, 270.0), (520.0, 270.0)],
        &[(490.0, 330.0), (460.0, 370.0), (520.0, 370.0)],
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
    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(3.0)
        .edge_separation(2.0);

    for i in 0..n_rect {
        let j = (i + 4) % n_rect;
        let src_pt = shapes[i].bounding_box().center();
        let tgt_pt = shapes[j].bounding_box().center();
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(i, src_pt),
            FloatingPort::new(j, tgt_pt),
        ));
    }
    for k in 0..n_poly * 2 {
        let r = (k * 3) % n_rect;
        let p = n_rect + (k % n_poly);
        let src_pt = shapes[r].bounding_box().center();
        let tgt_pt = shapes[p].bounding_box().center();
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(r, src_pt),
            FloatingPort::new(p, tgt_pt),
        ));
    }
    for k in 0..n_poly {
        let p1 = n_rect + k;
        let p2 = n_rect + (k + 1) % n_poly;
        let src_pt = shapes[p1].bounding_box().center();
        let tgt_pt = shapes[p2].bounding_box().center();
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(p1, src_pt),
            FloatingPort::new(p2, tgt_pt),
        ));
    }

    router
        .run()
        .edges
        .into_iter()
        .map(|e| e.points.into_iter().map(|p| (p.x(), p.y())).collect())
        .collect()
}

// ── JSON output ───────────────────────────────────────────────────────────────

fn dump_json(name: &str, edges: &[Vec<(f64, f64)>]) {
    print!("{{\"scenario\":\"{name}\",\"edges\":[");
    for (i, edge) in edges.iter().enumerate() {
        if i > 0 {
            print!(",");
        }
        print!("[");
        for (j, (x, y)) in edge.iter().enumerate() {
            if j > 0 {
                print!(",");
            }
            print!("[{x:.3},{y:.3}]");
        }
        print!("]");
    }
    println!("]}}");
}

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    let all = ["small", "medium", "large", "polygon"];
    let targets: Vec<&str> = if args.is_empty() {
        all.to_vec()
    } else {
        args.iter().map(|s| s.as_str()).collect()
    };

    for name in targets {
        let edges = if name == "polygon" {
            route_polygon()
        } else {
            let s = load_scenario(name);
            route(&s)
        };
        dump_json(name, &edges);
    }
}
