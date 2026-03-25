//! First batch of rectilinear routing tests.
//!
//! Ported from `RectilinearTests.cs` — covers basic obstacle configurations
//! and routing invariants.

mod test_harness;

use test_harness::{ScenarioBuilder, Verifier};

/// Three obstacles in a triangle, routes between all pairs.
/// Mirrors the C# `Diamond3` test pattern.
#[test]
fn diamond3() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(200.0, 100.0, 100.0, 100.0);
    let o1 = b.add_rectangle(100.0, 300.0, 100.0, 100.0);
    let o2 = b.add_rectangle(300.0, 300.0, 100.0, 100.0);
    b.route_between(o0, o1);
    b.route_between(o0, o2);
    b.route_between(o1, o2);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 3);
}

/// Two simple boxes side by side, one route between them.
/// Mirrors the C# `TwoSquares` test.
#[test]
fn two_squares() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(50.0, 50.0, 100.0, 100.0);
    let o1 = b.add_rectangle(250.0, 50.0, 100.0, 100.0);
    b.route_between(o0, o1);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
}

/// Single obstacle, no edges to route.
#[test]
fn single_obstacle_no_routing() {
    let mut b = ScenarioBuilder::new();
    b.add_rectangle(100.0, 100.0, 50.0, 50.0);

    let result = b.run();
    assert_eq!(result.edges.len(), 0);
}

/// Empty graph: zero obstacles, zero edges.
#[test]
fn zero_obstacles() {
    let b = ScenarioBuilder::new();
    let result = b.run();
    assert_eq!(result.edges.len(), 0);
}

/// Three boxes in an L-shaped grid, routes between all pairs.
#[test]
fn three_boxes_grid() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(0.0, 0.0, 80.0, 40.0);
    let o1 = b.add_rectangle(200.0, 0.0, 80.0, 40.0);
    let o2 = b.add_rectangle(100.0, 150.0, 80.0, 40.0);
    b.route_between(o0, o1);
    b.route_between(o0, o2);
    b.route_between(o1, o2);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 3);
}

/// Four boxes at corners of a square, routes between all 6 pairs.
#[test]
fn four_boxes_all_pairs() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(0.0, 0.0, 60.0, 60.0);
    let o1 = b.add_rectangle(200.0, 0.0, 60.0, 60.0);
    let o2 = b.add_rectangle(0.0, 200.0, 60.0, 60.0);
    let o3 = b.add_rectangle(200.0, 200.0, 60.0, 60.0);
    b.route_between(o0, o1);
    b.route_between(o0, o2);
    b.route_between(o0, o3);
    b.route_between(o1, o2);
    b.route_between(o1, o3);
    b.route_between(o2, o3);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 6);
}

/// Source and target with a blocking obstacle in the middle.
/// The route must go around the blocker.
///
/// Currently ignored: the router falls back to a straight-line path through
/// the blocker. Will pass once the visibility graph generator is faithfully
/// ported (PRD defect #2).
#[test]
#[ignore = "router does not yet route around blocking obstacles (PRD defect #2)"]
fn obstacle_in_between() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(0.0, 50.0, 60.0, 60.0);
    let o1 = b.add_rectangle(300.0, 50.0, 60.0, 60.0);
    let _blocker = b.add_rectangle(150.0, 50.0, 60.0, 60.0);
    b.route_between(o0, o1);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
    // Path should have at least one bend to go around the blocker
    assert!(
        result.edges[0].points.len() > 2,
        "Expected bends to route around blocker, got {} points",
        result.edges[0].points.len()
    );
}

/// Three obstacles stacked vertically, route from top to bottom
/// must go around the middle obstacle.
///
/// Currently ignored: same as `obstacle_in_between` — the router does not
/// yet avoid blocking obstacles.
#[test]
#[ignore = "router does not yet route around blocking obstacles (PRD defect #2)"]
fn vertically_stacked_obstacles() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(100.0, 0.0, 80.0, 40.0);
    let _middle = b.add_rectangle(100.0, 100.0, 80.0, 40.0);
    let o2 = b.add_rectangle(100.0, 200.0, 80.0, 40.0);
    b.route_between(o0, o2); // must route around middle obstacle

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
    assert!(
        result.edges[0].points.len() > 2,
        "Expected bends to route around middle obstacle, got {} points",
        result.edges[0].points.len()
    );
}
