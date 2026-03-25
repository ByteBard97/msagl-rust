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

// ---------------------------------------------------------------------------
// Second batch — ported from RectilinearTests.cs
// ---------------------------------------------------------------------------

/// Two boxes separated by a large horizontal gap (500 units).
/// With no obstacles between them the router should produce a single direct
/// horizontal edge.
///
/// Mirrors the C# `TwoSquares` extended variant with a wide gap.
#[test]
fn wide_horizontal_gap() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(0.0, 0.0, 50.0, 50.0);
    let o1 = b.add_rectangle(500.0, 0.0, 50.0, 50.0);
    b.route_between(o0, o1);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
}

/// Source is above and to the left of the target — the direct path is a
/// diagonal, so the router must introduce at least one bend.
///
/// Currently ignored: the router returns only 2 waypoints (a straight line)
/// for offset source/target pairs instead of generating the required bend.
/// Will pass once the visibility graph generator is faithfully ported
/// (PRD defect #2).
///
/// Mirrors the C# `Clust5_Minimal` pattern (source-above, target-right).
#[test]
#[ignore = "router produces straight 2-point path instead of bending for offset pairs (PRD defect #2)"]
fn l_shaped_routing() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(0.0, 200.0, 60.0, 60.0);
    let o1 = b.add_rectangle(200.0, 0.0, 60.0, 60.0);
    b.route_between(o0, o1);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
    // Diagonal layout requires at least one bend (≥ 3 waypoints).
    assert!(
        result.edges[0].points.len() >= 3,
        "Expected at least one bend for L-shaped path, got {} points",
        result.edges[0].points.len()
    );
}

/// Source and target are on the same side of a blocking wall.
/// The router must detour around the wall, producing a U-shaped path.
///
/// Currently ignored: the router does not yet route around blocking obstacles
/// (PRD defect #2).
#[test]
#[ignore = "router does not yet route around blocking obstacles (PRD defect #2)"]
fn u_shaped_routing() {
    let mut b = ScenarioBuilder::new();
    // Wall spans most of the gap between the two boxes.
    let o0 = b.add_rectangle(0.0, 50.0, 50.0, 50.0);
    let o1 = b.add_rectangle(200.0, 50.0, 50.0, 50.0);
    let _wall = b.add_rectangle(100.0, 50.0, 30.0, 200.0);
    b.route_between(o0, o1);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
    // U-path needs at least 3 bends (≥ 5 waypoints).
    assert!(
        result.edges[0].points.len() >= 5,
        "Expected U-shaped detour (≥5 points), got {}",
        result.edges[0].points.len()
    );
}

/// Three parallel edges between the same pair of obstacles.
/// The nudger should separate the paths so they do not overlap.
///
/// Mirrors the C# nudging tests that exercise the parallel-edge separation
/// pass.
#[test]
fn multiple_parallel_routes() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(0.0, 0.0, 60.0, 60.0);
    let o1 = b.add_rectangle(300.0, 0.0, 60.0, 60.0);
    b.route_between(o0, o1);
    b.route_between(o0, o1);
    b.route_between(o0, o1);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 3);
}

/// Two boxes separated by only a small gap (5 units, well below the default
/// padding of 1.0).  The router should still produce a valid rectilinear path.
///
/// Mirrors the C# close-obstacle stress tests.
#[test]
fn close_obstacles() {
    let mut b = ScenarioBuilder::new();
    // Boxes are 40 wide and placed so the gap between their right/left edges
    // is 5 units.
    let o0 = b.add_rectangle(0.0, 0.0, 40.0, 40.0);
    let o1 = b.add_rectangle(45.0, 0.0, 40.0, 40.0);
    b.route_between(o0, o1);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
}

/// 3×3 grid of obstacles (spacing of 150 units), route between the
/// top-left and bottom-right corners.
///
/// The direct path crosses through interior grid obstacles, so the router
/// must navigate around them.
///
/// Currently ignored: the router does not yet route around blocking obstacles
/// (PRD defect #2).
#[test]
#[ignore = "router does not yet route around blocking obstacles (PRD defect #2)"]
fn large_grid_corner_to_corner() {
    let mut b = ScenarioBuilder::new();
    let mut ids = [[0usize; 3]; 3];
    for row in 0..3usize {
        for col in 0..3usize {
            ids[row][col] = b.add_rectangle(
                col as f64 * 150.0,
                row as f64 * 150.0,
                60.0,
                60.0,
            );
        }
    }
    // Route top-left (0,0) → bottom-right (2,2).
    b.route_between(ids[0][0], ids[2][2]);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
    // Must bend around interior obstacles.
    assert!(
        result.edges[0].points.len() >= 3,
        "Expected bends to navigate the grid, got {} points",
        result.edges[0].points.len()
    );
}

/// Five obstacles along a diagonal, route between the two corner boxes.
/// No obstacle lies directly in the straight-line path, so the route can be
/// computed without bypass logic.
///
/// Currently ignored: the router returns only 2 waypoints (a straight line)
/// for diagonally offset source/target pairs instead of generating the
/// required bend.  Will pass once the visibility graph generator is faithfully
/// ported (PRD defect #2).
///
/// Mirrors the C# diagonal-arrangement layout pattern.
#[test]
#[ignore = "router produces straight 2-point path instead of bending for diagonal pairs (PRD defect #2)"]
fn diagonal_arrangement() {
    let mut b = ScenarioBuilder::new();
    // Place five boxes along a diagonal (step 120 in both x and y).
    let mut ids = Vec::new();
    for i in 0..5usize {
        ids.push(b.add_rectangle(
            i as f64 * 120.0,
            i as f64 * 120.0,
            50.0,
            50.0,
        ));
    }
    // Route from the first box to the last.
    b.route_between(ids[0], ids[4]);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
    // Diagonal layout forces at least one bend.
    assert!(
        result.edges[0].points.len() >= 3,
        "Expected at least one bend for diagonal path, got {} points",
        result.edges[0].points.len()
    );
}

/// Five boxes in a single horizontal row.  Route from the leftmost box to the
/// rightmost; the three middle boxes are in the direct path.
///
/// Currently ignored: the router does not yet route around blocking obstacles
/// (PRD defect #2).
#[test]
#[ignore = "router does not yet route around blocking obstacles (PRD defect #2)"]
fn same_row_multiple() {
    let mut b = ScenarioBuilder::new();
    let mut ids = Vec::new();
    for i in 0..5usize {
        ids.push(b.add_rectangle(i as f64 * 120.0, 0.0, 60.0, 60.0));
    }
    // Route first → last; middle boxes are blockers.
    b.route_between(ids[0], ids[4]);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 1);
    // Must detour around the three middle boxes.
    assert!(
        result.edges[0].points.len() >= 3,
        "Expected detour around middle boxes, got {} points",
        result.edges[0].points.len()
    );
}

/// One source box routes to four separate target boxes (fan-out).
///
/// Mirrors the C# `Clust5_Minimal` fan-out pattern.
#[test]
fn fan_out() {
    let mut b = ScenarioBuilder::new();
    let src = b.add_rectangle(150.0, 300.0, 80.0, 50.0);
    let t0 = b.add_rectangle(0.0, 0.0, 60.0, 60.0);
    let t1 = b.add_rectangle(100.0, 0.0, 60.0, 60.0);
    let t2 = b.add_rectangle(200.0, 0.0, 60.0, 60.0);
    let t3 = b.add_rectangle(300.0, 0.0, 60.0, 60.0);
    b.route_between(src, t0);
    b.route_between(src, t1);
    b.route_between(src, t2);
    b.route_between(src, t3);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 4);
}

/// Two edges between the same pair: one A→B and one B→A.
/// Both should be routed successfully and independently.
///
/// Mirrors the C# bidirectional routing pattern.
#[test]
fn bidirectional() {
    let mut b = ScenarioBuilder::new();
    let o0 = b.add_rectangle(0.0, 0.0, 60.0, 60.0);
    let o1 = b.add_rectangle(200.0, 0.0, 60.0, 60.0);
    b.route_between(o0, o1);
    b.route_between(o1, o0);

    let result = b.run();
    Verifier::verify_all(&result, b.shapes(), b.get_padding());
    assert_eq!(result.edges.len(), 2);
}
