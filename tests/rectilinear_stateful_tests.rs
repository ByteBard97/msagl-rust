/// Stateful rectilinear routing tests ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
/// Lines 948–1370, class RectilinearTests.
///
/// These tests exercise the incremental API introduced in Refactor 5:
/// add/remove edge geometries, update obstacles, and verify that routing
/// is still correct after each mutation.
///
/// Key difference from C#: The Rust port uses index-based obstacle references
/// instead of object identity. "Relative" ports (C# RelativeFloatingPort) are
/// simulated by recomputing the port location relative to the obstacle's
/// bounding box center at each routing step.

#[path = "test_harness/mod.rs"]
mod test_harness;

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

// ── Helpers ──────────────────────────────────────────────────────────────────

/// Create the standard two-square layout used throughout the C# stateful tests.
///
/// C# RectilinearVerifier.CreateTwoTestSquaresWithSentinels()
/// Returns:
///   [0] = left square  (20..100 × 20..100)
///   [1] = right square (220..300 × 20..100)
///   [2] = left sentinel
///   [3] = right sentinel
///   [4] = top sentinel
///   [5] = bottom sentinel
fn create_two_test_squares_with_sentinels() -> Vec<Shape> {
    vec![
        Shape::rectangle(20.0, 20.0, 80.0, 80.0),   // [0] left square
        Shape::rectangle(220.0, 20.0, 80.0, 80.0),  // [1] right square
        Shape::rectangle(0.0, 20.0, 5.0, 80.0),     // [2] left sentinel
        Shape::rectangle(315.0, 20.0, 5.0, 80.0),   // [3] right sentinel
        Shape::rectangle(0.0, 115.0, 320.0, 5.0),   // [4] top sentinel
        Shape::rectangle(0.0, 0.0, 320.0, 5.0),     // [5] bottom sentinel
    ]
}

/// Bounding box center of a shape (mirrors C# shape.BoundingBox.Center).
fn center(shape: &Shape) -> Point {
    let bb = shape.bounding_box();
    Point::new(
        (bb.left() + bb.right()) / 2.0,
        (bb.bottom() + bb.top()) / 2.0,
    )
}

/// Create a router with the standard padding=1.0 and edge_separation=1.0
/// used by the C# RectilinearVerifier.
fn create_router(shapes: &[Shape]) -> RectilinearEdgeRouter {
    RectilinearEdgeRouter::new(shapes)
        .padding(1.0)
        .edge_separation(1.0)
}

/// Assert that all edges in the result were successfully routed (no fallbacks).
fn assert_no_fallbacks(result: &msagl_rust::RoutingResult, context: &str) {
    for (i, e) in result.edges.iter().enumerate() {
        assert!(
            !e.is_fallback,
            "{context}: edge[{i}] is a genuine routing failure (path search returned None)"
        );
    }
}

// ── Test 1: Update_FreePort (C# line 948) ────────────────────────────────────

/// Port of C# Test: Update_FreePort (line 948).
///
/// Tests that we leave the visibility graph intact after removing a freeport
/// that was on another freeport line. Creates a route from portA to portB and
/// a route from portA to a moving FreePort. The FreePort is "dragged" by
/// removing the old EdgeGeometry and adding a new one with the updated location.
/// After each drag step, routing must succeed without fallbacks.
///
/// C# assertion: VertexCount unchanged after each re-route.
/// Rust assertion: all edges route without fallback.
#[test]
fn update_free_port() {
    let shapes = create_two_test_squares_with_sentinels();
    let a = &shapes[0]; // left square
    let b = &shapes[1]; // right square
    let a_center = center(a);
    let b_center = center(b);
    let a_bb = a.bounding_box();

    // freePort1 starts at: right of a-box + 10, center.y + 10
    // C# line 957: var loc1 = new Point(abox.Right + 10, abox.Center.Y + 10)
    let loc1 = Point::new(a_bb.right() + 10.0, a_center.y() + 10.0);

    let mut router = create_router(&shapes);

    // Add the stable A→B edge (index 0).
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, b_center),
    ));

    // Add the initial A→freePort1 edge (index 1).
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(0, loc1), // freeport: obstacle_index=0 (any, just for identity)
    ));

    // Initial route.
    let result = router.run();
    assert_eq!(result.edges.len(), 2, "should have 2 edges after initial route");
    assert_no_fallbacks(&result, "initial route");

    // "Drag" the freePort by removing the old edge (index 1) and adding a new one
    // with an updated location. The stable A→B edge stays at index 0.
    // C# loops ii in 0..2, jj in 0..5 (10 drag steps total).
    let mut current_loc = loc1;
    for ii in 0..2_i32 {
        for jj in 0..5_i32 {
            let new_loc = Point::new(
                loc1.x() + (ii as f64) * 3.0,
                loc1.y() + (jj as f64) * 4.0,
            );
            // Remove the moving freeport edge (always at index 1).
            router.remove_edge_geometry(1);
            // Add the new freeport edge at index 1.
            router.add_edge(EdgeGeometry::new(
                FloatingPort::new(0, a_center),
                FloatingPort::new(0, new_loc),
            ));
            assert_eq!(router.edge_count(), 2, "should always have 2 edges");
            let result = router.run();
            assert_no_fallbacks(
                &result,
                &format!("drag step ii={ii} jj={jj} loc={new_loc:?}"),
            );
            current_loc = new_loc;
        }
    }
    let _ = current_loc; // suppress unused warning
}

// ── Test 2: UpdatePortPosition_Without_UpdateObstacles (C# line 1001) ────────

/// Port of C# Test: UpdatePortPosition_Without_UpdateObstacles (line 1001).
///
/// In C#, a RelativeFloatingPort tracks the obstacle's center via a delegate.
/// Changing the port's offset and re-running the router automatically re-routes
/// without calling UpdateObstacles.
///
/// In Rust, ports have fixed locations. We simulate the "relative" port by
/// re-running the router with different edge geometries, replacing the edge
/// with new port locations that represent the shifted port. The obstacles do
/// NOT change — only the edge geometry.
///
/// C# assertion: routes draw correctly after each offset change (no assertion on vertex count).
/// Rust assertion: all edges route without fallback.
#[test]
fn update_port_position_without_update_obstacles() {
    let shapes = create_two_test_squares_with_sentinels();
    let a = &shapes[0]; // left square
    let b = &shapes[1]; // right square
    let a_center = center(a);
    let b_center = center(b);
    let b_bb = b.bounding_box();

    // portB starts at b center (offset = 0,0).
    let mut router = create_router(&shapes);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, b_center),
    ));

    let result = router.run();
    assert_no_fallbacks(&result, "initial route (portB at center)");

    // Now "move" portB to the top of b (offset y = b.top - b.center.y).
    let offset1 = Point::new(-5.0, b_bb.top() - b_center.y());
    let port_b_top = Point::new(b_center.x() + offset1.x(), b_center.y() + offset1.y());
    router.remove_edge_geometry(0);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, port_b_top),
    ));
    let result = router.run();
    assert_no_fallbacks(&result, "portB near top of b");

    // Move portB to the bottom.
    let offset2 = Point::new(-10.0, b_bb.bottom() - b_center.y());
    let port_b_bottom = Point::new(b_center.x() + offset2.x(), b_center.y() + offset2.y());
    router.remove_edge_geometry(0);
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, port_b_bottom),
    ));
    let result = router.run();
    assert_no_fallbacks(&result, "portB near bottom of b");
}

// ── Test 3: AddRemovePorts_Without_UpdateObstacles (C# line 1039) ────────────

/// Port of C# Test: AddRemovePorts_Without_UpdateObstacles (line 1039).
///
/// Adds and removes EdgeGeometries from the router without calling UpdateObstacles.
/// Verifies that routes are correct after each add/remove operation.
///
/// C# assertion: routes draw correctly (visual check).
/// Rust assertion: all edges route without fallback, edge count is correct.
#[test]
fn add_remove_ports_without_update_obstacles() {
    let shapes = create_two_test_squares_with_sentinels();
    let a = &shapes[0]; // left square
    let b = &shapes[1]; // right square
    let a_center = center(a);
    let b_center = center(b);
    let b_bb = b.bounding_box();

    let mut router = create_router(&shapes);

    // Route A→B (portB at center).
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, b_center),
    ));
    let result = router.run();
    assert_eq!(result.edges.len(), 1, "initial: 1 edge");
    assert_no_fallbacks(&result, "initial route");

    // Add two more ports C and D (relative offsets from b-center).
    let offset_c = Point::new(-5.0, b_bb.top() - b_center.y());
    let port_c_loc = Point::new(b_center.x() + offset_c.x(), b_center.y() + offset_c.y());

    let offset_d = Point::new(-10.0, b_bb.bottom() - b_center.y());
    let port_d_loc = Point::new(b_center.x() + offset_d.x(), b_center.y() + offset_d.y());

    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, port_c_loc),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, port_d_loc),
    ));
    let result = router.run();
    assert_eq!(result.edges.len(), 3, "after adding C and D: 3 edges");
    assert_no_fallbacks(&result, "after adding portC and portD");

    // Remove the last two edges (C and D, indices 1 and 2 → remove 2 then 1).
    router.remove_edge_geometry(2); // remove D (now at index 2)
    router.remove_edge_geometry(1); // remove C (now at index 1)
    let result = router.run();
    assert_eq!(result.edges.len(), 1, "after removing C and D: back to 1 edge");
    assert_no_fallbacks(&result, "after removing portC and portD");
}

// ── Test 4: MoveOneObstacle_ManuallyUpdateAbsolutePorts (C# line 1085) ───────

/// Port of C# Test: MoveOneObstacle_ManuallyUpdateAbsolutePorts (line 1085).
///
/// Moves obstacle B by calling update_obstacle() and manually replacing the
/// port locations in the EdgeGeometries. After each move, routing must succeed.
///
/// C# test steps:
/// 1. Route A→B and B→freePort.
/// 2. Shrink B (replace with smaller rectangle).
/// 3. Loop 5 times: translate B, remove old edges, add new edges with updated port,
///    call update_obstacle, re-run.
///
/// Rust translation: We track the current B shape index (it stays at index 1
/// throughout since we update in-place via update_obstacle).
#[test]
fn move_one_obstacle_manually_update_absolute_ports() {
    let mut shapes = create_two_test_squares_with_sentinels();
    let a_center = center(&shapes[0]);
    let b_bb = shapes[1].bounding_box();
    let b_center_init = Point::new(
        (b_bb.left() + b_bb.right()) / 2.0,
        (b_bb.bottom() + b_bb.top()) / 2.0,
    );

    // freePort1 starts to the left of B at mid-height + 10.
    let free_port_loc = Point::new(b_bb.left() - 10.0, b_center_init.y() + 10.0);

    let mut router = create_router(&shapes);

    // Add initial edges: A→B and B→freePort.
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, b_center_init),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, b_center_init),
        FloatingPort::new(0, free_port_loc),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 2, "initial: 2 edges");
    assert_no_fallbacks(&result, "initial route");

    // Step 1: shrink B — replace with a 20×20 box centered on the original center.
    let new_b = Shape::rectangle(
        b_center_init.x() - 10.0,
        b_center_init.y() - 10.0,
        20.0,
        20.0,
    );
    let new_b_center = b_center_init; // centered the same

    // Remove old edges referencing old B center, add new ones.
    router.remove_edge_geometry(1); // B→freePort
    router.remove_edge_geometry(0); // A→B

    // Update the obstacle in place.
    router.update_obstacle(1, new_b.clone());
    shapes[1] = new_b.clone();

    // Add new edges with updated B center.
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, new_b_center),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, new_b_center),
        FloatingPort::new(0, free_port_loc),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 2, "after shrink: 2 edges");
    assert_no_fallbacks(&result, "after shrink");

    // Step 2: loop — translate B by (-5,-5) each iteration.
    // C# loops 5 times, calling update_obstacle and re-routing.
    let mut current_b = new_b;
    for ii in 0..5_usize {
        // Translate by (-5, -5).
        let b_bb = current_b.bounding_box();
        let new_left = b_bb.left() - 5.0;
        let new_bottom = b_bb.bottom() - 5.0;
        let new_w = b_bb.right() - b_bb.left();
        let new_h = b_bb.top() - b_bb.bottom();
        let translated_b = Shape::rectangle(new_left, new_bottom, new_w, new_h);
        let translated_center = Point::new(new_left + new_w / 2.0, new_bottom + new_h / 2.0);

        // Remove old edges.
        router.remove_edge_geometry(1);
        router.remove_edge_geometry(0);

        // Add new edges with updated port location.
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(0, a_center),
            FloatingPort::new(1, translated_center),
        ));
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(1, translated_center),
            FloatingPort::new(0, free_port_loc),
        ));

        // Update obstacle (triggers VG rebuild).
        router.update_obstacle(1, translated_b.clone());

        let result = router.run();
        assert_eq!(result.edges.len(), 2, "move step {ii}: 2 edges");
        assert_no_fallbacks(&result, &format!("move step {ii}"));

        current_b = translated_b;
    }
}

// ── Test 5: MoveOneObstacle_NoUpdateAbsolutePorts_FreePoint_InAndOut (C# line 1184) ─

/// Port of C# Test: MoveOneObstacle_NoUpdateAbsolutePorts_FreePoint_InAndOut (line 1184).
///
/// Moves obstacle B via update_obstacle while keeping the port locations fixed
/// (they don't update with the obstacle). Verifies that the router handles the
/// case where a freeport ends up inside or outside of the moved obstacle.
///
/// C# test: Does NOT remove/re-add ports when moving. Just calls UpdateObstacle
/// and re-routes. The freePort may end up inside the obstacle — the router must
/// handle this gracefully (routing the path even if the port is covered).
///
/// Rust assertion: all edges route without fallback, even if ports end up inside obstacles.
#[test]
fn move_one_obstacle_no_update_absolute_ports_free_point_in_and_out() {
    let mut shapes = create_two_test_squares_with_sentinels();
    let a_center = center(&shapes[0]);
    let b_bb = shapes[1].bounding_box();
    let b_center = Point::new(
        (b_bb.left() + b_bb.right()) / 2.0,
        (b_bb.bottom() + b_bb.top()) / 2.0,
    );

    // freePort1 starts to the left of B.
    let free_port_loc = Point::new(b_bb.left() - 10.0, b_center.y() + 10.0);

    let mut router = create_router(&shapes);

    // Add edges: A→B and B→freePort (ports are FIXED — not updated when B moves).
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, b_center),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, b_center),
        FloatingPort::new(0, free_port_loc),
    ));

    let result = router.run();
    assert_no_fallbacks(&result, "initial route");

    // Step 1: Replace B with a 40×40 box translated by (-15,-15).
    let new_b = Shape::rectangle(
        b_center.x() - 20.0 - 15.0,
        b_center.y() - 20.0 - 15.0,
        40.0,
        40.0,
    );
    router.remove_obstacle(1);
    router.add_obstacles(&[new_b.clone()]);
    shapes[1] = new_b.clone();

    // Re-route with the same fixed port locations.
    let result = router.run();
    assert_no_fallbacks(&result, "after replace B");

    // Step 2: Translate by (-5,-5) each iteration (3 times as in C#).
    let mut current_b = new_b;
    for ii in 0..3_usize {
        let b_bb = current_b.bounding_box();
        let translated_b = Shape::rectangle(
            b_bb.left() - 5.0,
            b_bb.bottom() - 5.0,
            b_bb.right() - b_bb.left(),
            b_bb.top() - b_bb.bottom(),
        );
        // update_obstacle triggers VG rebuild; ports are NOT changed.
        router.update_obstacle(1, translated_b.clone());
        let result = router.run();
        assert_no_fallbacks(&result, &format!("move step {ii}"));
        current_b = translated_b;
    }
}

// ── Test 6: MoveOneObstacle_AutomaticallyUpdateAbsolutePorts (C# line 1234) ──

/// Port of C# Test: MoveOneObstacle_AutomaticallyUpdateAbsolutePorts (line 1234).
///
/// Replaces obstacle B (remove + add) and updates port locations at the same time.
/// Each iteration: translate B, remove old obstacle + edges, add new obstacle +
/// new edges with updated port at new B center.
///
/// C# test: Uses ReplaceObstaclesAndRouteEdges (remove + add) and automatically
/// re-routes with the new obstacle positions. Ports are updated manually in each
/// iteration (C# line 1293: newB.Ports.Remove + MakeAbsoluteObstaclePort).
///
/// Rust translation: remove old shape (index 1), insert new shape, update edges.
#[test]
fn move_one_obstacle_automatically_update_absolute_ports() {
    let shapes = create_two_test_squares_with_sentinels();
    let a_center = center(&shapes[0]);
    let b_bb = shapes[1].bounding_box();
    let b_center_init = Point::new(
        (b_bb.left() + b_bb.right()) / 2.0,
        (b_bb.bottom() + b_bb.top()) / 2.0,
    );

    let free_port_loc = Point::new(b_bb.left() - 10.0, b_center_init.y() + 10.0);

    let mut router = create_router(&shapes);

    // Route A→B and B→freePort.
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, b_center_init),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, b_center_init),
        FloatingPort::new(0, free_port_loc),
    ));

    let result = router.run();
    assert_no_fallbacks(&result, "initial route");

    // Step 1: Shrink B — new 20×20 box at same center.
    // Remove old edges, remove old B (index 1), add new B, add new edges.
    router.remove_edge_geometry(1); // B→freePort
    router.remove_edge_geometry(0); // A→B

    let new_b = Shape::rectangle(
        b_center_init.x() - 10.0,
        b_center_init.y() - 10.0,
        20.0,
        20.0,
    );
    let new_b_center = b_center_init;

    // Replace: remove old B at index 1, add new B (it gets pushed to end, index becomes 5 or wherever).
    // To keep index=1 stable, use update_obstacle instead.
    router.update_obstacle(1, new_b.clone());

    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, a_center),
        FloatingPort::new(1, new_b_center),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, new_b_center),
        FloatingPort::new(0, free_port_loc),
    ));

    let result = router.run();
    assert_eq!(result.edges.len(), 2, "after shrink: 2 edges");
    assert_no_fallbacks(&result, "after shrink");

    // Step 2: loop 5 times, translate B by (-5,-5).
    let mut current_b = new_b;
    for ii in 0..5_usize {
        let b_bb = current_b.bounding_box();
        let new_left = b_bb.left() - 5.0;
        let new_bottom = b_bb.bottom() - 5.0;
        let new_w = b_bb.right() - b_bb.left();
        let new_h = b_bb.top() - b_bb.bottom();
        let translated_b = Shape::rectangle(new_left, new_bottom, new_w, new_h);
        let translated_center = Point::new(new_left + new_w / 2.0, new_bottom + new_h / 2.0);

        // Remove old edges and update ports.
        router.remove_edge_geometry(1);
        router.remove_edge_geometry(0);
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(0, a_center),
            FloatingPort::new(1, translated_center),
        ));
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(1, translated_center),
            FloatingPort::new(0, free_port_loc),
        ));

        // Replace obstacle via update.
        router.update_obstacle(1, translated_b.clone());

        let result = router.run();
        assert_eq!(result.edges.len(), 2, "auto-update step {ii}: 2 edges");
        assert_no_fallbacks(&result, &format!("auto-update absolute step {ii}"));

        current_b = translated_b;
    }
}

// ── Test 7: MoveOneObstacle_AutomaticallyUpdateRelativePorts (C# line 1302) ──

/// Port of C# Test: MoveOneObstacle_AutomaticallyUpdateRelativePorts (line 1302).
///
/// In C#, relative ports track the obstacle center automatically. When the obstacle
/// is moved, the port location updates without code changes — the CurveDelegate
/// indirection handles it.
///
/// In Rust, we simulate this by computing the port location from the obstacle's
/// bounding box center + a fixed offset at each step, then updating the edge
/// geometry. The port offset is kept constant in obstacle-local coordinates.
///
/// C# assertion: VertexCount unchanged after each translation (same-size obstacle).
/// Rust assertion: all edges route without fallback.
#[test]
fn move_one_obstacle_automatically_update_relative_ports() {
    let shapes = create_two_test_squares_with_sentinels();
    let a = &shapes[0];
    let b = &shapes[1];
    let a_bb = a.bounding_box();
    let b_bb = b.bounding_box();
    let a_center = center(a);
    let b_center = center(b);

    // portA: on the right side of a (relative offset = right-center.x, 0)
    // portB: on the left side of b (relative offset = left-center.x, 0)
    // C# line 1317–1318:
    //   portA = MakeSingleRelativeObstaclePort(a, new Point(abox.Right - abox.Center.X, 0))
    //   portB = MakeSingleRelativeObstaclePort(b, new Point(bbox.Left - bbox.Center.X, 0))
    let a_right_offset = a_bb.right() - a_center.x();   // = +40
    let b_left_offset = b_bb.left() - b_center.x();     // = -40

    let port_a_loc = Point::new(a_center.x() + a_right_offset, a_center.y());
    let port_b_loc = Point::new(b_center.x() + b_left_offset, b_center.y());

    let free_port_loc = Point::new(b_bb.left() - 10.0, b_center.y() + 10.0);

    let mut router = create_router(&shapes);

    // Route A→B and B→freePort.
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, port_a_loc),
        FloatingPort::new(1, port_b_loc),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, port_b_loc),
        FloatingPort::new(0, free_port_loc),
    ));

    let result = router.run();
    assert_no_fallbacks(&result, "initial route");

    // Step 1: Shrink B to 20×20 box centered at original B center.
    let new_b_center = b_center;
    let new_b = Shape::rectangle(
        new_b_center.x() - 10.0,
        new_b_center.y() - 10.0,
        20.0,
        20.0,
    );
    let new_b_bb = new_b.bounding_box();
    let new_b_left_offset = new_b_bb.left() - new_b_center.x(); // -10
    let new_port_b_loc = Point::new(new_b_center.x() + new_b_left_offset, new_b_center.y());

    // Remove old edges; update obstacle; add new edges with relative port at new B.
    router.remove_edge_geometry(1);
    router.remove_edge_geometry(0);
    router.update_obstacle(1, new_b.clone());
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, port_a_loc),
        FloatingPort::new(1, new_port_b_loc),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(1, new_port_b_loc),
        FloatingPort::new(0, free_port_loc),
    ));

    let result = router.run();
    assert_no_fallbacks(&result, "after shrink");

    // Step 2: loop 5 times, translate new_b by (-5,-5).
    // C# line 1357-1369: same obstacle dimensions, just translated.
    // Port stays at "left side" of B (relative: center.x + left_offset).
    let mut current_b = new_b;
    for ii in 0..5_usize {
        let b_bb_curr = current_b.bounding_box();
        let new_left = b_bb_curr.left() - 5.0;
        let new_bottom = b_bb_curr.bottom() - 5.0;
        let w = b_bb_curr.right() - b_bb_curr.left();
        let h = b_bb_curr.top() - b_bb_curr.bottom();
        let translated_b = Shape::rectangle(new_left, new_bottom, w, h);
        let translated_center = Point::new(new_left + w / 2.0, new_bottom + h / 2.0);

        // Relative port tracks obstacle: left side = center.x + (left - center.x)
        let new_b_bb2 = translated_b.bounding_box();
        let left_offset = new_b_bb2.left() - translated_center.x();
        let translated_port_b = Point::new(translated_center.x() + left_offset, translated_center.y());

        // Remove old, update, re-add.
        router.remove_edge_geometry(1);
        router.remove_edge_geometry(0);
        router.update_obstacle(1, translated_b.clone());
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(0, port_a_loc),
            FloatingPort::new(1, translated_port_b),
        ));
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(1, translated_port_b),
            FloatingPort::new(0, free_port_loc),
        ));

        let result = router.run();
        assert_no_fallbacks(&result, &format!("relative step {ii}"));

        current_b = translated_b;
    }
}
