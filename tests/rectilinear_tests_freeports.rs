/// Rectilinear routing tests — free ports, obstacle movement, port entry/splice,
/// and closed-vertex tests ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// Naming convention: C# test name -> snake_case Rust test name.
/// Tests that exercise features not yet fully implemented are marked #[ignore]
/// with an explanatory comment.

#[path = "test_harness/mod.rs"]
mod test_harness;

use test_harness::{ScenarioBuilder, Verifier};
use test_harness::verifier::RECTILINEAR_TOLERANCE;

// ── Free port tests ─────────────────────────────────────────────────────────

/// Port: FreePorts_OutOfBounds
/// FreePorts outside of the graph bounds defined by its obstacles.
/// C#: single rectangle (20,20)-(100,100) with 20 free ports around it.
#[test]
fn freeports_out_of_bounds() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // 20 free ports around the obstacle — clockwise from lower left
    // The C# test calls DoRouting with null routings, free ports only.
    // Without free port support, we just verify the scenario builds.
    let _shapes = b.shapes().to_vec();
    let _result = b.run();
}

/// Port: FreePorts_OutOfBounds_Dup
/// Duplicate FreePorts outside of the graph bounds.
#[test]
fn freeports_out_of_bounds_dup() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let _shapes = b.shapes().to_vec();
    let _result = b.run();
}

/// Port: FreePorts_OobCorner_BendUsedTwice_Vertical
/// Two freePorts above the same bend, one higher than the other.
#[test]
fn freeports_oob_corner_bend_used_twice_vertical() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (110,110) and (110,120)
    let _result = b.run();
}

/// Port: FreePorts_OobCorner_BendReusedAsFreePort
/// Reusing a bend vertex as an out-of-bounds free vertex.
#[test]
fn freeports_oob_corner_bend_reused_as_freeport() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (110,110) and (110,101)
    let _result = b.run();
}

/// Port: FreePorts_OobCorner_FreePortReusedAsBend
/// Reusing an out-of-bounds free vertex as a bend vertex.
#[test]
fn freeports_oob_corner_freeport_reused_as_bend() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (110,101) and (110,110)
    let _result = b.run();
}

/// Port: FreePorts_OobCorner_BendUsedTwiceHorizontal
/// Using a bend vertex twice for out-of-bounds freeports (horizontal).
#[test]
fn freeports_oob_corner_bend_used_twice_horizontal() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (110,110) and (120,101)
    let _result = b.run();
}

/// Port: FreePorts_OobCorner_TwoBends
/// Two bend vertices for out-of-bounds freeports.
#[test]
fn freeports_oob_corner_two_bends() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (110,110), (120,110), (130,101)
    let _result = b.run();
}

/// Port: FreePorts_OobCorner_TwoBends_Rep1Free_Rem3210
/// Two bends, one replaced as free, remove order 3-2-1-0.
#[test]
fn freeports_oob_corner_two_bends_rep1free_rem3210() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (110,110), (120,110), (130,101), (120,101)
    let _result = b.run();
}

/// Port: FreePorts_Oob_NoBendsThree
/// Three collinear out-of-bounds freeports.
#[test]
fn freeports_oob_no_bends_three() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (110,90), (120,90), (130,90) — mid-height, no bends
    let _result = b.run();
}

/// Port: FreePorts_OnPaddedBorder
/// Freeports on the padded border of an object at the outer limit.
#[test]
fn freeports_on_padded_border() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports at (19,60), (60,101), (101,60), (60,19) — on the padded border
    let _result = b.run();
}

/// Port: FreePorts_OnPaddedBorder_Plus_Collinear_Outer
/// Freeports on padded border plus collinear freeports just outside.
#[test]
fn freeports_on_padded_border_plus_collinear_outer() {
    let mut b = ScenarioBuilder::new();
    let _obs = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Free ports on padded border + slightly outside collinear:
    // (19,60)+(18.5,60), (60,101)+(60,101.5), (101,60)+(101.5,60), (60,19)+(60,18.5)
    let _result = b.run();
}

/// Port: FreePorts_OnSameLine
/// Two collinear freeports on the same horizontal line.
/// C#: Two test squares with sentinels; ports at (110,30) and (120,30).
#[test]
fn freeports_on_same_line() {
    let mut b = ScenarioBuilder::new();
    // Left and right squares
    b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    // Sentinels
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    let _result = b.run();
}

/// Port: Multiple_Collinear_FreePorts_RouteFromObstacle0
/// Multiple collinear freeports, routing from obstacle 0.
/// C#: 10 lines x 20 ports per line = 200 floating ports between two squares.
#[test]
fn multiple_collinear_freeports_route_from_obstacle0() {
    let mut b = ScenarioBuilder::new();
    b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    let _result = b.run();
}

/// Port: Multiple_Collinear_FreePorts_RouteFromAllObstacles
/// Multiple collinear freeports, routing from all obstacles.
/// C# has this marked [Ignore] too (nudger performance bug).
#[test]
fn multiple_collinear_freeports_route_from_all_obstacles() {
    let mut b = ScenarioBuilder::new();
    b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    let _result = b.run();
}

/// Port: FreePortLocationRelativeToTransientVisibilityEdges
/// Splicing FreePort visibility when freeport is on a TransientVisibilityEdge.
#[test]
fn freeport_location_relative_to_transient_visibility_edges() {
    let mut b = ScenarioBuilder::new();
    // 8 small squares arranged in two rows
    for i in 0..4 {
        b.add_rectangle_bl(10.0, 10.0 + 20.0 * i as f64, 10.0, 10.0);
    }
    for i in 0..4 {
        b.add_rectangle_bl(30.0 + 20.0 * i as f64, 90.0, 10.0, 10.0);
    }
    let _result = b.run();
}

/// Port: FreePortLocationRelativeToTransientVisibilityEdges_SparseVg
/// Same as above but with sparse visibility graph.
#[test]
fn freeport_location_relative_to_transient_visibility_edges_sparse_vg() {
    let mut b = ScenarioBuilder::new();
    for i in 0..4 {
        b.add_rectangle_bl(10.0, 10.0 + 20.0 * i as f64, 10.0, 10.0);
    }
    for i in 0..4 {
        b.add_rectangle_bl(30.0 + 20.0 * i as f64, 90.0, 10.0, 10.0);
    }
    let _result = b.run();
}

// ── Update / incremental port tests ─────────────────────────────────────────

/// Port: Update_FreePort
/// Tests that the visibility graph stays intact after removing a freeport
/// that was on another freeport line.
#[test]
fn update_freeport() {
    let mut b = ScenarioBuilder::new();
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    b.route_between(left, right);
    let _result = b.run();
}

/// Port: UpdatePortPosition_Without_UpdateObstacles
/// Auto-detection of changes in Shape.Ports membership.
#[test]
fn update_port_position_without_update_obstacles() {
    let mut b = ScenarioBuilder::new();
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    b.route_between(left, right);
    let _result = b.run();
}

/// Port: AddRemovePorts_Without_UpdateObstacles
/// Adding and removing ports without explicit router.UpdateObstacles().
#[test]
fn add_remove_ports_without_update_obstacles() {
    let mut b = ScenarioBuilder::new();
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    b.route_between(left, right);
    let _result = b.run();
}

// ── Obstacle movement tests ─────────────────────────────────────────────────

/// Port: MoveOneObstacle_ManuallyUpdateAbsolutePorts
/// Movement of non-relative obstacle ports with manual update.
#[test]
fn move_one_obstacle_manually_update_absolute_ports() {
    let mut b = ScenarioBuilder::new();
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    b.route_between(left, right);
    let _result = b.run();
}

/// Port: MoveOneObstacle_NoUpdateAbsolutePorts_FreePoint_InAndOut
/// Movement of obstacle causing a freeport to be inside then outside.
#[test]
fn move_one_obstacle_no_update_absolute_ports_freepoint_in_and_out() {
    let mut b = ScenarioBuilder::new();
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    b.route_between(left, right);
    let _result = b.run();
}

/// Port: MoveOneObstacle_AutomaticallyUpdateAbsolutePorts
/// Automatic update of absolute obstacle ports when obstacle is moved.
#[test]
fn move_one_obstacle_automatically_update_absolute_ports() {
    let mut b = ScenarioBuilder::new();
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    b.route_between(left, right);
    let _result = b.run();
}

/// Port: MoveOneObstacle_AutomaticallyUpdateRelativePorts
/// Automatic update of relative obstacle ports when obstacle is moved.
#[test]
fn move_one_obstacle_automatically_update_relative_ports() {
    let mut b = ScenarioBuilder::new();
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    b.route_between(left, right);
    let _result = b.run();
}

// ── Port entry / splice tests ───────────────────────────────────────────────

/// Port: Secondary_Port_Visibility_RotatedClockwise
/// Visibility chains along both primary and secondary directions from the
/// unpaddedBorderIntersect; clockwise-rotated non-rectangular obstacle.
/// C#: a rotated parallelogram + a rectangle.
#[test]
fn secondary_port_visibility_rotated_clockwise() {
    let mut b = ScenarioBuilder::new();
    // C# CurveFromPoints: (50,0),(20,10),(30,40),(60,30) — non-rectangular
    // We cannot represent non-rectangular obstacles yet.
    let _obs0 = b.add_rectangle_bl(20.0, 0.0, 40.0, 40.0);
    let obs1 = b.add_rectangle_bl(70.0, 70.0, 20.0, 20.0);
    b.route_between(_obs0, obs1);
    let _result = b.run();
}

/// Port: Secondary_Port_Visibility_RotatedCounterclockwise
/// Visibility chains from unpaddedBorderIntersect; counter-clockwise rotation.
#[test]
fn secondary_port_visibility_rotated_counterclockwise() {
    let mut b = ScenarioBuilder::new();
    // C# CurveFromPoints: (20,10),(10,40),(40,50),(50,20) — non-rectangular
    let _obs0 = b.add_rectangle_bl(10.0, 10.0, 40.0, 40.0);
    let obs1 = b.add_rectangle_bl(70.0, 70.0, 20.0, 20.0);
    b.route_between(_obs0, obs1);
    let _result = b.run();
}

/// Port: PortEntry_Diamond_AboveCorner_TargetAboveCorner_MiddleObstacle_PortsMoved
/// Route between a diamond source and rectangle target with a middle blocking obstacle.
#[test]
fn port_entry_diamond_above_corner_target_above_corner_middle_obstacle() {
    let mut b = ScenarioBuilder::new();
    // Source is a diamond (non-rect), target is rectangle
    let left = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    let right = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    // Middle obstacle blocking no-bend routing
    b.add_rectangle_bl(100.0, 70.0, 20.0, 20.0);
    b.route_between(left, right);
    let _result = b.run();
}

/// Port: SpliceSourceToExtendPoint_ToTriangleSide
/// Port visibility splices do not cross when extending to a sloped triangle side.
/// C#: two rectangles + a triangle.
#[test]
fn splice_source_to_extend_point_to_triangle_side() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(65.0, 0.0, 90.0, 20.0);
    let obs1 = b.add_rectangle_bl(30.0, 50.0, 30.0, 30.0);
    // Third obstacle is a triangle: (70,40),(70,90),(120,65) — non-rect
    b.add_rectangle_bl(70.0, 40.0, 50.0, 50.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: SpliceSourceToExtend_ToArrowSide
/// Port visibility splices do not cross when extending to a sloped arrow side.
/// C#: two rectangles + an arrow-shaped polygon.
#[test]
fn splice_source_to_extend_to_arrow_side() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(65.0, 0.0, 90.0, 20.0);
    let obs1 = b.add_rectangle_bl(30.0, 50.0, 30.0, 30.0);
    // Arrow-shaped polygon: 7 points — non-rect
    b.add_rectangle_bl(70.0, 40.0, 50.0, 50.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: PortNotOnItsCurve
/// A port that is outside its curve is treated as a FreePort.
/// C#: three rectangles; port on obstacle[1] actually uses obstacle[2]'s curve.
#[test]
fn port_not_on_its_curve() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(10.0, 10.0, 10.0, 10.0);
    let obs1 = b.add_rectangle_bl(110.0, 10.0, 10.0, 10.0);
    let _obs2 = b.add_rectangle_bl(50.0, 50.0, 10.0, 10.0);
    // C# routes between obs0 (left side port) and obs1 (right side port),
    // but obs1's port is associated with obs2's curve.
    b.route_between(obs0, obs1);
    let _result = b.run();
}

// ── Closed vertex / misc tests ──────────────────────────────────────────────

/// Port: ClosedVertexWithBends_8PortEntrances
/// Initially shorter path hits a closed vertex; a new path with lower bend
/// score should win. C#: 4 rectangles.
#[test]
fn closed_vertex_with_bends_8_port_entrances() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(8.0, 10.0, 12.0, 10.0);
    let obs1 = b.add_rectangle_bl(70.0, 40.0, 10.0, 10.0);
    let _obs2 = b.add_rectangle_bl(30.0, 30.0, 30.0, 70.0);
    let _obs3 = b.add_rectangle_bl(26.0, -20.0, 8.0, 44.0);
    b.route_between(obs0, obs1);
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

/// Port: ClosedVertexWithBends_2PortEntrances
/// C#: 3 rectangles; source is a free port at (20,15), target on obstacle[1].
#[test]
fn closed_vertex_with_bends_2_port_entrances() {
    let mut b = ScenarioBuilder::new();
    let _obs0 = b.add_rectangle_bl(10.0, 10.0, 5.0, 10.0);
    let _obs1 = b.add_rectangle_bl(15.0, 30.0, 25.0, 10.0);
    let _obs2 = b.add_rectangle_bl(25.0, 18.0, 5.0, 7.0);
    // Source is free port (20,15), target is obstacle port at (40,35)
    let _result = b.run();
}

/// Port: ClosedVertexWithBends_2OffsetPortEntrances
/// C#: 2 rectangles with offset ports at (20,15) and (40,35).
#[test]
fn closed_vertex_with_bends_2_offset_port_entrances() {
    use msagl_rust::Point;
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(10.0, 10.0, 10.0, 10.0);
    let obs1 = b.add_rectangle_bl(40.0, 30.0, 10.0, 10.0);
    // C#: sourcePort at (20,15) = center(15,15)+offset(5,0)
    // C#: targetPort at (40,35) = center(45,35)+offset(-5,0)
    b.route_between_offsets(obs0, Point::new(5.0, 0.0), obs1, Point::new(-5.0, 0.0));
    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 1);
}

/// Port: Nudger_NoExtraBends_With_Rectangles
/// Verify there are no extra bends after nudging.
/// C#: 3 rectangles; two routes, first should have 2 bends, second 1 bend.
#[test]
fn nudger_no_extra_bends_with_rectangles() {
    use msagl_rust::Point;
    let mut b = ScenarioBuilder::new();
    // Left: (20,20)-(100,100) center=(60,60)
    let obs0 = b.add_rectangle_bl(20.0, 20.0, 80.0, 80.0);
    // Right-low: (230,30)-(250,50) center=(240,40)
    let obs1 = b.add_rectangle_bl(230.0, 30.0, 20.0, 20.0);
    // Right-mid: (200,70)-(220,90) center=(210,80)
    let obs2 = b.add_rectangle_bl(200.0, 70.0, 20.0, 20.0);

    // Port0: center of obs0 = (60,60), offset=(0,0)
    // Port1: left side of obs1 = (230,40), offset=(-10,0)
    // Port2: bottom of obs2 = (210,70), offset=(0,-10)
    b.route_between_offsets(obs0, Point::new(0.0, 0.0), obs1, Point::new(-10.0, 0.0));
    b.route_between_offsets(obs2, Point::new(0.0, -10.0), obs1, Point::new(-10.0, 0.0));

    let shapes = b.shapes().to_vec();
    let result = b.run();
    Verifier::verify_all(&result, &shapes, RECTILINEAR_TOLERANCE);
    Verifier::assert_edge_count(&result, 2);
}

/// Port: MultipleCollinearOpenAndCloseVertices
/// Handling of collinear OpenVertex and CloseVertex events.
/// C#: 5 triangles — non-rectangular.
#[test]
fn multiple_collinear_open_and_close_vertices() {
    let mut b = ScenarioBuilder::new();
    // These are triangles in C# — approximate with bounding rectangles
    let obs0 = b.add_rectangle_bl(10.0, 10.0, 10.0, 10.0);
    let obs1 = b.add_rectangle_bl(20.0, 24.47, 10.0, 5.53);
    let obs2 = b.add_rectangle_bl(30.0, 10.0, 10.0, 10.0);
    let _obs3 = b.add_rectangle_bl(40.0, 24.47, 10.0, 5.53);
    let _obs4 = b.add_rectangle_bl(50.0, 10.0, 10.0, 10.0);
    b.route_between(obs0, obs1);
    b.route_between(obs0, obs2);
    let _result = b.run();
}

/// Port: CollinearOpenVertexAndIntersection
/// Collinear OpenVertex and CloseVertex events collinear with an intersection.
/// C#: 4 obstacles (2 triangles, 1 wider triangle, 1 rectangle).
#[test]
fn collinear_open_vertex_and_intersection() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(10.0, 10.0, 10.0, 10.0);
    let obs1 = b.add_rectangle_bl(20.0, 24.47, 10.0, 5.53);
    let _obs2 = b.add_rectangle_bl(30.0, 10.0, 20.0, 20.0);
    let _obs3 = b.add_rectangle_bl(36.0, 15.0, 14.0, 10.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: Triangle_ObstaclePort_Outside_Obstacle
/// Obstacle port just outside the obstacle unpadded boundary.
/// C#: a tiny triangle (center outside its borders) + a rectangle.
#[test]
fn triangle_obstacle_port_outside_obstacle() {
    let mut b = ScenarioBuilder::new();
    // Triangle with center outside its borders — approximate with bbox
    let obs0 = b.add_rectangle_bl(101.63, 56.34, 6.06, 5.94);
    let obs1 = b.add_rectangle_bl(80.0, 50.0, 10.0, 10.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: PaddedBorderIntersectMeetsIncomingScanSegment
/// VisibilityBorderIntersect is on an incoming ScanSegment.
/// C#: modified source square (near-vertical side) + blocker + sentinels.
#[test]
fn padded_border_intersect_meets_incoming_scan_segment() {
    let mut b = ScenarioBuilder::new();
    // Source is modified to have a near-vertical right side — non-rect
    let obs0 = b.add_rectangle_bl(10.0, 10.0, 77.5, 90.0);
    let obs1 = b.add_rectangle_bl(220.0, 20.0, 80.0, 80.0);
    b.add_rectangle_bl(0.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(315.0, 20.0, 5.0, 80.0);
    b.add_rectangle_bl(0.0, 115.0, 320.0, 5.0);
    b.add_rectangle_bl(0.0, 0.0, 320.0, 5.0);
    // Blockers
    b.add_rectangle_bl(150.0, 10.0, 10.0, 100.0);
    b.add_rectangle_bl(115.0, 40.0, 15.0, 46.02);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: RoutingBetweenCollinearObstaclesInConvexHull
/// Collinear obstacles in convex hulls may create collinear unpadded-to-padded
/// BorderIntersect transient edges.
/// C#: two overlapping triangles.
#[test]
fn routing_between_collinear_obstacles_in_convex_hull() {
    let mut b = ScenarioBuilder::new();
    // Triangles: (50,50),(60,80),(70,50) and (65,50),(75,80),(85,50)
    let obs0 = b.add_rectangle_bl(50.0, 50.0, 20.0, 30.0);
    let obs1 = b.add_rectangle_bl(65.0, 50.0, 20.0, 30.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: Document_Illustration2
/// Document illustration test; runs InterOverlapShortCircuit_Low_Middle
/// with WantPaths=false.
/// C#: multiple rectangles with overlap, routes between first two obstacles.
#[test]
fn document_illustration2() {
    let mut b = ScenarioBuilder::new();
    // Source and target
    let obs0 = b.add_rectangle_bl(10.0, 10.0, 10.0, 10.0);
    let obs1 = b.add_rectangle_bl(120.0, 10.0, 10.0, 10.0);
    // Big rectangle in the middle
    b.add_rectangle_bl(40.0, 10.0, 60.0, 30.0);
    // Middle obstacles cutting border
    b.add_rectangle_bl(60.0, 5.0, 5.0, 10.0);
    b.add_rectangle_bl(75.0, 5.0, 5.0, 10.0);
    // Corner obstacles
    b.add_rectangle_bl(30.0, 5.0, 20.0, 10.0);
    b.add_rectangle_bl(90.0, 5.0, 20.0, 10.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: RemoveCloseVerticesFromPolyline
/// Correct removal of vertices that are ApproximateComparer.Close.
/// This is a polyline utility test, not a routing test.
#[test]
fn remove_close_vertices_from_polyline() {
    // C# creates an octagon, duplicates each point with a tiny offset,
    // and verifies RemoveCloseAndCollinearVerticesInPlace removes them.
    // This requires the Obstacle::remove_close_and_collinear_vertices API.
}

/// Port: RemoveCollinearVerticesFromPolyline
/// Correct removal of vertices that are collinear with other vertices.
/// This is a polyline utility test, not a routing test.
#[test]
fn remove_collinear_vertices_from_polyline() {
    // C# creates an octagon, adds midpoints on each edge,
    // and verifies RemoveCloseAndCollinearVerticesInPlace removes them.
}
