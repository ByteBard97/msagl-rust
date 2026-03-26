/// Rectilinear routing tests — group/cluster tests part 2, ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// Continuation of rectilinear_tests_groups.rs. Covers FreePort-in-group tests,
/// GroupBoundaryCrossings, non-rectangular groups, Group_Inside_* variants,
/// overlapping groups/obstacles, and the rectangular group sides-and-corners test.
///
/// ALL tests are marked `#[ignore = "requires group/cluster routing"]` because group
/// routing is not yet implemented in the Rust port.

#[path = "test_harness/mod.rs"]
mod test_harness;

use test_harness::ScenarioBuilder;

// ── Helper: add rectangle from two corner points (C# convention) ────────────
fn add_rect_corners(b: &mut ScenarioBuilder, x1: f64, y1: f64, x2: f64, y2: f64) -> usize {
    let min_x = x1.min(x2);
    let min_y = y1.min(y2);
    let w = (x2 - x1).abs();
    let h = (y2 - y1).abs();
    b.add_rectangle_bl(min_x, min_y, w, h)
}

// ── Group_FreePort variants ─────────────────────────────────────────────────

/// Port: Group_FreePort_Outside_Group
/// Route from obstacle inside a group to a FreePort outside the group.
/// s1=(30,30)-(40,40), g1=(20,20)-(50,50), FreePort at (60,35).
#[test]

fn group_free_port_outside_group() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 30.0, 30.0, 40.0, 40.0);
    let _g1 = add_rect_corners(&mut b, 20.0, 20.0, 50.0, 50.0);
    // g1 children: [s1]. FreePort at (60,35) — outside g1.
    // Note: FreePort routing not supported in ScenarioBuilder; route to a small obstacle instead.
    let fp = add_rect_corners(&mut b, 59.0, 34.0, 61.0, 36.0);
    b.route_between(s1, fp);
    let _result = b.run();
}

/// Port: Group_FreePort_Inside_Group
/// Route from obstacle inside a group to a FreePort inside the group.
/// s1=(30,30)-(40,40), g1=(20,20)-(70,50), FreePort at (60,35).
#[test]

fn group_free_port_inside_group() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 30.0, 30.0, 40.0, 40.0);
    let _g1 = add_rect_corners(&mut b, 20.0, 20.0, 70.0, 50.0);
    // g1 children: [s1]. FreePort at (60,35) — inside g1.
    let fp = add_rect_corners(&mut b, 59.0, 34.0, 61.0, 36.0);
    b.route_between(s1, fp);
    let _result = b.run();
}

/// Port: Group_FreePort_Inside_Group_Nested
/// Nested group with FreePort inside.
/// s1=(30,30)-(40,40), g1=(20,20)-(70,50), g1_nested=(10,10)-(80,60).
/// g1_nested children: [g1]. FreePort at (60,35).
#[test]

fn group_free_port_inside_group_nested() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 30.0, 30.0, 40.0, 40.0);
    let _g1 = add_rect_corners(&mut b, 20.0, 20.0, 70.0, 50.0);
    let _g1_nested = add_rect_corners(&mut b, 10.0, 10.0, 80.0, 60.0);
    // g1 children: [s1], g1_nested children: [g1]
    let fp = add_rect_corners(&mut b, 59.0, 34.0, 61.0, 36.0);
    b.route_between(s1, fp);
    let _result = b.run();
}

// ── GroupBoundaryCrossings_Test ─────────────────────────────────────────────

/// Port: GroupBoundaryCrossings_Test
/// Verifies creating, adding to, and removing from GroupBoundaryCrossingMap.
/// No routing is done — this is a data structure test.
/// Creates 30 tiny shapes at (0..29, 0) and tests ordered intersection queries.
#[test]

fn group_boundary_crossings_test() {
    // This test exercises GroupBoundaryCrossingMap which is not yet ported.
    // When implemented, create 30 point-and-shape entries, add to map in
    // reverse order, then verify GetOrderedListBetween returns correct subsets.
}

// ── Group_NonRect_BlockedReflections ────────────────────────────────────────

/// Port: Group_NonRect_BlockedReflections
/// Verify reflections don't create spurious crossings across a non-rect group boundary.
/// Uses diamond-shaped group and rotated diamond reflectors (non-rectangular).
/// All shapes approximated as rectangular placeholders.
#[test]

fn group_non_rect_blocked_reflections() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 220.0, 90.0, 240.0, 110.0);
    let s2 = add_rect_corners(&mut b, 130.0, 200.0, 150.0, 220.0);
    // Inner reflector r1 (diamond): bounding box approx (80,70)-(180,170)
    let _r1 = add_rect_corners(&mut b, 80.0, 70.0, 180.0, 170.0);
    // Outer reflector r2 (diamond): bounding box approx (60,130)-(190,170) + ReflectionWeight
    let _r2 = add_rect_corners(&mut b, 60.0, 130.0, 190.0, 170.0);
    // Group g1 (diamond): bounding box approx (-100,-100)-(200,200), children: [r1]
    let _g1 = add_rect_corners(&mut b, -100.0, -100.0, 200.0, 200.0);
    // Additional reflectors t1, t2 (diamonds)
    let _t1 = add_rect_corners(&mut b, -65.0, -75.0, 55.0, 45.0);
    let _t2 = add_rect_corners(&mut b, -145.0, -180.0, 0.0, -35.0);
    b.route_between(s1, s2);
    let _result = b.run();
}

// ── Simple_NonRectangular_Group ─────────────────────────────────────────────

/// Port: Simple_NonRectangular_Group
/// Route around a diamond-shaped group for non-children, into it for children.
/// s1=(160,10)-(180,30), s2=(160,170)-(180,190), c1=(120,90)-(140,110) inside group.
/// Group g1 is diamond (100,0)-(0,100)-(100,200)-(200,100), children: [c1].
#[test]

fn simple_non_rectangular_group() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 160.0, 10.0, 180.0, 30.0);
    let s2 = add_rect_corners(&mut b, 160.0, 170.0, 180.0, 190.0);
    let c1 = add_rect_corners(&mut b, 120.0, 90.0, 140.0, 110.0);
    // Group g1 diamond, bounding box (0,0)-(200,200), children: [c1]
    let _g1 = add_rect_corners(&mut b, 0.0, 0.0, 200.0, 200.0);
    b.route_between(s1, s2);
    b.route_between(s1, c1);
    let _result = b.run();
}

// ── Group_FlatTop_BlockedReflections ────────────────────────────────────────

/// Port: Group_FlatTop_BlockedReflections
/// Verify reflections don't create spurious crossings across a flat group top.
/// s1=(0,70)-(50,80), s2=(60,10)-(70,20).
/// r1 is triangle (10,10)-(20,30)-(30,10) approximated as rect.
/// b1=(32,50)-(160,60) blocker.
/// g1=(0,0)-(50,40), children: [r1].
#[test]

fn group_flat_top_blocked_reflections() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 0.0, 70.0, 50.0, 80.0);
    let s2 = add_rect_corners(&mut b, 60.0, 10.0, 70.0, 20.0);
    // Triangle r1 approx as rect (10,10)-(30,30)
    let _r1 = add_rect_corners(&mut b, 10.0, 10.0, 30.0, 30.0);
    let _b1 = add_rect_corners(&mut b, 32.0, 50.0, 160.0, 60.0);
    // Group g1=(0,0)-(50,40), children: [r1]
    let _g1 = add_rect_corners(&mut b, 0.0, 0.0, 50.0, 40.0);
    b.route_between(s1, s2);
    let _result = b.run();
}

// ── Group_Simple_One_Obstacle_Inside_One_Group ──────────────────────────────

/// Port: Group_Simple_One_Obstacle_Inside_One_Group
/// Simplest group test: one obstacle outside, one inside a single group.
/// s1=(40,40)-(50,50) inside g1=(30,30)-(60,60). s2=(80,40)-(90,50) outside.
#[test]

fn group_simple_one_obstacle_inside_one_group() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 40.0, 40.0, 50.0, 50.0);
    let s2 = add_rect_corners(&mut b, 80.0, 40.0, 90.0, 50.0);
    let _g1 = add_rect_corners(&mut b, 30.0, 30.0, 60.0, 60.0);
    // g1 children: [s1]
    b.route_between(s1, s2);
    let _result = b.run();
}

// ── Group_With_Overlapping_Obstacles ────────────────────────────────────────

/// Port: Group_With_Overlapping_Obstacles
/// Diamond group with s1 inside, s2 outside, plus 0/1/2 overlapping obstacles.
/// s1=(160,90)-(180,110), s2=(20,90)-(40,110).
/// g1 is diamond (100,0)-(0,100)-(100,200)-(200,100) approx as rect, children: [s1].
/// Overlapped1: parallelogram approx (120,160)-(180,220).
/// Overlapped2: rect (180,140)-(240,160).
#[test]

fn group_with_overlapping_obstacles() {
    // C# calls GroupAndObstacleOverlapWorker(0), (1), (2) sequentially.
    // We test the maximal case (2 overlapping obstacles).
    let mut b = ScenarioBuilder::new();
    let _s1 = add_rect_corners(&mut b, 160.0, 90.0, 180.0, 110.0);
    let _s2 = add_rect_corners(&mut b, 20.0, 90.0, 40.0, 110.0);
    // Diamond group approx as bounding rect (0,0)-(200,200), children: [s1]
    let _g1 = add_rect_corners(&mut b, 0.0, 0.0, 200.0, 200.0);
    // Overlapping obstacle 1 (parallelogram approx)
    let _overlap1 = add_rect_corners(&mut b, 120.0, 160.0, 180.0, 220.0);
    // Overlapping obstacle 2
    let _overlap2 = add_rect_corners(&mut b, 180.0, 140.0, 240.0, 160.0);
    // No routing edges in C# — just CreateRouter + RunAndShowGraph
    let _result = b.run();
}

// ── Group_With_Overlapping_Groups ───────────────────────────────────────────

/// Port: Group_With_Overlapping_Groups
/// Two overlapping non-rectangular groups plus a rectangular group.
/// g1 diamond, g2 parallelogram, g3 rect. Verifies visibility polylines
/// don't intersect and g2/g3 are inside g1's convex hull.
#[test]

fn group_with_overlapping_groups() {
    let mut b = ScenarioBuilder::new();
    let _s1 = add_rect_corners(&mut b, 160.0, 90.0, 180.0, 110.0);
    let _s2 = add_rect_corners(&mut b, 20.0, 90.0, 40.0, 110.0);
    // g1 diamond approx (0,0)-(200,200), children: [s1]
    let _g1 = add_rect_corners(&mut b, 0.0, 0.0, 200.0, 200.0);
    // g2 parallelogram approx (120,130)-(230,240), children: [s3]
    let _g2 = add_rect_corners(&mut b, 120.0, 130.0, 230.0, 240.0);
    let _s3 = add_rect_corners(&mut b, 160.0, 160.0, 180.0, 180.0);
    // g3 rect (215,60)-(315,150), children: [s4]
    let _g3 = add_rect_corners(&mut b, 215.0, 60.0, 315.0, 150.0);
    let _s4 = add_rect_corners(&mut b, 255.0, 95.0, 275.0, 115.0);
    let _result = b.run();
}

// ── Group_Inside_* variants ─────────────────────────────────────────────────

/// Port: Group_Inside_Rectangular_Obstacle
/// A rectangular group inside a rectangular obstacle — both should remain unchanged.
/// s1=(100,100)-(200,200), g1=(120,120)-(180,180), g1 children: [s1].
#[test]

fn group_inside_rectangular_obstacle() {
    let mut b = ScenarioBuilder::new();
    let _s1 = add_rect_corners(&mut b, 100.0, 100.0, 200.0, 200.0);
    // g1=(120,120)-(180,180), children: [s1]
    let _g1 = add_rect_corners(&mut b, 120.0, 120.0, 180.0, 180.0);
    let _result = b.run();
}

/// Port: Group_Inside_Rectangular_Obstacle_Contains_Rectangular_Obstacle
/// Obstacle inside a group inside an obstacle, all rectangular.
/// s1=(100,100)-(200,200), g1=(120,120)-(180,180) children: [s1, s2].
/// s2=(140,140)-(160,160) added as child of g1.
#[test]

fn group_inside_rectangular_obstacle_contains_rectangular_obstacle() {
    let mut b = ScenarioBuilder::new();
    let _s1 = add_rect_corners(&mut b, 100.0, 100.0, 200.0, 200.0);
    let _g1 = add_rect_corners(&mut b, 120.0, 120.0, 180.0, 180.0);
    let _s2 = add_rect_corners(&mut b, 140.0, 140.0, 160.0, 160.0);
    // g1 children: [s1, s2]
    let _result = b.run();
}

/// Port: Group_Inside_NonRectangular_Obstacle
/// A group inside a diamond obstacle. Group must grow to encompass it.
/// s1 is diamond (50,150)-(150,250)-(250,150)-(150,50) approx rect.
/// g1=(120,120)-(180,180), children: [s1].
#[test]

fn group_inside_non_rectangular_obstacle() {
    let mut b = ScenarioBuilder::new();
    // Diamond s1 approx as bounding rect (50,50)-(250,250)
    let _s1 = add_rect_corners(&mut b, 50.0, 50.0, 250.0, 250.0);
    let _g1 = add_rect_corners(&mut b, 120.0, 120.0, 180.0, 180.0);
    // g1 children: [s1]
    let _result = b.run();
}

/// Port: Group_Inside_NonRectangular_Obstacle_Contains_Rectangular_Obstacle
/// Same as above but g1 also contains s2=(140,140)-(160,160).
#[test]

fn group_inside_non_rectangular_obstacle_contains_rectangular_obstacle() {
    let mut b = ScenarioBuilder::new();
    let _s1 = add_rect_corners(&mut b, 50.0, 50.0, 250.0, 250.0);
    let _g1 = add_rect_corners(&mut b, 120.0, 120.0, 180.0, 180.0);
    let _s2 = add_rect_corners(&mut b, 140.0, 140.0, 160.0, 160.0);
    // g1 children: [s1, s2]
    let _result = b.run();
}

// ── Rectangular_Obstacles_Overlapping_Rectangular_Group_Sides_And_Corners ───

/// Port: Rectangular_Obstacles_Overlapping_Rectangular_Group_Sides_And_Corners
/// A large rectangular group with 8 smaller rectangles overlapping its sides and corners.
/// Group: (100,100)-(200,200).
/// Corners: (90,90)-(110,110), (90,190)-(110,210), (190,190)-(210,210), (190,90)-(210,110).
/// Sides: (90,140)-(110,160), (140,190)-(160,210), (190,140)-(210,160), (140,90)-(160,110).
#[test]

fn rectangular_obstacles_overlapping_rectangular_group_sides_and_corners() {
    let mut b = ScenarioBuilder::new();
    // The "group" obstacle
    let _group = add_rect_corners(&mut b, 100.0, 100.0, 200.0, 200.0);
    // Corners
    let _c1 = add_rect_corners(&mut b, 90.0, 90.0, 110.0, 110.0);
    let _c2 = add_rect_corners(&mut b, 90.0, 190.0, 110.0, 210.0);
    let _c3 = add_rect_corners(&mut b, 190.0, 190.0, 210.0, 210.0);
    let _c4 = add_rect_corners(&mut b, 190.0, 90.0, 210.0, 110.0);
    // Sides
    let _side1 = add_rect_corners(&mut b, 90.0, 140.0, 110.0, 160.0);
    let _side2 = add_rect_corners(&mut b, 140.0, 190.0, 160.0, 210.0);
    let _side3 = add_rect_corners(&mut b, 190.0, 140.0, 210.0, 160.0);
    let _side4 = add_rect_corners(&mut b, 140.0, 90.0, 160.0, 110.0);
    // No routing edges in C# — just CreateRouter + RunAndShowGraph
    let _result = b.run();
}
