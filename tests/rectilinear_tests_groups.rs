/// Rectilinear routing tests — group/cluster tests, ported from the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// These tests exercise group (cluster) routing features: nested groups, group boundary
/// crossings, spatial parents, landlocking, overlapping groups, free ports in groups,
/// non-rectangular groups, and obstacle-group overlap scenarios.
///
/// ALL tests are marked `#[ignore = "requires group/cluster routing"]` because group
/// routing is not yet implemented in the Rust port. They serve as acceptance criteria
/// for future group routing work.
///
/// Naming convention: C# test name → snake_case Rust test name.
/// Geometry is translated as faithfully as possible from the C# PolylineFromRectanglePoints
/// calls. Group hierarchy is documented in comments since ScenarioBuilder does not yet
/// support groups.

#[path = "test_harness/mod.rs"]
mod test_harness;

use test_harness::ScenarioBuilder;

// ── Helper: add rectangle from two corner points (C# convention) ────────────
// C# PolylineFromRectanglePoints(new Point(x1,y1), new Point(x2,y2))
// translates to add_rectangle_bl(min_x, min_y, width, height).
fn add_rect_corners(b: &mut ScenarioBuilder, x1: f64, y1: f64, x2: f64, y2: f64) -> usize {
    let min_x = x1.min(x2);
    let min_y = y1.min(y2);
    let w = (x2 - x1).abs();
    let h = (y2 - y1).abs();
    b.add_rectangle_bl(min_x, min_y, w, h)
}

// ── GroupTest_Simple variants ───────────────────────────────────────────────

/// Port: GroupTest_Simple (wantGroup=true)
/// Route between two obstacles blocked by a group, and between those obstacles
/// and an obstacle inside the group.
/// Group hierarchy: g1 contains s2. g1 rect=(30,-15)-(60,45).
/// Obstacles: s1=(10,12)-(20,22), s2=(40,5)-(50,25), s3=(70,8)-(80,18).
/// Edges: s1→s3, s1→s2, s2→s3.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test_simple() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 10.0, 12.0, 20.0, 22.0);
    let s2 = add_rect_corners(&mut b, 40.0, 5.0, 50.0, 25.0);
    let s3 = add_rect_corners(&mut b, 70.0, 8.0, 80.0, 18.0);
    // Group g1: rect (30,-15)-(60,45), children: [s2]
    let _g1 = add_rect_corners(&mut b, 30.0, -15.0, 60.0, 45.0);
    b.route_between(s1, s3);
    b.route_between(s1, s2);
    b.route_between(s2, s3);
    let _result = b.run();
}

/// Port: GroupTest_Simple_NoGroup (wantGroup=false)
/// Same layout but the group rectangle is just an obstacle (no children).
/// Obstacles: s1=(10,12)-(20,22), s3=(70,8)-(80,18).
/// Group g1 is present as obstacle but has no children — not a real group.
/// Edge: s1→s3.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test_simple_no_group() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 10.0, 12.0, 20.0, 22.0);
    let s3 = add_rect_corners(&mut b, 70.0, 8.0, 80.0, 18.0);
    // Group g1 as plain obstacle (no children): (30,-15)-(60,45)
    let _g1 = add_rect_corners(&mut b, 30.0, -15.0, 60.0, 45.0);
    b.route_between(s1, s3);
    let _result = b.run();
}

/// Port: GroupTest_Simple_NoGroup_PortSplice_LimitRect
/// Same as NoGroup but with LimitPortVisibilitySpliceToEndpointBoundingBox=true.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test_simple_no_group_port_splice_limit_rect() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 10.0, 12.0, 20.0, 22.0);
    let s3 = add_rect_corners(&mut b, 70.0, 8.0, 80.0, 18.0);
    let _g1 = add_rect_corners(&mut b, 30.0, -15.0, 60.0, 45.0);
    b.route_between(s1, s3);
    // Note: LimitPortVisibilitySpliceToEndpointBoundingBox not yet supported
    let _result = b.run();
}

// ── GroupTest_Worker variants (nested groups with 3 or 4 groups) ────────────
// Common layout:
//   s1=(-45,-5)-(-35,5), s2=(-5,off?-2:-5)-(5,5), s3=(85,-5)-(95,off?2:5),
//   s4=(145,off?-2:-5)-(155,5), s5=(245,-5)-(255,5)
//   g1=(-25,-15)-(outsideGroup1?80:105,15), g2=(-50,-30)-(120,30), g3=(145,-15)-(295,15)
//   Hierarchy: g1 children=[s2,s3], g2 children=[s1,g1], g3 children=[s4,s5]
//   If wantFourthGroup: s6=(-3,19)-(4,25), g4=(-12,-40)-(12,40), g4 children=[s6,s2]

fn group_test_worker(b: &mut ScenarioBuilder, want_fourth_group: bool, outside_group1: bool, off_center: bool) {
    let s1 = add_rect_corners(b, -45.0, -5.0, -35.0, 5.0);
    let s2_y_lo = if off_center { -2.0 } else { -5.0 };
    let s2 = add_rect_corners(b, -5.0, s2_y_lo, 5.0, 5.0);
    let s3_y_hi = if off_center { 2.0 } else { 5.0 };
    let s3 = add_rect_corners(b, 85.0, -5.0, 95.0, s3_y_hi);
    let s4_y_lo = if off_center { -2.0 } else { -5.0 };
    let s4 = add_rect_corners(b, 145.0, s4_y_lo, 155.0, 5.0);
    let s5 = add_rect_corners(b, 245.0, -5.0, 255.0, 5.0);

    // Groups (as placeholder obstacles — hierarchy documented in comments)
    let g1_x_hi = if outside_group1 { 80.0 } else { 105.0 };
    let _g1 = add_rect_corners(b, -25.0, -15.0, g1_x_hi, 15.0);
    // g1 children: [s2, s3]
    let _g2 = add_rect_corners(b, -50.0, -30.0, 120.0, 30.0);
    // g2 children: [s1, g1]
    let _g3 = add_rect_corners(b, 145.0, -15.0, 295.0, 15.0);
    // g3 children: [s4, s5]

    b.route_between(s1, s5);
    b.route_between(s2, s3);
    b.route_between(s2, s4);
    b.route_between(s2, s5);
    b.route_between(s4, s5);

    if want_fourth_group {
        let s6 = add_rect_corners(b, -3.0, 19.0, 4.0, 25.0);
        let _g4 = add_rect_corners(b, -12.0, -40.0, 12.0, 40.0);
        // g4 children: [s6, s2], g2 also gets s6
        b.route_between(s1, s4);
        b.route_between(s1, s6);
        b.route_between(s6, s5);
    }
}

/// Port: GroupTest0 — no fourth group, not outside g1, not off-center
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test0() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, false, false, false);
    let _result = b.run();
}

/// Port: GroupTest0_OutsideGroup1
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test0_outside_group1() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, false, true, false);
    let _result = b.run();
}

/// Port: GroupTest0_OffCenter
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test0_off_center() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, false, false, true);
    let _result = b.run();
}

/// Port: GroupTest0_OutsideGroup1_OffCenter
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test0_outside_group1_off_center() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, false, true, true);
    let _result = b.run();
}

/// Port: GroupTest — with fourth group
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, true, false, false);
    let _result = b.run();
}

/// Port: GroupTest_OutsideGroup1
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test_outside_group1() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, true, true, false);
    let _result = b.run();
}

/// Port: GroupTest_OffCenter
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test_off_center() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, true, false, true);
    let _result = b.run();
}

/// Port: GroupTest_OutsideGroup1_OffCenter
#[test]
#[ignore = "requires group/cluster routing"]
fn group_test_outside_group1_off_center() {
    let mut b = ScenarioBuilder::new();
    group_test_worker(&mut b, true, true, true);
    let _result = b.run();
}

// ── Group_Obstacle_Crossing_Boundary variants ───────────────────────────────
// Common layout:
//   s1=(45,20)-(55,30), s2=(45,60)-(55,70)
//   s3=(blocking?-40:-20, 40)-(70+gap, 50)
//   g1=(-30,10)-(70,80), children: [s1, s2, s3]
//   Edge: s1→s2

fn group_obstacle_crossing_boundary_worker(b: &mut ScenarioBuilder, gap: f64, blocking: bool) {
    let s1 = add_rect_corners(b, 45.0, 20.0, 55.0, 30.0);
    let s2 = add_rect_corners(b, 45.0, 60.0, 55.0, 70.0);
    let s3_x_lo = if blocking { -40.0 } else { -20.0 };
    let _s3 = add_rect_corners(b, s3_x_lo, 40.0, 70.0 + gap, 50.0);
    // g1=(-30,10)-(70,80), children: [s1, s2, s3]
    let _g1 = add_rect_corners(b, -30.0, 10.0, 70.0, 80.0);
    b.route_between(s1, s2);
}

/// Port: Group_Obstacle_Crossing_Boundary_Between_Routed_Obstacles_Gap2
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_crossing_boundary_between_routed_obstacles_gap2() {
    let mut b = ScenarioBuilder::new();
    group_obstacle_crossing_boundary_worker(&mut b, 2.0, false);
    let _result = b.run();
}

/// Port: Group_Obstacle_Crossing_Boundary_Between_Routed_Obstacles_Gap4
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_crossing_boundary_between_routed_obstacles_gap4() {
    let mut b = ScenarioBuilder::new();
    group_obstacle_crossing_boundary_worker(&mut b, 4.0, false);
    let _result = b.run();
}

/// Port: Group_Obstacle_Crossing_Boundary_On_Routed_Obstacles
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_crossing_boundary_on_routed_obstacles() {
    let mut b = ScenarioBuilder::new();
    group_obstacle_crossing_boundary_worker(&mut b, 0.0, false);
    let _result = b.run();
}

/// Port: Group_Obstacle_Crossing_Boundary_Inside_Routed_Obstacles_Gap2
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_crossing_boundary_inside_routed_obstacles_gap2() {
    let mut b = ScenarioBuilder::new();
    group_obstacle_crossing_boundary_worker(&mut b, -2.0, false);
    let _result = b.run();
}

/// Port: Group_Obstacle_Crossing_Boundary_Inside_Routed_Obstacles_Gap4
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_crossing_boundary_inside_routed_obstacles_gap4() {
    let mut b = ScenarioBuilder::new();
    group_obstacle_crossing_boundary_worker(&mut b, -4.0, false);
    let _result = b.run();
}

/// Port: Group_Obstacle_Crossing_Boundary_Fully_Blocking_Routed_Obstacles
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_crossing_boundary_fully_blocking_routed_obstacles() {
    let mut b = ScenarioBuilder::new();
    group_obstacle_crossing_boundary_worker(&mut b, 10.0, true);
    let _result = b.run();
}

// ── Group_AdjacentOuterEdge variants ────────────────────────────────────────
// Common layout:
//   s1=(20,40)-(30,50), s2=(70,30)-(80,50), s3=(120,40)-(130,50), s4=(120,60)-(130,70)
//   g1=(10,10)-(nested?100:48-gap, 80), g2=(50,20)-(100,70)
//   g1 children: [s1], g2 children: [s2]. If nested: g1 also contains g2.

fn group_adjacent_outer_edge_worker(b: &mut ScenarioBuilder, nested: bool, gap: f64) {
    let s1 = add_rect_corners(b, 20.0, 40.0, 30.0, 50.0);
    let s2 = add_rect_corners(b, 70.0, 30.0, 80.0, 50.0);
    let s3 = add_rect_corners(b, 120.0, 40.0, 130.0, 50.0);
    let s4 = add_rect_corners(b, 120.0, 60.0, 130.0, 70.0);
    let g1_x_hi = if nested { 100.0 } else { 48.0 - gap };
    let _g1 = add_rect_corners(b, 10.0, 10.0, g1_x_hi, 80.0);
    // g1 children: [s1]. If nested, g1 also contains g2.
    let _g2 = add_rect_corners(b, 50.0, 20.0, 100.0, 70.0);
    // g2 children: [s2]
    b.route_between(s1, s2);
    b.route_between(s1, s4);
    b.route_between(s2, s3);
}

/// Port: Group_AdjacentOuterEdge_Outside
#[test]
#[ignore = "requires group/cluster routing"]
fn group_adjacent_outer_edge_outside() {
    let mut b = ScenarioBuilder::new();
    group_adjacent_outer_edge_worker(&mut b, false, 0.0);
    let _result = b.run();
}

/// Port: Group_AdjacentOuterEdge_Nested
#[test]
#[ignore = "requires group/cluster routing"]
fn group_adjacent_outer_edge_nested() {
    let mut b = ScenarioBuilder::new();
    group_adjacent_outer_edge_worker(&mut b, true, 0.0);
    let _result = b.run();
}

/// Port: Group_AdjacentOuterEdge_Gap
#[test]
#[ignore = "requires group/cluster routing"]
fn group_adjacent_outer_edge_gap() {
    let mut b = ScenarioBuilder::new();
    group_adjacent_outer_edge_worker(&mut b, false, 5.0);
    let _result = b.run();
}

/// Port: Group_AdjacentOuterEdge_Straddle
#[test]
#[ignore = "requires group/cluster routing"]
fn group_adjacent_outer_edge_straddle() {
    let mut b = ScenarioBuilder::new();
    group_adjacent_outer_edge_worker(&mut b, false, -5.0);
    let _result = b.run();
}

// ── Group_Spatial_Parent variants ───────────────────────────────────────────

/// Port: Group_Spatial_Parent (non-overlapping)
/// Complex hierarchy with 8 groups and 5 obstacles.
/// See C# Group_Spatial_Parent_Worker for full hierarchy.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_spatial_parent() {
    let mut b = ScenarioBuilder::new();
    // Obstacles
    let s1 = add_rect_corners(&mut b, 40.0, 40.0, 50.0, 50.0);
    let s2 = add_rect_corners(&mut b, 100.0, 50.0, 110.0, 60.0);
    let _s1_dummy = add_rect_corners(&mut b, 34.0, 34.0, 36.0, 36.0);
    let _s2_dummy = add_rect_corners(&mut b, 94.0, 44.0, 96.0, 46.0);
    let _s3_dummy = add_rect_corners(&mut b, 100.0, 20.0, 105.0, 25.0);
    // Groups (non-overlapping variant: g1b starts at x=25)
    let _g0 = add_rect_corners(&mut b, 10.0, 10.0, 140.0, 80.0); // g0: children [s1_dummy, s2_dummy]
    let _g1a = add_rect_corners(&mut b, 15.0, 15.0, 75.0, 75.0); // g1a: children [s1_dummy]
    let _g1p = add_rect_corners(&mut b, 20.0, 20.0, 70.0, 70.0); // g1p: children [s1]
    let _g1b = add_rect_corners(&mut b, 25.0, 25.0, 65.0, 65.0); // g1b: children [s1_dummy]
    let _g1c = add_rect_corners(&mut b, 30.0, 30.0, 60.0, 60.0); // g1c: children [s1_dummy]
    let _g2a = add_rect_corners(&mut b, 85.0, 35.0, 125.0, 75.0); // g2a: children [s2_dummy]
    let _g2p = add_rect_corners(&mut b, 90.0, 40.0, 120.0, 70.0); // g2p: children [s2]
    let _g3_dummy = add_rect_corners(&mut b, 85.0, 15.0, 125.0, 30.0); // g3dummy: children [s3_dummy]
    b.route_between(s1, s2);
    let _result = b.run();
}

/// Port: Group_Spatial_Parent_GroupOverlap (overlapping variant)
/// Same as Group_Spatial_Parent but g1b starts at x=-25 and g2p extends to x=170.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_spatial_parent_group_overlap() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 40.0, 40.0, 50.0, 50.0);
    let s2 = add_rect_corners(&mut b, 100.0, 50.0, 110.0, 60.0);
    let _s1_dummy = add_rect_corners(&mut b, 34.0, 34.0, 36.0, 36.0);
    let _s2_dummy = add_rect_corners(&mut b, 94.0, 44.0, 96.0, 46.0);
    let _s3_dummy = add_rect_corners(&mut b, 100.0, 20.0, 105.0, 25.0);
    let _g0 = add_rect_corners(&mut b, 10.0, 10.0, 140.0, 80.0);
    let _g1a = add_rect_corners(&mut b, 15.0, 15.0, 75.0, 75.0);
    let _g1p = add_rect_corners(&mut b, 20.0, 20.0, 70.0, 70.0);
    let _g1b = add_rect_corners(&mut b, -25.0, 25.0, 65.0, 65.0); // overlapping: x starts at -25
    let _g1c = add_rect_corners(&mut b, 30.0, 30.0, 60.0, 60.0);
    let _g2a = add_rect_corners(&mut b, 85.0, 35.0, 125.0, 75.0);
    let _g2p = add_rect_corners(&mut b, 90.0, 40.0, 170.0, 70.0); // overlapping: extends to 170
    let _g3_dummy = add_rect_corners(&mut b, 85.0, 15.0, 125.0, 30.0);
    b.route_between(s1, s2);
    let _result = b.run();
}

/// Port: Group_Spatial_Parent_GroupOverlap_Collinear
/// Collinear ports through overlapping spatial parent groups.
/// All groups share a single dummy child (s3_dummy) to be recognized as groups.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_spatial_parent_group_overlap_collinear() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 40.0, 40.0, 50.0, 50.0);
    let s2 = add_rect_corners(&mut b, 100.0, 40.0, 110.0, 50.0);
    let _s3_dummy = add_rect_corners(&mut b, 100.0, 72.0, 110.0, 75.0);
    // All groups below have s3_dummy as child to be recognized as groups
    let _g0 = add_rect_corners(&mut b, 10.0, 10.0, 140.0, 80.0);
    let _g1a = add_rect_corners(&mut b, 15.0, 15.0, 75.0, 75.0);
    let _g1p = add_rect_corners(&mut b, 20.0, 20.0, 70.0, 70.0); // also has s1
    let _g1b = add_rect_corners(&mut b, 25.0, 25.0, 65.0, 65.0);
    let _g1c = add_rect_corners(&mut b, 30.0, 30.0, 60.0, 60.0);
    let _g2a = add_rect_corners(&mut b, 85.0, 25.0, 125.0, 65.0);
    let _g2p = add_rect_corners(&mut b, 90.0, 30.0, 170.0, 60.0); // also has s2
    b.route_between(s1, s2);
    let _result = b.run();
}

// ── Group_Landlock ──────────────────────────────────────────────────────────

/// Port: Group_Landlock
/// Groups that are not in an obstacle hierarchy may landlock an obstacle.
/// g1p=(40,30)-(70,60) children=[s1], g2p=(130,30)-(160,60) children=[s2]
/// ga=(10,10)-(20,90), gb=(10,80)-(100,90), gc=(90,10)-(100,90), gd=(10,10)-(100,20)
/// Each landlocking group has a dummy child.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_landlock() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 50.0, 40.0, 60.0, 50.0);
    let s2 = add_rect_corners(&mut b, 140.0, 40.0, 150.0, 50.0);
    let _sa_dummy = add_rect_corners(&mut b, 13.0, 50.0, 17.0, 54.0);
    let _sb_dummy = add_rect_corners(&mut b, 53.0, 83.0, 57.0, 87.0);
    let _sc_dummy = add_rect_corners(&mut b, 93.0, 50.0, 97.0, 54.0);
    let _sd_dummy = add_rect_corners(&mut b, 53.0, 13.0, 57.0, 17.0);
    // Parent groups
    let _g1p = add_rect_corners(&mut b, 40.0, 30.0, 70.0, 60.0); // children: [s1]
    let _g2p = add_rect_corners(&mut b, 130.0, 30.0, 160.0, 60.0); // children: [s2]
    // Landlocking groups
    let _ga = add_rect_corners(&mut b, 10.0, 10.0, 20.0, 90.0); // children: [sa_dummy]
    let _gb = add_rect_corners(&mut b, 10.0, 80.0, 100.0, 90.0); // children: [sb_dummy]
    let _gc = add_rect_corners(&mut b, 90.0, 10.0, 100.0, 90.0); // children: [sc_dummy]
    let _gd = add_rect_corners(&mut b, 10.0, 10.0, 100.0, 20.0); // children: [sd_dummy]
    b.route_between(s1, s2);
    let _result = b.run();
}

// ── Group_Obstacle_Overlap variants ─────────────────────────────────────────
// Common layout:
//   s1=(50,40)-(60,60), s2=(140,40)-(150,60)
//   Nested parent groups for s1:
//     g1p1=(40,30)-(72,70), g1p2=(40,30)-(75,70), g1p3=(40,30)-(78,70), g1p4=(40,30)-(81,70)
//   Triangle/rect obstacles overlap group boundaries.

/// Port: Group_Obstacle_Overlap_Triangle
/// Two triangles cross the group boundaries. Triangles are non-rect so we
/// approximate with rectangles as placeholders.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_overlap_triangle() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 50.0, 40.0, 60.0, 60.0);
    let s2 = add_rect_corners(&mut b, 140.0, 40.0, 150.0, 60.0);
    // Triangle1 approx bounding box: (70,40)-(90,60), Triangle2: (70,44)-(76.5,50.5)
    let _tri1 = add_rect_corners(&mut b, 70.0, 40.0, 90.0, 60.0);
    let _tri2 = add_rect_corners(&mut b, 70.0, 44.0, 76.5, 50.5);
    // Nested groups (g1p4 > g1p3 > g1p2 > g1p1 > s1)
    let _g1p1 = add_rect_corners(&mut b, 40.0, 30.0, 72.0, 70.0);
    let _g1p2 = add_rect_corners(&mut b, 40.0, 30.0, 75.0, 70.0);
    let _g1p3 = add_rect_corners(&mut b, 40.0, 30.0, 78.0, 70.0);
    let _g1p4 = add_rect_corners(&mut b, 40.0, 30.0, 81.0, 70.0);
    b.route_between(s1, s2);
    let _result = b.run();
}

/// Port: Group_Obstacle_Overlap_Triangle_Inverted
/// Same as triangle but with 180-degree rotated triangles.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_overlap_triangle_inverted() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 50.0, 40.0, 60.0, 60.0);
    let s2 = add_rect_corners(&mut b, 140.0, 40.0, 150.0, 60.0);
    // Inverted triangles — same bounding boxes, rotated 180 degrees
    let _tri1 = add_rect_corners(&mut b, 70.0, 40.0, 90.0, 60.0);
    let _tri2 = add_rect_corners(&mut b, 70.0, 44.0, 76.5, 50.5);
    let _g1p1 = add_rect_corners(&mut b, 40.0, 30.0, 72.0, 70.0);
    let _g1p2 = add_rect_corners(&mut b, 40.0, 30.0, 75.0, 70.0);
    let _g1p3 = add_rect_corners(&mut b, 40.0, 30.0, 78.0, 70.0);
    let _g1p4 = add_rect_corners(&mut b, 40.0, 30.0, 81.0, 70.0);
    b.route_between(s1, s2);
    let _result = b.run();
}

/// Port: Group_Obstacle_Overlap_Rectangle
/// A rectangle obstacle crosses the group boundaries.
#[test]
#[ignore = "requires group/cluster routing"]
fn group_obstacle_overlap_rectangle() {
    let mut b = ScenarioBuilder::new();
    let s1 = add_rect_corners(&mut b, 50.0, 40.0, 60.0, 60.0);
    let s2 = add_rect_corners(&mut b, 140.0, 40.0, 150.0, 60.0);
    let _blocker = add_rect_corners(&mut b, 70.0, 40.0, 90.0, 60.0);
    let _g1p1 = add_rect_corners(&mut b, 40.0, 30.0, 72.0, 70.0);
    let _g1p2 = add_rect_corners(&mut b, 40.0, 30.0, 75.0, 70.0);
    let _g1p3 = add_rect_corners(&mut b, 40.0, 30.0, 78.0, 70.0);
    let _g1p4 = add_rect_corners(&mut b, 40.0, 30.0, 81.0, 70.0);
    b.route_between(s1, s2);
    let _result = b.run();
}

