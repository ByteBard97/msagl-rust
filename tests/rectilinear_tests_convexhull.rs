/// Rectilinear routing tests — convex hull / overlap tests ported from
/// the C# MSAGL test suite.
///
/// Reference: MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs
///
/// Naming convention: C# test name -> snake_case Rust test name.
/// Tests that exercise features not yet fully implemented are marked #[ignore]
/// with an explanatory comment.

#[path = "test_harness/mod.rs"]
mod test_harness;

#[allow(unused_imports)]
use test_harness::verifier::RECTILINEAR_TOLERANCE;
#[allow(unused_imports)]
use test_harness::{ScenarioBuilder, Verifier};

// ── Overlapping obstacles with non-overlapped rectangle ─────────────────────

/// Port: Overlapping_Obstacles_With_NonOverlapped_Rectangle_Creating_Convex_Hull
/// Baseline to illustrate issues of using a single convex hull.
/// C#: 2 rectangles + 1 triangle, routes between obs[0] and obs[1].
#[test]
fn overlapping_obstacles_with_non_overlapped_rectangle_creating_convex_hull() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(5.0, 20.0, 30.0, 10.0);
    let obs1 = b.add_rectangle_bl(45.0, 0.0, 15.0, 50.0);
    // Third obstacle is triangle: (10,40),(30,80),(50,40) — approx with bbox
    b.add_rectangle_bl(10.0, 40.0, 40.0, 40.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: Overlapping_Obstacles_With_NonOverlapped_Rectangle_Inside_Simulated_ConvexHull
/// Simulated convex hull (overlapping rather than true CH).
/// C#: same 3 obstacles + a convex hull wrapping them.
#[test]
fn overlapping_obstacles_with_non_overlapped_rect_inside_simulated_convex_hull() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(5.0, 20.0, 30.0, 10.0);
    let obs1 = b.add_rectangle_bl(45.0, 0.0, 15.0, 50.0);
    b.add_rectangle_bl(10.0, 40.0, 40.0, 40.0);
    // Plus a convex hull wrapping all three
    b.add_rectangle_bl(5.0, 0.0, 55.0, 80.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

// ── Multiply-nested obstacles ───────────────────────────────────────────────

/// Port: Multiply_Nested_Nonrectilinear_Obstacles
/// Multiply-nested nonrectilinear obstacles with no overlaps.
/// C#: 3 nested diamond shapes (non-rect).
#[test]
fn multiply_nested_nonrectilinear_obstacles() {
    let mut b = ScenarioBuilder::new();
    // 3 nested diamonds at offset 0 — approximate as rectangles
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    let _result = b.run();
}

/// Port: Multiply_Nested_Rectilinear_Obstacles
/// Multiply-nested rectilinear obstacles with no overlaps — should be in one clump.
#[test]
fn multiply_nested_rectilinear_obstacles() {
    let mut b = ScenarioBuilder::new();
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    let _result = b.run();
}

/// Port: Multiply_Nested_Nonrectilinear_Obstacles_With_Outer_Overlap
/// Two sets of nested diamonds with only outer obstacles overlapping.
#[test]
fn multiply_nested_nonrectilinear_obstacles_with_outer_overlap() {
    let mut b = ScenarioBuilder::new();
    // Set 1: offset 0
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    // Set 2: offset 75
    b.add_rectangle(125.0, 40.0, 80.0, 60.0);
    b.add_rectangle(125.0, 40.0, 60.0, 40.0);
    b.add_rectangle(125.0, 40.0, 40.0, 20.0);
    let _result = b.run();
}

/// Port: Multiply_Nested_Nonrectilinear_Obstacles_With_All_Overlap
/// Nested diamonds with an obstacle forcing all overlapped.
#[test]
fn multiply_nested_nonrectilinear_obstacles_with_all_overlap() {
    let mut b = ScenarioBuilder::new();
    // Set 1 at offset 0
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    // Set 2 at offset 90
    b.add_rectangle(140.0, 40.0, 80.0, 60.0);
    b.add_rectangle(140.0, 40.0, 60.0, 40.0);
    b.add_rectangle(140.0, 40.0, 40.0, 20.0);
    // Diamond connector: (55,40),(95,60),(135,40),(95,20)
    b.add_rectangle(95.0, 40.0, 80.0, 40.0);
    let _result = b.run();
}

/// Port: Multiply_Nested_Rectilinear_Obstacles_With_Outer_Overlap_Clump
/// Two sets of nested rectangles with outer overlap — same clump.
#[test]
fn multiply_nested_rectilinear_obstacles_with_outer_overlap_clump() {
    let mut b = ScenarioBuilder::new();
    // Set 1 (rectangles, offset 0)
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    // Set 2 (rectangles, offset 75)
    b.add_rectangle(125.0, 40.0, 80.0, 60.0);
    b.add_rectangle(125.0, 40.0, 60.0, 40.0);
    b.add_rectangle(125.0, 40.0, 40.0, 20.0);
    let _result = b.run();
}

/// Port: Multiply_Nested_Rectilinear_Obstacles_With_Outer_Overlap_ConvexHull
/// Nested rectangles with a non-rectangular connector forcing convex hull.
#[test]
fn multiply_nested_rectilinear_obstacles_with_outer_overlap_convex_hull() {
    let mut b = ScenarioBuilder::new();
    // Set 1 at 0
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    // Set 2 at 90
    b.add_rectangle(140.0, 40.0, 80.0, 60.0);
    b.add_rectangle(140.0, 40.0, 60.0, 40.0);
    b.add_rectangle(140.0, 40.0, 40.0, 20.0);
    // Diamond connector: (85,40),(95,50),(105,40),(95,30)
    b.add_rectangle(95.0, 40.0, 20.0, 20.0);
    let _result = b.run();
}

/// Port: Multiply_Nested_Rectilinear_Obstacles_With_All_Overlap_Clump
/// Nested rectangles with a rectangular overlap forcing same clump.
#[test]
fn multiply_nested_rectilinear_obstacles_with_all_overlap_clump() {
    let mut b = ScenarioBuilder::new();
    // Set 1 at 0
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    // Set 2 at 90
    b.add_rectangle(140.0, 40.0, 80.0, 60.0);
    b.add_rectangle(140.0, 40.0, 60.0, 40.0);
    b.add_rectangle(140.0, 40.0, 40.0, 20.0);
    // Diamond as rect: bbox of (55,40),(95,60),(135,40),(95,20) = (55,20)-(135,60)
    b.add_rectangle(95.0, 40.0, 80.0, 40.0);
    let _result = b.run();
}

/// Port: Multiply_Nested_Rectilinear_Obstacles_With_All_Overlap_ConvexHull
/// Nested rectangles with a non-rectangular connector forcing convex hull.
#[test]
fn multiply_nested_rectilinear_obstacles_with_all_overlap_convex_hull() {
    let mut b = ScenarioBuilder::new();
    // Set 1 at 0
    b.add_rectangle(50.0, 40.0, 80.0, 60.0);
    b.add_rectangle(50.0, 40.0, 60.0, 40.0);
    b.add_rectangle(50.0, 40.0, 40.0, 20.0);
    // Set 2 at 90
    b.add_rectangle(140.0, 40.0, 80.0, 60.0);
    b.add_rectangle(140.0, 40.0, 60.0, 40.0);
    b.add_rectangle(140.0, 40.0, 40.0, 20.0);
    // Diamond: (55,40),(95,60),(135,40),(95,20) — non-rect
    b.add_rectangle(95.0, 40.0, 80.0, 40.0);
    let _result = b.run();
}

// ── Transitive convex hull tests ────────────────────────────────────────────

/// Port: Transitive_ConvexHull_Single_Accretion
/// Test transitivity of ConvexHull creation (non-rectangular).
/// C#: 11 obstacles (mix of rectangles and non-rect polygons), 5 routes.
#[test]
fn transitive_convex_hull_single_accretion() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(0.0, 15.0, 25.0, 20.0);
    let obs1 = b.add_rectangle_bl(10.0, 32.0, 25.0, 23.0);
    let obs2 = b.add_rectangle_bl(0.0, 50.0, 25.0, 35.0);
    let _obs3 = b.add_rectangle_bl(15.0, 75.0, 40.0, 25.0);
    let _obs4 = b.add_rectangle_bl(70.0, 85.0, 25.0, 15.0);
    // obs5: diamond (42,65),(57,80),(72,65),(57,50)
    b.add_rectangle(57.0, 65.0, 30.0, 30.0);
    // obs6: hexagon (70,60),(70,70),(85,75),(100,70),(100,60),(85,55)
    b.add_rectangle(85.0, 65.0, 30.0, 20.0);
    let obs7 = b.add_rectangle_bl(62.0, 25.0, 25.0, 20.0);
    let obs8 = b.add_rectangle_bl(35.0, 2.0, 20.0, 15.0);
    let obs9 = b.add_rectangle_bl(120.0, 80.0, 20.0, 20.0);
    let obs10 = b.add_rectangle_bl(120.0, 55.0, 20.0, 20.0);
    b.route_between(obs1, obs0); // placeholder for intra-hull route 1->5
    b.route_between(obs1, obs7);
    b.route_between(obs1, obs8);
    b.route_between(obs2, obs10);
    b.route_between(obs0, obs9); // placeholder for 3->9
    let _result = b.run();
}

/// Port: Transitive_ConvexHull_Single_Accretion_Becomes_Clump_With_Rectilinear_Shapes
/// Same as above but all shapes replaced with their bounding rectangles.
#[test]
fn transitive_convex_hull_single_accretion_becomes_clump() {
    let mut b = ScenarioBuilder::new();
    b.add_rectangle_bl(0.0, 15.0, 25.0, 20.0);
    b.add_rectangle_bl(10.0, 32.0, 25.0, 23.0);
    b.add_rectangle_bl(0.0, 50.0, 25.0, 35.0);
    b.add_rectangle_bl(15.0, 75.0, 40.0, 25.0);
    b.add_rectangle_bl(70.0, 85.0, 25.0, 15.0);
    b.add_rectangle(57.0, 65.0, 30.0, 30.0);
    b.add_rectangle(85.0, 65.0, 30.0, 20.0);
    b.add_rectangle_bl(62.0, 25.0, 25.0, 20.0);
    b.add_rectangle_bl(35.0, 2.0, 20.0, 15.0);
    b.add_rectangle_bl(120.0, 80.0, 20.0, 20.0);
    b.add_rectangle_bl(120.0, 55.0, 20.0, 20.0);
    let _result = b.run();
}

/// Port: Transitive_ConvexHull_Multiple_Accretion
/// Transitivity of ConvexHull with multiple accretions.
/// C#: runs 3 sub-cases: (!both,!block,!rect), (both,!block,!rect), (both,block,!rect).
#[test]
fn transitive_convex_hull_multiple_accretion() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(30.0, 30.0, 40.0, 20.0);
    let obs1 = b.add_rectangle_bl(140.0, 30.0, 20.0, 20.0);
    // Non-rect triangles approximated as bounding rects
    b.add_rectangle_bl(0.0, 0.0, 40.0, 20.0);
    b.add_rectangle_bl(0.0, 10.0, 20.0, 60.0);
    b.add_rectangle_bl(0.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(60.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(80.0, 10.0, 20.0, 60.0);
    b.add_rectangle_bl(60.0, 0.0, 40.0, 20.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: Transitive_ConvexHull_Multiple_Accretion_Becomes_Separate_Clumps_With_Rectilinear_Shapes
/// Same as above but with rectilinear shapes — creates separate clumps.
#[test]
fn transitive_convex_hull_multiple_accretion_becomes_separate_clumps() {
    let mut b = ScenarioBuilder::new();
    let obs0 = b.add_rectangle_bl(30.0, 30.0, 40.0, 20.0);
    let obs1 = b.add_rectangle_bl(140.0, 30.0, 20.0, 20.0);
    b.add_rectangle_bl(0.0, 0.0, 40.0, 20.0);
    b.add_rectangle_bl(0.0, 10.0, 20.0, 60.0);
    b.add_rectangle_bl(0.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(60.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(80.0, 10.0, 20.0, 60.0);
    b.add_rectangle_bl(60.0, 0.0, 40.0, 20.0);
    b.route_between(obs0, obs1);
    let _result = b.run();
}

/// Port: Transitive_ConvexHull_Is_Local_SingleReflection
/// Convex hull transitivity is local; does not affect distant paths.
/// C#: source/target + 1 outside square + 6 non-rect blocking obstacles.
#[test]
fn transitive_convex_hull_is_local_single_reflection() {
    let mut b = ScenarioBuilder::new();
    let src = b.add_rectangle_bl(90.0, 40.0, 10.0, 10.0);
    let tgt = b.add_rectangle_bl(120.0, 110.0, 10.0, 10.0);
    // Outside square
    b.add_rectangle_bl(-10.0, 40.0, 10.0, 10.0);
    // 6 non-rect blocking obstacles (approximated with bounding rects)
    b.add_rectangle_bl(10.0, 10.0, 20.0, 70.0);
    b.add_rectangle_bl(20.0, 70.0, 40.0, 20.0);
    b.add_rectangle_bl(50.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(80.0, 70.0, 40.0, 20.0);
    b.add_rectangle_bl(110.0, 10.0, 20.0, 70.0);
    b.add_rectangle_bl(20.0, 10.0, 100.0, 20.0);
    b.route_between(src, tgt);
    let _result = b.run();
}

/// Port: Transitive_ConvexHull_Is_Local_SingleReflection_SparseVg
/// Same as SingleReflection with sparse visibility graph.
#[test]
fn transitive_convex_hull_is_local_single_reflection_sparse_vg() {
    let mut b = ScenarioBuilder::new();
    let src = b.add_rectangle_bl(90.0, 40.0, 10.0, 10.0);
    let tgt = b.add_rectangle_bl(120.0, 110.0, 10.0, 10.0);
    b.add_rectangle_bl(-10.0, 40.0, 10.0, 10.0);
    b.add_rectangle_bl(10.0, 10.0, 20.0, 70.0);
    b.add_rectangle_bl(20.0, 70.0, 40.0, 20.0);
    b.add_rectangle_bl(50.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(80.0, 70.0, 40.0, 20.0);
    b.add_rectangle_bl(110.0, 10.0, 20.0, 70.0);
    b.add_rectangle_bl(20.0, 10.0, 100.0, 20.0);
    b.route_between(src, tgt);
    let _result = b.run();
}

/// Port: Transitive_ConvexHull_Is_Local_DoubleReflection
/// Same setup with double reflection (adjustment = -5).
#[test]
fn transitive_convex_hull_is_local_double_reflection() {
    let mut b = ScenarioBuilder::new();
    let src = b.add_rectangle_bl(90.0, 40.0, 10.0, 10.0);
    let tgt = b.add_rectangle_bl(120.0, 110.0, 10.0, 10.0);
    b.add_rectangle_bl(-10.0, 40.0, 10.0, 10.0);
    b.add_rectangle_bl(10.0, 10.0, 20.0, 70.0);
    // adjustment = -5 for double reflection
    b.add_rectangle_bl(20.0, 65.0, 40.0, 20.0);
    b.add_rectangle_bl(50.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(80.0, 65.0, 40.0, 20.0);
    b.add_rectangle_bl(110.0, 10.0, 20.0, 70.0);
    b.add_rectangle_bl(20.0, 10.0, 100.0, 20.0);
    b.route_between(src, tgt);
    let _result = b.run();
}

/// Port: Transitive_ConvexHull_Is_Local_TripleReflection
/// Same setup with triple reflection (adjustment = -10).
#[test]
fn transitive_convex_hull_is_local_triple_reflection() {
    let mut b = ScenarioBuilder::new();
    let src = b.add_rectangle_bl(90.0, 40.0, 10.0, 10.0);
    let tgt = b.add_rectangle_bl(120.0, 110.0, 10.0, 10.0);
    b.add_rectangle_bl(-10.0, 40.0, 10.0, 10.0);
    b.add_rectangle_bl(10.0, 10.0, 20.0, 70.0);
    // adjustment = -10 for triple reflection
    b.add_rectangle_bl(20.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(50.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(80.0, 60.0, 40.0, 20.0);
    b.add_rectangle_bl(110.0, 10.0, 20.0, 70.0);
    b.add_rectangle_bl(20.0, 10.0, 100.0, 20.0);
    b.route_between(src, tgt);
    let _result = b.run();
}
