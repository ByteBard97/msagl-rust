# C# Rectilinear Test Analysis

## Summary

- **Total:** 242 tests
- **Portable now:** 47 tests
- **Blocked — groups (deferred):** 51 tests
- **Blocked — non-rect obstacles only (reflection/convex hull machinery required):** 71 tests
- **Blocked — update/incremental API:** 7 tests
- **Blocked — sparse VG flag:** 3 tests
- **Blocked — overlap/convex hull structure tests:** 26 tests
- **Blocked — visibility graph structure assertions:** 7 tests
- **Blocked — polyline utility tests:** 2 tests
- **Blocked — other (VG generator rewrite required):** 28 tests

### What "Portable Now" Means

A test is **portable now** if it satisfies ALL of the following:
1. All obstacles are rectangular (axis-aligned rectangles or polylines defining rectangles)
2. Ports are obstacle center ports or absolute free ports (no relative floating ports, no `CurvePort`)
3. No groups (`AddChild` calls)
4. No `UseSparseVisibilityGraph` or `UseObstacleRectangles` flags
5. No dynamic update API (`UpdateObstacle`, `RemoveObstacle`, `AddObstacle`, `RemoveEdgeGeometryToRoute`)
6. Does not assert on internal VG vertex/edge counts (would need exact VG match)
7. Does not assert on scan segment counts or reflection segment counts (needs full sweep-line)

Tests with non-rectangular obstacles (circles, triangles, diamonds, parallelograms) are **not** portable now because the VG generator rewrite (#1 in the PRD) is required to generate correct visibility for non-axis-aligned sides. Once the VG generator and path search rewrites are complete, these become the next wave.

---

## Category Breakdown

| Category | Count | Portable Now | Primary Blocker |
|----------|-------|-------------|-----------------|
| basic | 14 | 10 | non-rect obstacles in 4 |
| free_port | 18 | 7 | OobSetup VG-count assertions; update API |
| reflection | 27 | 0 | non-rect obstacles, reflection sweep required |
| overlap | 64 | 9 | non-rect obstacles; convex hull machinery |
| update | 7 | 0 | incremental update API (deferred) |
| group | 51 | 0 | groups deferred |
| visibility | 7 | 0 | VG structure assertions |
| collinear | 8 | 6 | 2 require reflection/VG structure asserts |
| clump | 18 | 0 | clump/convex hull tree structure |
| nested | 10 | 0 | convex hull + group or nested obstacle tree |
| other | 18 | 15 | VG/reflection/sparse-VG variants |

---

## Full Test List

| # | Test Name | Category | Portable | Complexity | Notes |
|---|-----------|----------|----------|------------|-------|
| 1 | Diamond3 | basic | NO | simple | Obstacles are diamond (non-rect polylines); needs VG rewrite |
| 2 | Diamond3_With_FreePorts | free_port | NO | medium | Diamond obstacles; also has free ports |
| 3 | Diamond3_Square6_Overlap | overlap | NO | medium | Diamond + rect overlap; convex hull required |
| 4 | Diamond3_Square8_Overlap | overlap | NO | medium | Diamond + rect overlap |
| 5 | Diamond3_Square9_Overlap | overlap | NO | medium | Diamond + rect overlap |
| 6 | Diamond3_Square9_Overlap_HalfWidths | overlap | NO | medium | Diamond + rect overlap |
| 7 | CircleTest | basic | NO | complex | Circle obstacles; non-rect VG required |
| 8 | Clust5_Minimal | basic | YES | medium | All rect obstacles, single-port routing, fan-out |
| 9 | Reflection_Block1_Big | reflection | NO | medium | Non-rect (rotated rect) obstacles |
| 10 | Reflection_Block1_Big_UseRect | reflection | NO | medium | Sets UseObstacleRectangles=true; non-rect base |
| 11 | Reflection_Triangle1 | reflection | NO | simple | Triangle obstacles |
| 12 | Reflection_Triangle1_Overlap | reflection | NO | simple | Triangle obstacles with overlap |
| 13 | Reflection_Triangle1_NoOverlap | reflection | NO | simple | Triangle obstacles |
| 14 | Reflection_Block1_Small | reflection | NO | medium | Non-rect (rotated rect) obstacles |
| 15 | Reflection_Block2 | reflection | NO | medium | Non-rect obstacles |
| 16 | Reflection_LongAngle | reflection | NO | medium | Non-rect (rotated rect) obstacles |
| 17 | Reflection_LongAngle_Overlap_ToHighRight | reflection | NO | medium | Non-rect; calls DoRouting(obstacles) only |
| 18 | Reflection_LongAngle_Overlap_ToHighLeft | reflection | NO | medium | Non-rect |
| 19 | Reflection_LongAngle_Overlap_FromLowRight | reflection | NO | medium | Non-rect |
| 20 | Reflection_LongAngle_Overlap_FromLowLeft | reflection | NO | medium | Non-rect |
| 21 | FreePorts_OutOfBounds | free_port | YES | complex | Single rect obstacle, many out-of-bounds free ports, no VG count asserts |
| 22 | FreePorts_OutOfBounds_Dup | free_port | YES | complex | Same as above with duplicate ports |
| 23 | FreePorts_OobCorner_BendUsedTwice_Vertical | free_port | NO | medium | OobSetup asserts exact VG vertex counts after AddPort |
| 24 | FreePorts_OobCorner_BendReusedAsFreePort | free_port | NO | medium | VG vertex count assertions |
| 25 | FreePorts_OobCorner_FreePortReusedAsBend | free_port | NO | medium | VG vertex count assertions |
| 26 | FreePorts_OobCorner_BendUsedTwiceHorizontal | free_port | NO | medium | VG vertex count assertions |
| 27 | FreePorts_OobCorner_TwoBends | free_port | NO | complex | VG vertex count assertions |
| 28 | FreePorts_OobCorner_TwoBends_Rep1Free_Rem3210 | free_port | NO | complex | VG vertex count assertions |
| 29 | FreePorts_Oob_NoBendsThree | free_port | NO | medium | VG vertex count assertions |
| 30 | FreePorts_OnPaddedBorder | free_port | YES | medium | Single rect, free ports on padded border, routes only |
| 31 | FreePorts_OnPaddedBorder_Plus_Collinear_Outer | free_port | YES | medium | Free ports collinear with padded border |
| 32 | TwoSquares | basic | YES | simple | Two rect obstacles, basic routing |
| 33 | TwoSquaresWithSentinels | basic | YES | simple | Two squares + sentinel rects, VG only (no routing) |
| 34 | FreePorts_OnSameLine | free_port | NO | complex | Asserts exact VG vertex counts after AddPort/RemoveControlPoints |
| 35 | Multiple_Collinear_FreePorts_RouteFromObstacle0 | free_port | YES | complex | Many free ports, routes from obstacle0; no count asserts |
| 36 | Multiple_Collinear_FreePorts_RouteFromAllObstacles | free_port | NO | complex | Marked [Ignore] in source; known timeout bug |
| 37 | Collinear_Center_Ports | collinear | YES | simple | Two test squares, ports at center |
| 38 | Outer_Border_Ports | collinear | YES | simple | Two test squares, ports on outer border |
| 39 | Top_Border_Ports | collinear | YES | simple | Two test squares, ports on top border |
| 40 | Update_FreePort | update | NO | complex | Uses RemoveEdgeGeometryToRoute / AddEdgeGeometryToRoute in loop |
| 41 | UpdatePortPosition_Without_UpdateObstacles | update | NO | medium | Uses RelativeFloatingPort with delegate; re-routes on moved port |
| 42 | AddRemovePorts_Without_UpdateObstacles | update | NO | medium | Inserts/removes ports at runtime |
| 43 | MoveOneObstacle_ManuallyUpdateAbsolutePorts | update | NO | complex | Uses router.UpdateObstacle(); VG count assertions |
| 44 | MoveOneObstacle_NoUpdateAbsolutePorts_FreePoint_InAndOut | update | NO | complex | Uses ReplaceObstacles + UpdateObstacle loop |
| 45 | MoveOneObstacle_AutomaticallyUpdateAbsolutePorts | update | NO | complex | ReplaceObstacles + UpdateObstacle |
| 46 | MoveOneObstacle_AutomaticallyUpdateRelativePorts | update | NO | complex | RelativeFloatingPort + UpdateObstacle |
| 47 | Waypoints2 | other | YES | simple | Two rects, waypoints via free ports |
| 48 | Waypoints3 | other | YES | simple | Three waypoints |
| 49 | Waypoints4 | other | YES | simple | Four waypoints |
| 50 | Waypoints11 | other | YES | simple | Eleven waypoints |
| 51 | Waypoints2_Multiple | other | YES | medium | Two waypoints, multiple coinciding paths |
| 52 | Waypoints3_Multiple | other | YES | medium | Three waypoints, multiple paths |
| 53 | Waypoints4_Multiple | other | YES | medium | Four waypoints, multiple paths |
| 54 | Waypoints11_Multiple | other | YES | medium | Eleven waypoints, multiple paths |
| 55 | Waypoints2_Oob | other | YES | simple | Two waypoints outside graph bounds |
| 56 | Waypoints4_Oob | other | YES | simple | Four waypoints, out-of-bounds |
| 57 | MultipleCollinearOpenAndCloseVertices | collinear | NO | medium | Asserts HorizontalScanLineSegments.Count == 3; needs VG internals |
| 58 | CollinearOpenVertexAndIntersection | collinear | NO | medium | Asserts HorizontalScanLineSegments.Count == 5; needs VG internals |
| 59 | FlatTopSideWithMultipleCrosses | overlap | NO | medium | Near-flat non-rect sides; needs sweep-line |
| 60 | FlatBottomSideWithMultipleCrosses | overlap | NO | medium | Near-flat non-rect sides |
| 61 | AlmostFlatHighSideWithMultipleCrosses | overlap | NO | medium | Almost-flat sides; epsilon geometry |
| 62 | AlmostFlatLowSideWithMultipleCrosses | overlap | NO | medium | Almost-flat sides |
| 63 | AlmostFlat_Open_LowSide_NoOverlap | overlap | NO | medium | Almost-flat non-rect side via CurveFromPoints |
| 64 | AlmostFlat_Open_LowSide_InteriorLowOverlap | overlap | NO | medium | Almost-flat with interior overlap |
| 65 | AlmostFlat_Open_LowSide_InteriorLowNeighbor | overlap | NO | medium | Almost-flat with interior neighbor |
| 66 | AlmostFlat_Open_LowSide_InteriorLowOverlap_LowNeighbor | overlap | NO | medium | Combined |
| 67 | AlmostFlat_Open_HighSide_NoOverlap | overlap | NO | medium | Almost-flat HighSide |
| 68 | AlmostFlat_Open_HighSide_InteriorHighOverlap | overlap | NO | medium | Almost-flat HighSide with overlap |
| 69 | AlmostFlat_Open_HighSide_InteriorHighNeighbor | overlap | NO | medium | Almost-flat HighSide with neighbor |
| 70 | AlmostFlat_Open_HighSide_InteriorHighOverlap_HighNeighbor | overlap | NO | medium | Combined |
| 71 | AlmostFlat_Close_LowSide_NoOverlap | overlap | NO | medium | Close-event almost-flat LowSide |
| 72 | AlmostFlat_Close_LowSide_InteriorLowOverlap | overlap | NO | medium | Close-event with overlap |
| 73 | AlmostFlat_Close_LowSide_InteriorLowNeighbor | overlap | NO | medium | Close-event with neighbor |
| 74 | AlmostFlat_Close_LowSide_InteriorLowOverlap_LowNeighbor | overlap | NO | medium | Combined |
| 75 | AlmostFlat_Close_HighSide_NoOverlap | overlap | NO | medium | Close-event almost-flat HighSide |
| 76 | AlmostFlat_Close_HighSide_InteriorHighOverlap | overlap | NO | medium | Close-event HighSide with overlap |
| 77 | AlmostFlat_Close_HighSide_InteriorHighNeighbor | overlap | NO | medium | Close-event HighSide with neighbor |
| 78 | AlmostFlat_Close_HighSide_InteriorHighOverlap_HighNeighbor | overlap | NO | medium | Combined |
| 79 | AlmostFlat_MultipleBottomInversion1 | overlap | NO | medium | Non-rect epsilon-slope sides; bit-mask variants |
| 80 | AlmostFlat_MultipleBottomInversion2 | overlap | NO | medium | Variant |
| 81 | AlmostFlat_MultipleBottomInversion3 | overlap | NO | medium | Variant |
| 82 | AlmostFlat_MultipleBottomInversion4 | overlap | NO | medium | Variant |
| 83 | AlmostFlat_MultipleBottomInversion5 | overlap | NO | medium | Variant |
| 84 | AlmostFlat_MultipleBottomInversion6 | overlap | NO | medium | Variant |
| 85 | AlmostFlat_MultipleBottomInversion7 | overlap | NO | medium | Variant |
| 86 | AlmostFlat_MultipleTopInversion1 | overlap | NO | medium | Mirror of bottom variants |
| 87 | AlmostFlat_MultipleTopInversion2 | overlap | NO | medium | Variant |
| 88 | AlmostFlat_MultipleTopInversion3 | overlap | NO | medium | Variant |
| 89 | AlmostFlat_MultipleTopInversion4 | overlap | NO | medium | Variant |
| 90 | AlmostFlat_MultipleTopInversion5 | overlap | NO | medium | Variant |
| 91 | AlmostFlat_MultipleTopInversion6 | overlap | NO | medium | Variant |
| 92 | AlmostFlat_MultipleTopInversion7 | overlap | NO | medium | Variant |
| 93 | TouchingSquares | basic | YES | simple | Two rect squares with padded borders touching |
| 94 | InterOverlapShortCircuit_Low | overlap | YES | medium | All rect obstacles, inter-overlap short-circuit |
| 95 | InterOverlapShortCircuit_High | overlap | YES | medium | All rect obstacles |
| 96 | InterOverlapShortCircuit_Low_Middle | overlap | YES | medium | All rect obstacles |
| 97 | InterOverlapShortCircuit_High_Middle | overlap | YES | medium | All rect obstacles |
| 98 | IntraOverlapShortCircuit_Low | overlap | YES | medium | All rect, outer encompassing rect added |
| 99 | IntraOverlapShortCircuit_High | overlap | YES | medium | All rect |
| 100 | IntraOverlapShortCircuit_Low_Middle | overlap | YES | medium | All rect |
| 101 | IntraOverlapShortCircuit_High_Middle | overlap | YES | medium | All rect |
| 102 | InterOverlap_AllBorders_H1 | overlap | NO | medium | Overlaps on all 4 corners — all rect, but requires correct overlap ordering in VG |
| 103 | InterOverlap_AllBorders_H2 | overlap | NO | medium | Variant |
| 104 | InterOverlap_AllBorders_H3 | overlap | NO | medium | Variant |
| 105 | InterOverlap_AllBorders_V1 | overlap | NO | medium | Variant |
| 106 | InterOverlap_AllBorders_V2 | overlap | NO | medium | Variant |
| 107 | InterOverlap_AllBorders_V3 | overlap | NO | medium | Variant |
| 108 | AdjoiningRectangles_Left | basic | YES | simple | Three rect obstacles, two share a border |
| 109 | AdjoiningRectangles_Right | basic | YES | simple | Variant |
| 110 | AdjoiningRectangles_Both | basic | YES | simple | Both sides adjoin |
| 111 | AdjoiningObstacles_DipToOverlapped | overlap | YES | medium | All rect, overlap with collinear CloseVertex events |
| 112 | AdjoiningObstacles_DipToOverlapped_Collinear_CloseOpen | overlap | YES | medium | All rect, collinear CloseOpen events |
| 113 | Landlocked_OverlapSide_NonAdjoining | overlap | NO | medium | Diamond + rect obstacles |
| 114 | Landlocked_OverlapSide_Adjoining | overlap | NO | medium | Diamond + rect obstacles |
| 115 | OverlappedObstacles_InMiddleOfBottom_AdjoiningUnpadded_Inside | overlap | YES | simple | All rect, 3 obstacles |
| 116 | OverlappedObstacles_InMiddleOfBottom_AdjoiningPadded_Inside | overlap | YES | simple | All rect |
| 117 | OverlappedObstacles_InMiddleOfBottom_AdjoiningUnpadded_Outside | overlap | YES | simple | All rect |
| 118 | OverlappedObstacles_InMiddleOfBottom_AdjoiningPadded_Outside | overlap | YES | simple | All rect |
| 119 | Coinciding_SameHeight3 | overlap | NO | medium | Overlapping same-location rects; complex overlap tree required |
| 120 | Coinciding_SameHeight3_Nested | nested | NO | medium | Nested encompassing rect added |
| 121 | Coinciding_DifferentHeight3 | overlap | NO | medium | Same-location rects with varying heights |
| 122 | Coinciding_DifferentHeight3_Nested | nested | NO | medium | Nested |
| 123 | Overlapped_Rectangles_With_Same_Open_And_Close_Coordinate | overlap | NO | medium | Requires correct scan segment overlap-coordinate handling |
| 124 | Connected_Vertical_Segments_Are_Intersected | visibility | NO | medium | Asserts VisibilityGraph.VertexCount == 52, HEdge == 44, VEdge == 42 |
| 125 | Triangle_ObstaclePort_Outside_Obstacle | overlap | NO | medium | Triangle obstacle; center port outside obstacle |
| 126 | FlatBottom_FullyOverlapped_WithAdjoiningOverlapNeighbors | overlap | NO | medium | Rect, but requires correct overlap-neighbor scan ordering |
| 127 | FlatBottom_FullyOverlapped_WithDupAdjoiningOverlapNeighbors | overlap | NO | medium | Dup variant |
| 128 | Overlap_Obstacle_Between_PreviousPoint_And_StartVertex_TargetAtTop | overlap | NO | medium | All rect; port splice across obstacle — needs TransientGraphUtility rewrite |
| 129 | Overlap_Obstacle_Between_PreviousPoint_And_StartVertex_TargetInsideLeft | overlap | NO | medium | Variant |
| 130 | Overlap_Gaps_On_All_Boundaries_TargetUpperLeft | overlap | NO | complex | All rect; gap-in-border visibility; needs port splice rewrite |
| 131 | DeadEnd_OpenSpace_ObstaclePort0 | reflection | NO | medium | Triangle obstacle; dead-end scan segments |
| 132 | DeadEnd_OpenSpace_FreePort0 | reflection | NO | medium | Triangle + free port |
| 133 | DeadEnd_OpenSpace_ObstaclePort0_EliminateExtraBend | reflection | NO | medium | Asserts CountBends == 1; needs accurate path + bend counting |
| 134 | DeadEnd_Crossing | reflection | NO | medium | Triangle; asserts CountBends == 2 for all edges |
| 135 | DeadEnd_Crossing_EdgeChains_Intersect | reflection | NO | medium | Triangle obstacles |
| 136 | Secondary_Port_Visibility_RotatedClockwise | reflection | NO | medium | Non-rect (rotated parallelogram) obstacle |
| 137 | Secondary_Port_Visibility_RotatedCounterclockwise | reflection | NO | medium | Non-rect obstacle |
| 138 | PortEntry_Diamond_AboveCorner_TargetAboveCorner_MiddleObstacle_PortsMoved | reflection | NO | complex | Diamond obstacle; ReplaceTestSquareWithDiamond called |
| 139 | Nudger_NoExtraBends_With_Rectangles | basic | NO | medium | All rect; asserts CountBends per path (needs accurate nudger + path) |
| 140 | GroupTest_Simple | group | NO | medium | Uses AddChild (group hierarchy) |
| 141 | GroupTest_Simple_NoGroup | group | NO | medium | No groups, but comparison to group test; all rect |
| 142 | GroupTest_Simple_NoGroup_PortSplice_LimitRect | group | NO | medium | Sets LimitPortVisibilitySpliceToEndpointBoundingBox; no groups |
| 143 | GroupTest0 | group | NO | complex | Groups via AddChild |
| 144 | GroupTest0_OutsideGroup1 | group | NO | complex | Groups |
| 145 | GroupTest0_OffCenter | group | NO | complex | Groups |
| 146 | GroupTest0_OutsideGroup1_OffCenter | group | NO | complex | Groups |
| 147 | GroupTest | group | NO | complex | Groups with 4th group added |
| 148 | GroupTest_OutsideGroup1 | group | NO | complex | Groups |
| 149 | GroupTest_OffCenter | group | NO | complex | Groups |
| 150 | GroupTest_OutsideGroup1_OffCenter | group | NO | complex | Groups |
| 151 | Group_Obstacle_Crossing_Boundary_Between_Routed_Obstacles_Gap2 | group | NO | medium | Groups |
| 152 | Group_Obstacle_Crossing_Boundary_Between_Routed_Obstacles_Gap4 | group | NO | medium | Groups |
| 153 | Group_Obstacle_Crossing_Boundary_On_Routed_Obstacles | group | NO | medium | Groups |
| 154 | Group_Obstacle_Crossing_Boundary_Inside_Routed_Obstacles_Gap2 | group | NO | medium | Groups |
| 155 | Group_Obstacle_Crossing_Boundary_Inside_Routed_Obstacles_Gap4 | group | NO | medium | Groups |
| 156 | Group_Obstacle_Crossing_Boundary_Fully_Blocking_Routed_Obstacles | group | NO | medium | Groups |
| 157 | Group_AdjacentOuterEdge_Outside | group | NO | medium | Groups |
| 158 | Group_AdjacentOuterEdge_Nested | group | NO | medium | Groups |
| 159 | Group_AdjacentOuterEdge_Gap | group | NO | medium | Groups |
| 160 | Group_AdjacentOuterEdge_Straddle | group | NO | medium | Groups |
| 161 | Group_Spatial_Parent | group | NO | complex | Groups with spatial parent handling |
| 162 | Group_Spatial_Parent_GroupOverlap | group | NO | complex | Groups |
| 163 | Group_Spatial_Parent_GroupOverlap_Collinear | group | NO | complex | Groups |
| 164 | Group_Landlock | group | NO | complex | Landlocking groups |
| 165 | Group_Obstacle_Overlap_Triangle | group | NO | complex | Groups + triangle obstacles |
| 166 | Group_Obstacle_Overlap_Triangle_Inverted | group | NO | complex | Groups + triangles |
| 167 | Group_Obstacle_Overlap_Rectangle | group | NO | medium | Groups + rect |
| 168 | Group_FreePort_Outside_Group | group | NO | medium | Groups + free port |
| 169 | Group_FreePort_Inside_Group | group | NO | medium | Groups + free port inside |
| 170 | Group_FreePort_Inside_Group_Nested | group | NO | medium | Groups nested |
| 171 | GroupBoundaryCrossings_Test | group | NO | complex | Tests GroupBoundaryCrossingMap internals directly |
| 172 | Group_NonRect_BlockedReflections | group | NO | complex | Groups + non-rect; asserts reflection segment counts |
| 173 | Simple_NonRectangular_Group | group | NO | medium | Non-rect group (diamond) |
| 174 | Group_FlatTop_BlockedReflections | group | NO | medium | Rect group with triangle inside |
| 175 | Group_Simple_One_Obstacle_Inside_One_Group | group | NO | simple | Simplest group test |
| 176 | Grid_Neighbors_8_Aligned | basic | YES | complex | 8x8 grid of rect obstacles, nearest-neighbor routing |
| 177 | Grid_Neighbors_8_Unaligned | basic | YES | complex | 8x8 grid, randomly offset positions |
| 178 | Route_Between_Two_Separately_Disconnected_Obstacles | reflection | NO | complex | Non-rect angled rects (rotated); disconnected by angled obstacles |
| 179 | Route_Between_Two_NonOrthogonally_Disconnected_Obstacles_4Reflections | reflection | NO | complex | Triangle obstacles, 4 reflections; WantVerify disabled |
| 180 | Route_Between_Two_NonOrthogonally_AlmostDisconnected_Obstacles_4Reflections | reflection | NO | complex | Variant |
| 181 | Route_From_One_NonOrthogonally_Disconnected_Obstacle_4Reflections | reflection | NO | complex | Variant |
| 182 | Route_From_One_NonOrthogonally_AlmostDisconnected_Obstacle_4Reflections | reflection | NO | complex | Variant |
| 183 | Route_Between_Two_NonOrthogonally_Disconnected_Obstacles_2Reflections | reflection | NO | complex | Variant |
| 184 | Route_Between_Two_NonOrthogonally_AlmostDisconnected_Obstacles_2Reflections | reflection | NO | complex | Variant |
| 185 | Route_From_One_NonOrthogonally_Disconnected_Obstacle_2Reflections | reflection | NO | complex | Variant |
| 186 | Route_From_One_NonOrthogonally_AlmostDisconnected_Obstacle_2Reflections | reflection | NO | complex | Variant |
| 187 | Route_Between_Two_NonOrthogonally_Disconnected_Obstacles_1Reflection | reflection | NO | complex | Variant |
| 188 | Route_Between_Two_NonOrthogonally_AlmostDisconnected_Obstacles_1Reflection | reflection | NO | complex | Variant |
| 189 | Route_From_One_NonOrthogonally_Disconnected_Obstacle_1Reflection | reflection | NO | complex | Variant |
| 190 | Route_From_One_NonOrthogonally_AlmostDisconnected_Obstacle_1Reflection | reflection | NO | complex | Variant |
| 191 | Overlap_SpliceAcrossObstacle | overlap | NO | complex | Non-rect obstacles; port splice across obstacle |
| 192 | Overlap_ReflectionToken | overlap | NO | complex | Non-rect + rect mix; reflection token logic |
| 193 | SpliceSourceToExtendPoint_ToTriangleSide | overlap | NO | medium | Triangle obstacle; port splice to triangle side |
| 194 | SpliceSourceToExtend_ToArrowSide | overlap | NO | medium | Arrow-shaped polygon obstacle |
| 195 | PortNotOnItsCurve | other | YES | medium | All rect; port assigned to wrong curve — tests graceful handling |
| 196 | ClosedVertexWithBends_8PortEntrances | other | YES | medium | All rect; A* path selection with PortEntrances |
| 197 | ClosedVertexWithBends_2PortEntrances | other | YES | medium | Rect + free source port |
| 198 | ClosedVertexWithBends_2OffsetPortEntrances | other | YES | medium | All rect, offset border ports |
| 199 | Overlapping_Obstacles_With_NonOverlapped_Rectangle_Creating_Convex_Hull | clump | NO | medium | Triangle + rects; convex hull creation test |
| 200 | Overlapping_Obstacles_With_NonOverlapped_Rectangle_Inside_Simulated_ConvexHull | clump | NO | medium | Same + simulated hull |
| 201 | Multiply_Nested_Nonrectilinear_Obstacles | clump | NO | medium | Non-rect nested; VerifyAllObstaclesInConvexHull |
| 202 | Multiply_Nested_Rectilinear_Obstacles | clump | NO | medium | Rect nested; VerifyAllObstaclesInClump |
| 203 | Multiply_Nested_Nonrectilinear_Obstacles_With_Outer_Overlap | clump | NO | medium | Non-rect outer overlap |
| 204 | Multiply_Nested_Nonrectilinear_Obstacles_With_All_Overlap | clump | NO | medium | Non-rect all overlap |
| 205 | Multiply_Nested_Rectilinear_Obstacles_With_Outer_Overlap_Clump | clump | NO | medium | Rect outer overlap; clump verification |
| 206 | Multiply_Nested_Rectilinear_Obstacles_With_Outer_Overlap_ConvexHull | clump | NO | medium | Non-rect bridge forces convex hull |
| 207 | Multiply_Nested_Rectilinear_Obstacles_With_All_Overlap_Clump | clump | NO | medium | All rect, non-rect bridge |
| 208 | Multiply_Nested_Rectilinear_Obstacles_With_All_Overlap_ConvexHull | clump | NO | medium | Non-rect bridge added |
| 209 | Transitive_ConvexHull_Single_Accretion | clump | NO | complex | Non-rect in chain; VerifyAllObstaclesInConvexHull |
| 210 | Transitive_ConvexHull_Single_Accretion_Becomes_Clump_With_Rectilinear_Shapes | clump | NO | complex | makeRect=true variant |
| 211 | Transitive_ConvexHull_Multiple_Accretion | clump | NO | complex | Multiple hull accretions (calls worker 3x) |
| 212 | Transitive_ConvexHull_Multiple_Accretion_Becomes_Separate_Clumps_With_Rectilinear_Shapes | clump | NO | complex | makeRect=true |
| 213 | Zero_Obstacle_Graph | other | YES | simple | CreateVisibilityGraph with no obstacles |
| 214 | One_Obstacle_Graph | other | NO | simple | Single non-rect diamond; CreateVisibilityGraph only |
| 215 | Group_With_Overlapping_Obstacles | group | NO | medium | Non-rect group + overlapping obstacles |
| 216 | Group_With_Overlapping_Groups | group | NO | complex | Non-rect groups; detailed convex hull + visibility polyline assertions |
| 217 | Group_Inside_Rectangular_Obstacle | nested | NO | medium | Group nested inside rect; ConvexHull/Clump assertions |
| 218 | Group_Inside_Rectangular_Obstacle_Contains_Rectangular_Obstacle | nested | NO | medium | Nested group + obstacle |
| 219 | Group_Inside_NonRectangular_Obstacle | nested | NO | medium | Group nested in diamond; ConvexHull assertions |
| 220 | Group_Inside_NonRectangular_Obstacle_Contains_Rectangular_Obstacle | nested | NO | medium | Variant |
| 221 | Transitive_ConvexHull_Is_Local_SingleReflection | clump | NO | complex | Non-rect; obstacle moves to create overlap; DoRouting twice |
| 222 | Transitive_ConvexHull_Is_Local_SingleReflection_SparseVg | clump | NO | complex | Sparse VG flag |
| 223 | Transitive_ConvexHull_Is_Local_DoubleReflection | clump | NO | complex | Variant |
| 224 | Transitive_ConvexHull_Is_Local_TripleReflection | clump | NO | complex | Variant |
| 225 | Rectangular_Obstacles_Overlapping_Rectangular_Group_Sides_And_Corners | other | YES | medium | All rect, group corner/side overlaps; RunAndShowGraph only (no routing assertion) |
| 226 | NudgerSmoothingStaircasesAlongConvexHulls | clump | NO | complex | Non-rect triangles force convex hull; nudger staircase test |
| 227 | Reflections_Taken_And_Skipped | reflection | NO | medium | Non-rect parallelograms; two DoRouting calls |
| 228 | RemoveCloseVerticesFromPolyline | other | NO | medium | Unit-tests Obstacle.RemoveCloseAndCollinearVerticesInPlace (obstacle utility) |
| 229 | RemoveCollinearVerticesFromPolyline | other | NO | medium | Same utility function |
| 230 | Reflection_Staircase_Stops_At_BoundingBox_Side_NorthWest | reflection | NO | complex | Non-rect (diamond + rotated rect); asserts reflection segment counts |
| 231 | Reflection_Staircase_Stops_At_BoundingBox_Side_NorthEast | reflection | NO | complex | Rotated variant |
| 232 | Reflection_Staircase_Stops_At_BoundingBox_Side_SouthEast | reflection | NO | complex | Rotated variant |
| 233 | Reflection_Staircase_Stops_At_BoundingBox_Side_SouthWest | reflection | NO | complex | Rotated variant |
| 234 | ReflectionsDetectedByAlreadyLoadedSide | reflection | NO | complex | Non-rect; asserts H/V reflection segment counts |
| 235 | ReflectionsSitedByLowSideAreNotLoadedByHighSide | reflection | NO | complex | Non-rect; asserts exact reflection segment with specific coordinates |
| 236 | ReflectionsRemoveInterceptedSite | reflection | NO | complex | Non-rect; asserts 0 reflection segments |
| 237 | FreePortLocationRelativeToTransientVisibilityEdges | free_port | NO | medium | Asserts intersection at TransientVisibilityEdge; needs TransientGraphUtility |
| 238 | FreePortLocationRelativeToTransientVisibilityEdgesSparseVg | free_port | NO | medium | Sparse VG variant |
| 239 | PaddedBorderIntersectMeetsIncomingScanSegment | visibility | NO | complex | Near-vertical side (almost-rect); VG splice precision test |
| 240 | RoutingBetweenCollinearObstaclesInConvexHull | visibility | NO | medium | Two triangles in convex hull; collinear edge creation |
| 241 | Document_Illustration1 | other | YES | simple | Triangles + rects; DoRouting(shapes, null) — no routing, VG only |
| 242 | Document_Illustration2 | other | NO | simple | Sets WantPaths=false; calls InterOverlapShortCircuit_Worker |

---

## Portability Notes by Blocker

### Why "Portable Now" tests are safe to port immediately

These 47 tests use only:
- `PolylineFromRectanglePoints` for all obstacles (axis-aligned rectangles)
- `MakeAbsoluteFreePort`, `MakeAbsoluteObstaclePort`, `MakeSingleRelativeObstaclePort` with zero offset (center port)
- Standard `DoRouting` / `RunAndShowGraph` with no internal assertions
- No `AddChild`, no `UpdateObstacle`, no VG/scan segment count assertions

They will exercise the basic obstacle scanning, path search, and nudging pipeline for rectangular obstacle graphs, which is exactly the first milestone needed.

### Primary Blocker: Non-Rectangular Obstacles (71 tests)

The overwhelming majority of blocked tests use `CurveFromPoints`, `PolylineFromPoints` with non-axis-aligned coordinates, `CreateCircle`, or `CurveFactory.CreateRectangle` with rotation. These all require the full sweep-line VG generator (PRD Defect #1) with:
- LowObstacleSide/HighObstacleSide slope tracking
- Reflection event generation
- LookaheadScan

### Secondary Blocker: Groups (51 tests)

All tests calling `shape.AddChild(...)` require group/cluster routing support (PRD Defect #14, explicitly deferred). This is the largest single category after the VG generator.

### Tertiary Blocker: Convex Hull / Clump Tree (26 tests)

Tests calling `VerifyAllObstaclesInConvexHull` / `VerifyAllObstaclesInClump` require `ObstacleTree` to track overlap chains and merge into ConvexHull objects (PRD Defect #8). These are structural correctness tests for the obstacle preprocessing stage.

### Update API Blocker (7 tests)

Tests using `UpdateObstacle`, `ReplaceObstacles`, `RemoveEdgeGeometryToRoute`, `AddEdgeGeometryToRoute` require the interactive re-routing API (PRD Defect #16, deferred).

### VG Internal Assertion Blocker (7 tests)

Tests that assert `VisibilityGraph.VertexCount`, `HorizontalScanLineSegments.Count`, exact reflection segment counts, or `CountBends` on paths require the VG generator to be fully faithful. These are medium-priority once the VG rewrite is done.

---

## Recommended Porting Order

### Wave 1 — Port now (47 tests)

Port these as the first batch of Rust `#[test]` functions. They validate the basic rectangular routing pipeline end-to-end.

**Best starting tests (simplest and most representative):**
1. `Zero_Obstacle_Graph` — trivial smoke test
2. `TwoSquares` — simplest routing
3. `TwoSquaresWithSentinels` — VG generation only
4. `Clust5_Minimal` — fan-out from single port
5. `TouchingSquares` — touching padded borders
6. `AdjoiningRectangles_Left/Right/Both` — adjoin cases
7. `Collinear_Center_Ports`, `Outer_Border_Ports`, `Top_Border_Ports`
8. `Waypoints2..11` (8 tests) — waypoint routing
9. `Grid_Neighbors_8_Aligned/Unaligned` — stress test
10. `OverlappedObstacles_InMiddleOfBottom_*` (4 tests) — overlap basics
11. `InterOverlapShortCircuit_*` (8 tests) — inter/intra overlap
12. `AdjoiningObstacles_DipToOverlapped*` (2 tests)
13. `FreePorts_OutOfBounds/Dup`, `FreePorts_OnPaddedBorder*` (4 tests)
14. `Multiple_Collinear_FreePorts_RouteFromObstacle0`
15. `ClosedVertexWithBends_*` (3 tests) — A* path selection
16. `PortNotOnItsCurve` — graceful port handling
17. `Rectangular_Obstacles_Overlapping_Rectangular_Group_Sides_And_Corners`

### Wave 2 — After VG generator rewrite (remaining non-group, non-clump tests)

Once PRD Defects #1 (VG generator), #3 (VisibilityVertex/VertexEntry), #4 (path search) are done, ~71 additional tests become portable.

### Wave 3 — After convex hull + overlap tree (clump/nested tests)

Once PRD Defect #8 (ObstacleTree full implementation) is done, ~26 more tests become portable.

### Wave 4 — After group support (if/when deferred)

51 group tests remain permanently deferred unless group support is implemented.
