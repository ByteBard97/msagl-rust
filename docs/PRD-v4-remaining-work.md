# PRD v4: Remaining Work

**Supersedes:** `docs/PRD-v3-api-audit.md`
**Date:** 2026-03-28
**Source of truth:** `bash tools/compliance-check.sh`

---

## How to Read This Document

Every item here maps to a compliance check failure. When all items are resolved,
`bash tools/compliance-check.sh` exits 0 and the codebase is a faithful port.

**Compliance check catches:**
- `#[ignore]` in tests/ — hidden failing tests
- `// TODO` in src/routing/ — incomplete implementations
- Banned patterns (bbox approximations, todo!(), etc.)
- C# test parity (all 243 tests must exist)

---

## Current State (2026-03-28)

```
cargo test: 806 passing, 0 failing, 4 ignored
C# test parity: 243/243 ported
Compliance failures: 8 (5 TODOs + 3 #[ignore])
```

The 4 ignored tests: 3 polygon obstacle + 1 QPSC solver (via `fixture_test_ignored!` macro).

---

## Outstanding Items

### P1 — Non-Rectangular Obstacle Support

**Root cause of 3 `#[ignore]` tests and 3 of 5 TODOs.**

The VG generator currently routes around bounding boxes for non-rectangular (polygon)
obstacles. The actual polygon boundary is stored but not used for routing.

#### P1a — Fix padded polyline in `obstacle.rs`

**File:** `src/routing/obstacle.rs:60`

**TODO:** When bend/reflection events are implemented in the VG generator,
non-rectangular shapes should use `create_padded_polyline()` instead of the
bounding-box approximation.

**C# reference:** `Obstacle.cs` — `CreatePaddedPolyline()` uses actual polygon points,
not bbox corners. The padded polyline is the shape the VG sweep processes.

**Fix:** Call `create_padded_polyline()` for non-rectangular shapes. The function
already exists — it just isn't called.

**Complexity:** ~10 lines.

---

#### P1b — Polygon port entrances in `port_manager_entrances.rs`

**File:** `src/routing/port_manager_entrances.rs:128`

**TODO:** Implement perpendicular entrance for non-rectangular obstacles.

**C# reference:** `PortManager.cs` — `CreateObstaclePortEntrancesFromPoints()` calls
`CreatePortEntrancesAtBorderIntersections()` which computes entrance points by
intersecting the port location with each polygon side. For rectangles this is trivial
(4 sides); for polygons it requires iterating the polyline segments.

**Fix:** Implement the polygon side intersection logic. The convex hull and polyline
types already exist in `src/geometry/`.

**Complexity:** ~80-120 lines.

---

#### P1c — Wire convex hull into `obstacle_tree.rs`

**File:** `src/routing/obstacle_tree.rs`

**No TODO marker** — this is a missing integration.

**What's needed:** `ObstacleTree.CreateConvexHulls()` from C# detects pairs of
overlapping non-rectangular obstacles and merges them into a convex hull group.
The convex hull algorithm lives at `src/geometry/convex_hull.rs` (fully implemented).
It just isn't called from the obstacle tree.

**C# reference:** `ObstacleTree.cs` — `CreateConvexHulls()` called during
`InitObstaclesForRouting()`.

**Fix:** Call `calculate_convex_hull()` on overlapping polygon obstacle point sets,
store the hull in `overlap_convex_hull.rs`, register it back on the obstacles.

**Complexity:** ~100-150 lines.

---

#### P1d — Un-ignore 3 tests in `rectilinear_tests_obstacleport.rs`

Once P1a–P1c are done:
- Remove `#[ignore]` from `triangle_obstacle_port_outside_obstacle`
- Remove `#[ignore]` from `dead_end_open_space_obstacle_port0`
- Remove `#[ignore]` from `dead_end_open_space_obstacle_port0_eliminate_extra_bend`

These are the acceptance criteria for P1.

---

### P2 — Wire Two Router Flags

**Independent of P1. Trivial.**

#### P2a — `route_to_center_of_obstacles`

**File:** `src/routing/rectilinear_edge_router.rs:159`

**TODO:** Flag is stored but not wired into the routing pipeline.
Wire it into port-entrance selection in `port_manager_entrances.rs`.

**C# reference:** `RectilinearEdgeRouter.cs` — when `RouteToCenterOfObstacles = true`,
`PortManager.FindVertices()` uses the obstacle center as the port location instead
of the border intersection.

**Fix:** Pass `self.route_to_center` into `PortManager` and use it in entrance
selection. ~15 lines.

---

#### P2b — `limit_port_visibility_splice_to_endpoint_bounding_box`

**File:** `src/routing/rectilinear_edge_router.rs:171`

**TODO:** Flag is stored but not wired into the routing pipeline.
Wire it into the TGU splice step in `transient_graph_utility.rs`.

**C# reference:** `RectilinearEdgeRouter.cs` — when `LimitPortVisibilitySpliceToEndpointBoundingBox = true`,
`GetPortSpliceLimitRectangle()` returns the bounding box of the two endpoint locations,
restricting how far visibility splicing extends.

**Fix:** Pass `self.limit_splice_to_bbox` into `TransientGraphUtility` and call
`GetPortSpliceLimitRectangle()` conditionally. ~20 lines.

---

### P3 — `segment_crosses_a_non_group_obstacle`

**File:** `src/routing/transient_graph_utility_helpers.rs:493`

**TODO:** Replace approximate `segment_crosses_an_obstacle` with the correct
`segment_crosses_a_non_group_obstacle` when group support is added.

**What this does:** C# asserts that the extended edge path does not cross a
non-group obstacle (group obstacles are intentionally permeable). The current
Rust code comments out this assertion entirely, which means invalid paths can
go unchecked.

**Dependency:** Requires knowing which obstacles are group obstacles vs regular.
Group obstacle support is implemented — `Obstacle` has an `is_group` field.

**Fix:** Implement `segment_crosses_a_non_group_obstacle` as a spatial query
that filters out group obstacles, then re-enable the assertion. ~30-40 lines.

---

### P4 — QPSC Solver Convergence (1:1e6 weight ratio)

**File:** `tests/projection_solver/fixture_tests.rs:169`

**Test:** `neighbors_vars1000_constraintsmax10_neighborsmax10_neighborweightmax100_varweights_1_to_1e6_at_10_percent`

**What's broken:** The projection solver fails to converge when variable weights
span a 1:1,000,000 ratio. The solver loop terminates before reaching the correct
solution.

**C# reference:** `ProjectionSolver/Solver.cs` — check if C# handles extreme weight
ratios with additional scaling, normalization, or increased iteration count.

**Fix:** Diagnose by un-ignoring the test and reading the failure. Likely a
convergence threshold or iteration count adjustment. ~20-50 lines.

---

## Priority Order

| Priority | Item | Complexity | Blocks |
|----------|------|------------|--------|
| P2a + P2b | Wire router flags | ~35 lines | nothing |
| P3 | Non-group obstacle assertion | ~35 lines | nothing |
| P4 | QPSC convergence | ~30-50 lines | nothing |
| P1a | Padded polyline fix | ~10 lines | P1d |
| P1b | Polygon port entrances | ~100 lines | P1d |
| P1c | Wire convex hull | ~125 lines | P1d |
| P1d | Un-ignore 3 tests | 0 lines | P1a+b+c done |

**Total estimated new code:** ~350-400 lines.

---

## Acceptance Criteria

`bash tools/compliance-check.sh` exits 0:
- Zero `// TODO` in `src/routing/`
- Zero `#[ignore]` in `tests/`
- Zero `todo!()` or `unimplemented!()` in `src/`
- 243/243 C# tests ported and passing
- `cargo test` 0 failures, 0 ignored
- `cargo clippy -- -D warnings` clean
- `node tools/api-diff.mjs --ts-only --no-ToString` 0 missing from src/

---

## What This Document Is NOT

This PRD does not cover:
- SparseVisibilityGraphGenerator (deferred — no test coverage)
- Non-rectilinear edge routing (out of scope for this crate)
- Performance tuning beyond current benchmarks
