# PRD v3: msagl-rust API Audit & Remaining Work

**Supersedes:** `docs/PRD-v2-complete-port.md`
**Date:** 2026-03-28
**Trigger:** Programmatic three-way API audit (C# + TS vs Rust) via `tools/api-diff.mjs`,
plus full `cargo test` run to establish accurate current pass rates.

---

## What Changed Since PRD v2

PRD v2 was written before the VertexEntry[4] + path search rewrite and before the
stateful API was implemented. It overstated how much was blocked. Current reality:

| PRD v2 Claim | Actual Status |
|---|---|
| GROUPS (43 tests) "blocked entirely" | 43 group tests **passing** |
| WAYPOINTS (10 tests) "blocked entirely" | 10 waypoint tests **passing** |
| STATEFUL_UPDATE (2 tests) "blocked" | 7 stateful tests **passing** |
| VertexEntry[4] "not done" | **Done** (commit 23bd798) |
| Path search "different algorithm" | **Done** — SsstRectilinearPath port |

**Current pass rate: 771 passing, 0 failing, 4 ignored.**

The 4 ignored tests are documented below.

---

## Current Test Inventory

### Passing (771)

| Suite | Count | Coverage |
|---|---|---|
| Unit tests (geometry, visibility, solver) | ~500 | Core data structures |
| `rectilinear_tests.rs` | 26 | Basic routing, center ports |
| `rectilinear_tests_freeports.rs` | 41 | Free ports, OOB, collinear |
| `rectilinear_tests_groups.rs` | 28 | Group/cluster routing |
| `rectilinear_tests_groups2.rs` | 15 | More group variants |
| `rectilinear_tests_batch3.rs` | 31 | Batch 3 + all 10 waypoint tests |
| `rectilinear_tests_convexhull.rs` | 18 | Convex hull / overlap |
| `rectilinear_tests_overlap.rs` | 31 | Overlapping obstacles |
| `rectilinear_tests_nonrect.rs` | 20 | Non-rectangular (2 ignored) |
| `rectilinear_tests_reflection.rs` | 19 | Reflection events |
| `rectilinear_tests_remaining.rs` | 13 | Miscellaneous |
| `rectilinear_stateful_tests.rs` | 7 | Stateful add/remove/update |

### Ignored (4)

| Test | File | Reason |
|---|---|---|
| `route_between_two_nonorthogonally_disconnected_1reflection` | `rectilinear_tests_nonrect.rs` | Known regression: route clips obstacle bbox after solver position readback bugfix |
| `route_from_one_nonorthogonally_disconnected_1reflection` | `rectilinear_tests_nonrect.rs` | Same as above |
| `diag_fan_out_vg_state` | `rectilinear_tests.rs` | Diagnostic/visualization test, not a correctness test |
| `neighbors_vars1000_...varweights_1_to_1e6_at_10_percent` | `projection_solver.rs` | QPSC extreme weight ratio convergence (1 to 1e6) |

---

## Defect Inventory (Programmatic Audit)

The audit was run with: `node tools/api-diff.mjs --ts-only --no-ToString`
This reports only items confirmed in **both** C# and TS that are absent from Rust.

### D1. API Naming Aliases (Low priority — both sides work, names differ)

These items exist in Rust under different names. Adding aliases improves
drop-in compatibility but is not required for correctness.

| C# / TS Name | Rust Name | Notes |
|---|---|---|
| `AddEdgeGeometryToRoute` | `add_edge` | C# name is more descriptive |
| `BendPenaltyAsAPercentageOfDistance` | `bend_penalty_as_percentage` | C# name is more explicit |
| `RemoveAllObstacles` | `clear()` | C# name is domain-specific |
| `RemoveVertexEntries` | `clear_vertex_entries()` | Minor naming difference |
| `AddObstacle` (single + rebuild) | no exact equivalent | C# `AddObstacle` = `add_shape_without_rebuild` + `rebuild`; consider adding a convenience wrapper |

### D2. Missing API (Medium priority — real gaps, some tested in C#)

| Item | File | C# Tested? | Notes |
|---|---|---|---|
| `UpdateObstacles` (plural batch) | `rectilinear_edge_router.rs` | Yes (3 tests) | We have singular `update_obstacle`; C# also has a batch variant taking `IEnumerable<Shape>` |
| `EdgeGeometriesToRoute` (introspection) | `rectilinear_edge_router.rs` | Yes (`Update_FreePort` line 989) | Returns the collection of registered edge geometries; `edge_count()` exists but not the collection itself |
| `ObstaclePort.Port` | `port.rs` | Yes (68 test callsites) | The back-reference from ObstaclePort to its underlying port object |
| `ObstaclePort.PortCurve` | `port.rs` | Yes | The ICurve of the obstacle the port lives on |
| `ObstaclePort.PortLocation` | `port.rs` | Yes | Convenience accessor combining Port + obstacle location |
| `MaxVisibilitySegment` | `port_manager_entrances.rs` | No | The farthest visibility segment reachable from this entrance; used in splice extension |

### D3. Unimplemented but Untested in C# (Lowest priority)

| Item | File | Notes |
|---|---|---|
| `IsInSameClump` | `obstacle.rs` | Needed for clump/convex hull routing; no dedicated C# test |

### D4. Four Ignored Tests (Fix required for zero-ignored target)

| Test | Root Cause | Fix |
|---|---|---|
| 2× nonrect reflection | Solver position readback bugfix caused route to clip obstacle bbox | Investigate VG reflection path entering obstacle; likely needs `ObstaclePortEntrance` entrance extension logic |
| `diag_fan_out_vg_state` | Diagnostic, always ignored | Remove `#[ignore]` or delete test if it has no assertion |
| QPSC 1e6 weight ratio | Projection solver convergence with extreme weight ratios | Solver algorithm fix — see original PRD |

### D5. WASM Stateful Router (High priority — blocks real-time drag routing)

The WASM binding remains a one-shot stateless function. C# and TS both expose
a long-lived `RectilinearEdgeRouter` object. Required:

```rust
#[wasm_bindgen]
pub struct Router { /* wraps RouterSession */ }
impl Router {
    pub fn new(padding: f64, edge_separation: f64) -> Router
    pub fn add_obstacle(x: f64, y: f64, w: f64, h: f64) -> usize  // returns handle
    pub fn move_obstacle(id: usize, x: f64, y: f64, w: f64, h: f64)
    pub fn remove_obstacle(id: usize)
    pub fn add_edge(src: usize, tgt: usize) -> usize
    pub fn remove_edge(id: usize)
    pub fn route()
    pub fn get_paths() -> JsValue
}
```

Keep `route_edges()` one-shot function alongside it.

---

## Priority Order

### Priority 1 — Fix 2 ignored nonrect reflection tests

These are correctness regressions. The route clips an obstacle bbox — that is
a routing failure, not just a quality issue.

- Read the ignored test setup to understand the geometry
- Trace why the path enters the obstacle bbox
- Fix in VG generator or entrance extension; do not suppress the test

### Priority 2 — WASM Stateful Router

Implement `Router` struct in `crates/msagl-wasm/src/lib.rs`.
Keeps `route_edges` unchanged. No new algorithm work required — wraps existing
`RectilinearEdgeRouter` API.

### Priority 3 — D2 Missing API

In order:
1. `AddObstacle` convenience wrapper (trivial)
2. `UpdateObstacles` batch variant (trivial — loop + single rebuild)
3. `EdgeGeometriesToRoute` — expose `&[EdgeGeometry]` slice, not just count
4. `ObstaclePort` fields — `port`, `port_curve`, `port_location` — requires reading
   C# `ObstaclePort.cs` and TS `ObstaclePort.ts` to understand the field semantics,
   then adding to the `ObstaclePort` struct in `routing/port.rs`

### Priority 4 — D1 API Naming Aliases

Add `pub fn add_edge_geometry_to_route` etc. as `#[inline]` wrappers calling
the existing methods. Purely additive, zero-risk.

### Priority 5 — QPSC solver fixture

Fix the 1-to-1e6 extreme weight ratio convergence case in the projection solver.

### Priority 6 — D3 IsInSameClump + MaxVisibilitySegment

Low priority since neither has test coverage. Implement when porting any
remaining C# tests that exercise convex hull or entrance extension.

---

## Acceptance Criteria

1. `cargo test` passes with **zero failures and zero ignored**.
2. `cargo clippy -- -D warnings` is clean.
3. `node tools/api-diff.mjs --ts-only --no-ToString` reports **zero items
   missing entirely from src/** (naming differences are acceptable).
4. WASM `Router` object exposes stateful add/move/remove/route API.
5. `route_edges` one-shot function continues to pass all bench scenarios
   with `is_fallback == false` for all edges.
6. Large benchmark stays under 30ms (currently ~24ms native, ~30ms WASM).

---

## What the api-diff Tool Tells Us Going Forward

Run `node tools/api-diff.mjs --ts-only --no-ToString` after any significant
change to get a current gap report. The tool is at `tools/api-diff.mjs` and
requires no dependencies beyond Node.js.

It will produce false positives for:
- C# properties that map to Rust struct fields (the tool now scans `pub` fields,
  so this should be minimal)
- `Compare`/`ToString` which map to Rust traits — suppress with `--no-ToString`
- Items that exist in Rust under different names — these show as "exists elsewhere
  in src/" and need manual review
