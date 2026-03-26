# Performance Parity Plan: Rust vs C# MSAGL

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Close the remaining performance gap between msagl-rust (120ms) and C# MSAGL (78ms) on the large benchmark (210 obstacles, 500 edges) by replacing agent shortcuts with faithful ports.

**Architecture:** The gap is caused by 4 files where agents substituted brute-force or simplified algorithms for the real C#/TS implementations. Each task ports one file faithfully. The FreeSpaceFinder is the root cause — it under-constrains axis edges, causing path explosion that overwhelms the solver.

**Tech Stack:** Rust, BTreeMap (for sweep-line active sets), ordered-float, existing arena patterns.

**Current profiling (large benchmark):**
| Stage | Time | Root cause |
|-------|------|-----------|
| Solver | 64ms (57%) | Too many variables from path explosion |
| Port splicing | 19.5ms (17%) | Linear scans instead of VG chain walks |
| Combinatorial nudger | 11ms (10%) | Fixed, but insertion sort overhead |
| Path refiner | 7.6ms (7%) | Over-subdivides due to weak FreeSpaceFinder constraints |

**Acceptance criteria:** `cargo bench` large scenario completes in <90ms (competitive with C# 78ms). All 750+ tests pass.

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `src/routing/nudging/free_space_finder.rs` | REWRITE | Sweep-line axis edge bounding |
| `src/routing/nudging/free_space_finder_sweep.rs` | CREATE | Sweep-line implementation (split for file size) |
| `src/routing/nudging/nudger.rs` | MODIFY | Add `MapAxisEdgesToTheirObstacles`, `BoundAxisEdgesByRectsKnownInAdvance`, `CalculateIdealPositions` |
| `src/routing/port_manager.rs` | MODIFY | Replace remaining linear scans with VG chain walks |
| `src/projection_solver/solver/solver_impl.rs` | MODIFY | Remove Vec clones, use index-based iteration |
| `src/projection_solver/solver/dfdv.rs` | MODIFY | Remove Vec clones, add DfDvNode pool |
| `src/projection_solver/solver/qpsc_solve.rs` | MODIFY | Remove Vec clones |

---

### Task 1: FreeSpaceFinder — Faithful Sweep-Line Port

**This is the highest-impact task.** The brute-force FreeSpaceFinder under-constrains axis edges, causing the path refiner to over-subdivide, which inflates solver variable counts from ~200 to ~16,000.

**Files:**
- Rewrite: `src/routing/nudging/free_space_finder.rs`
- Create: `src/routing/nudging/free_space_finder_sweep.rs`
- Reference: `src/routing/nudging/free_space_finder_types.rs` (types already exist)
- Modify: `src/routing/nudging/mod.rs` (register new module)

**C# reference files (READ EVERY LINE):**
- `/Users/ceres/Desktop/SignalCanvas/MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/Nudging/FreeSpaceFinder.cs` (527 lines)
- `/Users/ceres/Desktop/SignalCanvas/MSAGL-Reference/GraphLayout/MSAGL/Routing/Visibility/LineSweeperBase.cs` (172 lines)

**TS reference:**
- `/Users/ceres/Desktop/SignalCanvas/SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/rectilinear/nudging/FreeSpaceFinder.ts` (498 lines)

**CRITICAL PERFORMANCE REQUIREMENT:** The new sweep-line MUST be faster than the current brute-force on the large benchmark. The previous agent's attempt was 10x slower (2.5s vs 234ms) due to a bug. Verify with `cargo bench` before committing. If it's slower, find the bug — don't ship it.

**Previous failed attempt available for reference (DO NOT COPY — it has a perf bug):**
- `.claude/worktrees/agent-aa6ee5e3/src/routing/nudging/free_space_finder_sweep.rs` (618 lines)

- [ ] **Step 1: Read C# FreeSpaceFinder.cs (all 527 lines) and LineSweeperBase.cs (172 lines)**

Understand the sweep-line structure:
- `LineSweeperBase` provides: event queue, `InsertEvent`, `GetAllPoints`, sweep iteration
- `FreeSpaceFinder` inherits and adds: `LeftObstacleSideTree`/`RightObstacleSideTree` (RBTrees of active obstacle sides), `AxisEdgesContainer` (active axis edges ordered by perpendicular projection)
- Event types: `LeftVertexEvent`, `RightVertexEvent`, `AxisEdgeLowPointEvent`, `AxisEdgeHighPointEvent`
- Key methods: `ProcessLeftVertex`, `ProcessRightVertex`, `ProcessLowEdgeEvent`, `ProcessHighEdgeEvent`, `ConstraintEdgeWithObstaclesAtZ`

- [ ] **Step 2: Read the TS FreeSpaceFinder.ts (498 lines) for cross-reference**

- [ ] **Step 3: Read the failed attempt at `.claude/worktrees/agent-aa6ee5e3/src/routing/nudging/free_space_finder_sweep.rs`**

Identify the performance bug. Common causes:
- O(n) linear scan in an inner loop that should be O(log n) tree lookup
- Rebuilding data structures per event instead of maintaining incrementally
- Excessive cloning or allocation in the hot path

- [ ] **Step 4: Implement `FreeSpaceFinderSweep` in `free_space_finder_sweep.rs`**

Port from C# faithfully. Use `BTreeMap` for the obstacle side trees (approved deviation from C# RBTree). Use the types from `free_space_finder_types.rs`.

Key C# methods to port:
```
FindFreeSpace() → creates events, runs sweep
ProcessLeftVertex(point) → inserts left obstacle sides into active trees
ProcessRightVertex(point) → removes right obstacle sides, constrains edges
ProcessLowEdgeEvent(axisEdge) → inserts edge into AxisEdgesContainer, discovers right neighbors
ProcessHighEdgeEvent(axisEdge) → removes edge from container
ConstraintEdgeWithObstaclesAtZ(z, axisEdge) → constrains edge bounds using nearest active obstacle sides
RestrictEdgeFromLeft/Right(event) → finds nearest obstacle side in tree, updates edge bound
TryToAddRightNeighbor(edge1, edge2) → checks if edges' projections overlap
```

- [ ] **Step 5: Update `free_space_finder.rs` public API**

Keep the existing `find_free_space()` signature but delegate to the sweep-line internally.

- [ ] **Step 6: Register module in `mod.rs`**

- [ ] **Step 7: `cargo check`**

- [ ] **Step 8: `cargo test --no-fail-fast` — 750+ passing, 0 failures**

- [ ] **Step 9: `cargo bench` — large scenario MUST be faster than 234ms (current brute-force)**

If slower, profile to find the bug. Do NOT ship a slower implementation.

- [ ] **Step 10: Commit**

```bash
git add src/routing/nudging/free_space_finder.rs src/routing/nudging/free_space_finder_sweep.rs src/routing/nudging/mod.rs
git commit -m "perf: faithful FreeSpaceFinder sweep-line port from C#"
```

---

### Task 2: Nudger — Add Missing Pipeline Methods

**Files:**
- Modify: `src/routing/nudging/nudger.rs`
- Modify: `src/routing/nudging/longest_nudged_segment.rs` (add `ideal_position` centering)

**C# reference (READ EVERY LINE):**
- `/Users/ceres/Desktop/SignalCanvas/MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/Nudging/Nudger.cs` (911 lines) — focus on:
  - `MapAxisEdgesToTheirObstacles()` (line 89-148)
  - `BoundAxisEdgesByRectsKnownInAdvance()` (line 622-634)
  - `BoundAxisEdgesAdjacentToSourceAndTargetOnEdge()` (line 636+)
  - `CalculateIdealPositionsForLongestSegs` / `SetIdealPositionForSeg` (line ~400+)
  - `MoveLongestSegsIdealPositionsInsideFeasibleIntervals` (line ~450+)

These methods are critical because:
- `MapAxisEdgesToTheirObstacles` tells the FreeSpaceFinder which obstacles to exempt per edge (prevents edges being constrained by their own source/target obstacles)
- `BoundAxisEdgesByRectsKnownInAdvance` pre-bounds first/last edges of each path to their obstacle bounding boxes
- `CalculateIdealPositions` centers segments between their neighbors instead of leaving them at their current position — this gives the solver much better starting positions, reducing iteration count

- [ ] **Step 1: Read C# Nudger.cs lines 89-148 (`MapAxisEdgesToTheirObstacles`)**

- [ ] **Step 2: Port `map_axis_edges_to_their_obstacles()` to Rust**

Returns `HashMap<AxisEdgeId, usize>` mapping axis edge to obstacle index. Walk each path's first/last edges and map them to their source/target obstacles.

- [ ] **Step 3: Port `bound_axis_edges_by_rects_known_in_advance()`**

For each path, bound the first and last axis edges by the source/target obstacle bounding boxes.

- [ ] **Step 4: Pass the mapping to FreeSpaceFinder**

Update `find_free_space()` signature to accept `axis_edges_to_obstacles: &HashMap<AxisEdgeId, usize>`. The FreeSpaceFinder uses this to skip constraining an edge by the obstacle it originates from.

- [ ] **Step 5: Port `calculate_ideal_positions_for_longest_segs()`**

For each segment, compute ideal position as midpoint between left and right neighbors' positions: `(left_neighbor_pos + right_neighbor_pos) / 2.0`. C# ref: `SetIdealPositionForSeg`.

- [ ] **Step 6: Port `move_longest_segs_ideal_positions_inside_feasible_intervals()`**

Clamp each segment's ideal position to be within its left_bound..right_bound.

- [ ] **Step 7: `cargo check`**

- [ ] **Step 8: `cargo test --no-fail-fast` — 0 regressions**

- [ ] **Step 9: `cargo bench` — verify improvement**

- [ ] **Step 10: Commit**

---

### Task 3: Solver — Remove Vec Cloning Overhead

**Files:**
- Modify: `src/projection_solver/solver/solver_impl.rs` (11 clones)
- Modify: `src/projection_solver/solver/dfdv.rs` (4 clones)
- Modify: `src/projection_solver/solver/qpsc_solve.rs` (3 clones)
- Modify: `src/projection_solver/solver/mod.rs` (2 clones)

**C# reference:**
- `/Users/ceres/Desktop/SignalCanvas/MSAGL-Reference/GraphLayout/MSAGL/Core/ProjectionSolver/Solver.cs` (1342 lines)
- `/Users/ceres/Desktop/SignalCanvas/MSAGL-Reference/GraphLayout/MSAGL/Core/ProjectionSolver/Block.cs` (737 lines)

**The problem:** The Rust solver clones `Vec<VarIndex>` and `Vec<ConIndex>` from blocks and variables 20 times to work around the borrow checker. C# iterates directly. For the large benchmark's 3rd nudge pass with 16,000+ variables, this creates significant allocation overhead.

**The fix pattern:** Replace `.clone()` with index-based iteration. Instead of:
```rust
let vars: Vec<VarIndex> = self.blocks[bi.0].variables.clone();
for vi in vars { self.do_something(vi); }
```
Use:
```rust
for i in 0..self.blocks[bi.0].variables.len() {
    let vi = self.blocks[bi.0].variables[i];
    self.do_something(vi);
}
```

This is safe because `do_something` doesn't modify the `variables` Vec. The borrow checker rejects the direct form because `self` is mutably borrowed, but indexing by `i` avoids holding a reference to the Vec.

- [ ] **Step 1: Read all 4 solver files, find every `.clone()` call**

- [ ] **Step 2: For each clone, verify that the iteration body does NOT modify the cloned collection**

If it does modify it, the clone is necessary — leave it. If it doesn't, replace with index-based iteration.

- [ ] **Step 3: Replace clones in `solver_impl.rs` (11 clones)**

- [ ] **Step 4: Replace clones in `dfdv.rs` (4 clones)**

Also add DfDvNode pool: instead of allocating `Vec<DfDvNode>` fresh on every `dfs_compute_dfdv` call, reuse a `Vec` stored on `Solver`. C# uses `DfDvRecycleStack`.

- [ ] **Step 5: Replace clones in `qpsc_solve.rs` (3 clones) and `mod.rs` (2 clones)**

- [ ] **Step 6: `cargo check`**

- [ ] **Step 7: `cargo test --no-fail-fast` — 0 regressions**

- [ ] **Step 8: `cargo bench` — verify improvement (expect ~10-20ms savings)**

- [ ] **Step 9: Commit**

---

### Task 4: Port Splicing — Complete VG Chain Walk

**Files:**
- Modify: `src/routing/port_manager.rs`
- Modify: `src/routing/transient_graph_utility.rs` (if needed)

**C# reference:**
- `/Users/ceres/Desktop/SignalCanvas/MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/TransientGraphUtility.cs` (810 lines) — `FindNearestPerpendicularOrContainingEdge` (line 294-329)
- `/Users/ceres/Desktop/SignalCanvas/MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/StaticGraphUtility.cs` — `FindAdjacentVertex`, `FindAdjacentEdge`

**The problem:** The current `splice_port` still does a Phase 1 O(V) vertex scan building a BTreeMap, then Phase 2 walks from seed vertices. C#'s approach starts from the nearest VG vertex (found via point-to-vertex HashMap in O(1)) and walks the chain directly.

- [ ] **Step 1: Read C# `TransientGraphUtility.FindNearestPerpendicularOrContainingEdge` (lines 294-329)**

The C# approach:
1. Find the VG vertex at or nearest the port location using the graph's point-to-vertex map
2. Walk perpendicular from that vertex using `StaticGraphUtility.FindAdjacentVertex`
3. Find the edge that brackets the port's perpendicular coordinate

- [ ] **Step 2: Read the current Rust `port_manager.rs` — understand what the Phase 1 scan does**

- [ ] **Step 3: Replace Phase 1 O(V) scan with O(1) vertex lookup + O(chain) walk**

Use `graph.find_vertex(point)` to find the starting vertex, then walk the adjacency chain.

- [ ] **Step 4: `cargo check`**

- [ ] **Step 5: `cargo test --no-fail-fast` — 0 regressions**

- [ ] **Step 6: `cargo bench` — verify port splicing drops from 19.5ms (expect <5ms)**

- [ ] **Step 7: Commit**

---

### Task 5: Verify and Benchmark

- [ ] **Step 1: Run full test suite**
```bash
cargo test --no-fail-fast
```
Expected: 750+ passed, 0 failed, 1 ignored

- [ ] **Step 2: Run Rust benchmark**
```bash
cargo bench
```
Expected: large < 90ms

- [ ] **Step 3: Run C# benchmark for comparison**
```bash
cd benches/bench_csharp && dotnet run -c Release
```

- [ ] **Step 4: Update README benchmark table with final numbers**

- [ ] **Step 5: Run `cargo clippy -- -D warnings` — must be clean**

- [ ] **Step 6: Run `cargo fmt --check` — must be clean**

- [ ] **Step 7: Run `cargo doc --no-deps` — 0 warnings**

- [ ] **Step 8: Final commit**
```bash
git add -A
git commit -m "perf: achieve performance parity with C# MSAGL on large benchmarks"
```

---

## Execution Order and Dependencies

```
Task 1 (FreeSpaceFinder) ─┐
                           ├─→ Task 2 (Nudger methods) ─→ Task 5 (Verify)
Task 3 (Solver clones)   ─┘                                    ↑
Task 4 (Port splicing)   ─────────────────────────────────────────┘
```

- Tasks 1, 3, 4 can run in parallel (different files)
- Task 2 depends on Task 1 (FreeSpaceFinder must accept the obstacle mapping)
- Task 5 depends on all others

## Expected Impact

| Task | Expected savings | From → To |
|------|-----------------|-----------|
| Task 1 (FreeSpaceFinder) | ~40ms (fewer solver variables) | 64ms solver → ~25ms |
| Task 2 (Nudger methods) | ~15ms (better ideal positions) | 64ms solver → further reduced |
| Task 3 (Solver clones) | ~10-20ms | allocation overhead eliminated |
| Task 4 (Port splicing) | ~15ms | 19.5ms → ~4ms |
| **Combined** | **~80-90ms savings** | **120ms → ~40-50ms** |

If all tasks succeed, Rust should be **faster** than C# (78ms) on the large benchmark.
