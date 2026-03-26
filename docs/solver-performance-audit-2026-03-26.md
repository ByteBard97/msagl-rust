# Projection Solver Performance Audit — 2026-03-26

## Problem

Rust solver takes 585ms on 26K constraints. C# solver handles the same workload in ~10ms. **60x gap.**

## Two Dominant Root Causes

### Root Cause #1: `get_connected_variables` allocates O(n) visited vec every call

**File:** `src/projection_solver/solver/dfdv.rs:63-104`
**C# ref:** `Block.cs:615-686` (`RecurseGetConnectedVariables`)

**Rust:** Allocates `vec![false; self.variables.len()]` on EVERY call to `get_connected_variables`. Called from `split_on_constraint` and `expand` — the innermost hot-path operations.

**C#:** Uses `VariableDoneEval` field on each Variable for backtracking prevention. Zero allocation. The DfDv tree structure prevents revisiting nodes without a visited array.

**Impact:** With 24K variables and ~5K split+expand calls per solve: 5,000 × 24,000 bytes = ~120MB of allocation + zeroing + deallocation in the inner loop.

**Fix:** Port the C# approach — use a generation counter on the Solver and a `last_visited_generation` field on each Variable. Increment generation at the start of each `get_connected_variables` call. Mark variables as visited by setting their generation. Check `var.last_visited_generation == self.generation` instead of `visited[vi]`. Zero allocation.

### Root Cause #2: Bounds create 2x extra variables + constraints (no deduplication)

**File:** `src/projection_solver/uniform_solver.rs:35-43`
**C# ref:** `UniformOneDimensionalSolver.cs:36-70` (`SetLowBound` / `SetUpperBound`)

**Rust:** Every `set_lower_bound(var_id, bound)` creates a NEW anchor variable (weight=1e8) and a NEW constraint (anchor <= var). Every `set_upper_bound` does the same. No deduplication.

**C#:** Stores bounds as fields on the variable (`v.LowBound`, `v.UpperBound`). In `Solve()`, calls `CreateVariablesForBounds()` which uses a `Dictionary<double, int>` to deduplicate — multiple variables with the same bound value SHARE one fixed anchor variable.

**Impact:** With N segments each having 2 bounds, Rust creates 2N extra variables + 2N extra constraints. C# creates ~100 shared anchors (many segments share obstacle boundaries). This triples the variable count, making every O(n) operation 3x slower and every O(n²) operation 9x slower.

**Fix:** Port the C# approach — store bounds as fields on the variable. In `solve()`, create a `HashMap<OrderedFloat<f64>, VarId>` to deduplicate anchor variables by bound value. Only create one anchor per unique bound value.

## Other Contributing Factors (smaller impact)

| # | Issue | File | Multiplier |
|---|-------|------|-----------|
| 3 | `retain()` vs swap-remove in split | `solver_impl.rs:400` | 1.2-1.5x |
| 4 | `connected_vars.clone()` in split | `solver_impl.rs:400` | 1.1-1.3x |
| 5 | DfDvNode pool not recycled mid-traversal | `dfdv.rs` | 1.5-2x |
| 6 | Double indirection in constraint iteration | `solver_impl.rs:230` | 1.3-1.5x |
| 7 | Closure indirection in violation cache | `solver_impl.rs:156` | 1.05-1.1x |

## Fix Priority

1. **Root Cause #1** (visited array) — eliminates ~120MB allocation from inner loop. Expected 5-10x improvement.
2. **Root Cause #2** (bound deduplication) — reduces variable count by ~2/3. Expected 3-9x improvement.
3. Combined effect: 15-60x improvement, closing the gap with C#.

## Skeleton Files Needed

Both fixes need skeletons with the exact C# method signatures and data structures pre-defined (per CLAUDE.md rule 9) to prevent agents from taking shortcuts.

### Skeleton for Root Cause #1:
- Add `visited_generation: u64` field to `Solver`
- Add `last_visited_generation: u64` field to `Variable`
- Replace `vec![false; n]` with generation check in `get_connected_variables`
- Remove the `visited` parameter entirely

### Skeleton for Root Cause #2:
- Add `low_bound: f64` and `high_bound: f64` fields to `Variable` (default: -INF, +INF)
- Change `set_lower_bound`/`set_upper_bound` to store on variable, not create anchor
- Add `create_variables_for_bounds()` method to `Solver` that deduplicates via `HashMap<OrderedFloat<f64>, VarId>`
- Call `create_variables_for_bounds()` at the start of `solve()`
