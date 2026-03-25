# Solver Failure Triage

## Investigation Date: 2026-03-25

## Fixture Test Framework

Each fixture file encodes a projection solver problem from the C# test suite. The
Rust test runner parses it, feeds variables/constraints/neighbors into `SolverShell`,
solves, and compares every variable's resolved position against the C# golden result.

Tolerances are generous:
- Position tolerance: `max(0.5, range × 1e-3)` — for a 1M-range fixture this is ~2000 units.
- Goal tolerance: 1% relative.

Tests only fail when positions are substantially wrong — these are not precision nits.

---

## Failure Classifications

| # | Fixture | Root Cause | Severity | Worst Variable Error | Tolerance | Blocks Layers 0-6? |
|---|---------|-----------|----------|---------------------|-----------|---------------------|
| 1 | Cycles_Vars100_EqualityConstraints | Cycle handling divergence | High | 1,025,509 | 2,047 | No |
| 2 | Cycles_Vars100_NoEquality | Cycle handling divergence | High | 793,601 | 2,112 | No |
| 3 | Cycles_Vars500_EqualityConstraints | Cycle handling divergence | High | 2,303,694 | 5,186 | No |
| 4 | Cycles_Vars500_NoEquality | Cycle handling divergence | High | 1,648,923 | 3,525 | No |
| 5 | Neighbors_Vars1000_VarWeights_1_to_1e6 | QPSC convergence: extreme weight ratio | Medium | 23.95 | 0.5 | No |
| 6 | Neighbors_Vars400 | QPSC convergence: edge case | Medium | 23.10 | 0.5 | No |

---

## Root Cause Analysis

### Category A: Cycle Handling Divergence (Tests 1–4)

**What the tests contain:**
All four fixtures are named "Cycles_*" and carry the header field
`UnsatisfiableConstraints` with values 42–201 (out of 544–2800 total constraints).
These are constraint graphs that contain directed cycles — sets of constraints like
`a ≤ b, b ≤ c, c ≤ a` that are inherently contradictory.

**How C# handles it:**
In `Block.Expand()`, when both variables of a violated constraint are already in the
same block, the solver:
1. Runs `ComputeDfDv` from the violated constraint's left variable, tracking a path to
   the right variable through the active constraint spanning tree.
2. Finds the forward non-equality constraint on that path with the minimum Lagrangian.
3. If found: deactivates it and adjusts offsets of connected variables by the violation.
4. If not found (no forward non-equality path exists): marks the violated constraint as
   `IsUnsatisfiable` and returns without altering variable positions.

The critical invariant: when a constraint is declared unsatisfiable, variable positions
are **left unchanged**. Only the block's reference position is updated at the end if
a valid split was made.

**What the Rust code does differently:**
The Rust `expand()` function in `solver/solver_impl.rs` follows the same overall
structure, but there is a structural divergence from the C# in how the path is computed.

In C#, `ComputeDfDv` is called on the entire block starting from `violatedConstraint.Left`,
and populates `constraintPath` as a side-effect when the traversal reaches
`pathTargetVariable` (= `violatedConstraint.Right`). This traversal uses a proper stack
that tracks the parent chain so the path can be reconstructed accurately.

In Rust, `compute_dfdv_with_path` uses a breadth-first tree expansion (`build_constraint_tree`)
followed by a reverse post-order pass (`process_post_order_with_path`). The path
reconstruction finds the index of the first entry that matches `target_var` and walks
`parent_idx` back to root. However, because the C# uses a depth-first stack traversal
while the Rust uses BFS expansion + reverse iteration, the **traversal order diverges**
when there are multiple paths through the active constraint spanning tree. This is
iteration-order sensitivity in the tree traversal.

The consequence is that in cyclic constraint graphs with many unsatisfiable constraints,
the Rust solver labels different constraints as unsatisfiable than C# does. Once a
constraint is wrongly declared unsatisfiable (or the right one is missed), the block
structure diverges. With 64+ unsatisfiable constraints in a 100-variable system, these
small per-cycle errors compound dramatically — the final position errors are 500x to
1000x the tolerance.

**Specific evidence:**
- Test 1: Variable 61 expected 66409.8 but got 1,091,919 — error 1,025,509 vs
  tolerance 2,047. The actual position is ~16x the expected position, indicating the
  variable ended up in a wrong block with a grossly incorrect reference position.
- Test 3: Variable 60 expected 1,665,783 but got −637,910 — a sign flip, which strongly
  suggests a block merge happened in the wrong direction due to a wrong unsatisfiable
  classification.

**Classification:** Algorithmic divergence — BFS vs DFS traversal order in
`compute_dfdv_with_path`. This is a faithful-port defect: the Rust uses BFS where C#
uses a true DFS stack with parent-chain path reconstruction.

---

### Category B: QPSC Convergence with Extreme Weight Ratios (Tests 5–6)

**What the tests contain:**
- Test 5: 1000 variables, weights spanning 1 to 1e6 (6 orders of magnitude) at 10%
  density. No unsatisfiable constraints.
- Test 6: 400 variables, weights up to 100. No unsatisfiable constraints. The comment
  in `fixture_tests.rs` notes "Other Vars400 fixtures pass" — so it is this specific
  random seed/configuration that fails.

**Error profile:**
Both failures have errors of ~23 units on position ranges of 108–144 units, giving a
relative error of ~21%. The absolute tolerance is 0.5 (no range-scaling kicks in
because the range is so small).

**Root cause:**
Pure QPSC gradient-projection convergence. The Rust QPSC implementation in `qpsc.rs`
is structurally faithful to the paper, but when variable weights span many orders of
magnitude the gradient step size `alpha = g'g / g'Qg` becomes poorly conditioned.
The diagonal scaling (`ScaleInQpsc = true`) is intended to mitigate this, but
the Rust scaling implementation may diverge from C# in the scaling transform applied
to the b-vector and Q matrix. Specifically:

- In `variables_complete()`, the Rust sets `b[ordinal] *= var.scale` (scales b in-place).
- In `apply_scaling()`, the Rust transforms Q to S·Q·S form and sets diagonal to 1.0.
- The C# `QPSC.cs` applies this transform in a single unified pass.

For normal weight ranges (1–100) the approximation is close enough; for 6-order-of-
magnitude ranges (1–1e6) the accumulated floating-point differences cause the
gradient projection to converge to a different local minimum than C#.

Test 6 (Neighbors_Vars400) fails despite modest weights (max 100). This is likely
iteration-order sensitivity: the specific combination of random seed, constraint
topology, and neighbor pairs creates a scenario where the Rust and C# QPSC paths
diverge by a few gradient steps — enough to land at different feasible points.

**Classification:** Numerical convergence difference — not a missing feature, not a
crash. The algorithm terminates with a feasible solution, just not the same one C#
finds. Likely fixable by matching C#'s exact QPSC iteration order and scaling pass.

---

## Summary Table

| Category | Tests | Error Magnitude | Feasible? | Fix Approach |
|----------|-------|----------------|-----------|--------------|
| Cycle handling (DFS vs BFS) | 1–4 | 500–1000× tolerance | No (wrong positions) | Rewrite `compute_dfdv_with_path` to use DFS stack matching C# |
| QPSC convergence (weight ratio) | 5–6 | ~46× tolerance (abs 0.5) | Yes (feasible but different) | Match C# QPSC scaling pass ordering |

---

## Does Anything Here Block Layers 0–6?

**No.** The projection solver is used by the nudging pipeline (Layers 4–5), not by the
core visibility graph or path search (Layers 1–3). All 80 passing fixture tests cover
the ordinary cases — variables without constraint cycles, and neighbors without extreme
weight ratios. Layers 0–6 development can continue without fixing these 6 tests.

The cycle-handling failures (Tests 1–4) are correctness defects that will matter if the
router generates constraint cycles (which it can in degenerate graph configurations).
The convergence failures (Tests 5–6) are accuracy defects that only surface under
extreme weight ratios unlikely in normal router use.

**Recommended fix priority:** After Layers 0–6 are complete. Fix the DFS/BFS divergence
in `compute_dfdv_with_path` first (Tests 1–4), then tune QPSC scaling (Tests 5–6).
