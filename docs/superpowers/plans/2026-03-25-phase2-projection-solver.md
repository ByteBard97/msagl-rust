# Phase 2: Projection Solver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port the MSAGL ProjectionSolver (QPSC constrained optimization) to Rust, validated against 86 C# fixture files.

**Architecture:** Index-based arena pattern — the Solver struct owns all Variables, Constraints, and Blocks in `Vec`s. All cross-references use `usize` indices instead of object references. This eliminates the reference cycles present in the C#/TS implementations and satisfies Rust's ownership model without `Rc`/`RefCell`.

**Tech Stack:** Rust (2021 edition). No additional dependencies beyond Phase 1.

**Spec:** `msagl-rust/docs/superpowers/specs/2026-03-25-msagl-rust-port-design.md`

**Porting sources:**
- TypeScript: `SignalCanvasFrontend/reference/msagl-js/modules/core/src/math/projectionSolver/`
- C#: `MSAGL-Reference/GraphLayout/MSAGL/Core/ProjectionSolver/`
- Test fixtures: `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Resources/Constraints/ProjectionSolver/Data/`

---

## Rust Ownership Design

In C#/TS, Variables hold references to their Block and Constraints, Constraints reference Variables, and Blocks reference Variables. These form reference cycles handled by GC.

In Rust, the **Solver** owns three flat `Vec`s:
- `variables: Vec<Variable>` — indexed by `VarIndex` (newtype over `usize`)
- `constraints: Vec<Constraint>` — indexed by `ConIndex`
- `blocks: Vec<Block>` — indexed by `BlockIndex`

All cross-references are indices:
- `Variable.block: BlockIndex`
- `Variable.left_constraints: Vec<ConIndex>`, `right_constraints: Vec<ConIndex>`
- `Constraint.left: VarIndex`, `Constraint.right: VarIndex`
- `Block.variables: Vec<VarIndex>`

This is the standard ECS-style arena pattern. The Solver passes `&mut self` to methods that need to modify multiple entities.

---

## File Structure

```
msagl-rust/src/projection_solver/
├── mod.rs                  # Module declarations + public re-exports
├── variable.rs             # Variable struct + VarIndex newtype
├── constraint.rs           # Constraint struct + ConIndex newtype
├── parameters.rs           # Parameters, AdvancedParameters
├── solution.rs             # Solution result + SolverAlgorithm enum
├── constraint_vector.rs    # Active/inactive partitioned constraint array
├── violation_cache.rs      # Top-20 most-violated constraint cache
├── block.rs                # Block struct + DfDvNode + BlockIndex
├── solver.rs               # Main solver driver (Project, Split, Merge, Solve)
├── qpsc.rs                 # Gradient projection (sparse matrix, PreProject/PostProject)
├── solver_shell.rs         # High-level API wrapper
└── uniform_solver.rs       # UniformOneDimensionalSolver + UniformSolverVar

msagl-rust/tests/
├── projection_solver/
│   ├── mod.rs
│   ├── basic_tests.rs      # Simple hand-crafted solver tests
│   ├── fixture_parser.rs   # Parse C# fixture files
│   └── fixture_tests.rs    # Run all 86 fixtures
└── fixtures/
    └── projection_solver/  # C# text fixture files (copied from MSAGL test data)
```

**Consolidation from C# (15 files → 12 Rust files):**
- `DfDvNode` → absorbed into `block.rs` (only used there, 70 lines)
- `BlockVector` → absorbed into `solver.rs` (just a `Vec` with swap-remove, 53 lines)
- `UniformSolverVar` → absorbed into `uniform_solver.rs` (46 lines)
- `SolverAlgorithm` → absorbed into `solution.rs` (32 lines)

---

### Task 1: Module Scaffolding

**Files:**
- Modify: `msagl-rust/src/lib.rs`
- Create: `msagl-rust/src/projection_solver/mod.rs`
- Create: all 11 source files as empty placeholders
- Create: `msagl-rust/tests/projection_solver/mod.rs` + test files

- [ ] **Step 1: Create module directory and mod.rs**

```rust
// src/projection_solver/mod.rs
pub mod variable;
pub mod constraint;
pub mod parameters;
pub mod solution;
pub mod constraint_vector;
pub mod violation_cache;
pub mod block;
pub mod solver;
pub mod qpsc;
pub mod solver_shell;
pub mod uniform_solver;
```

- [ ] **Step 2: Create index newtypes in variable.rs and constraint.rs**

```rust
// src/projection_solver/variable.rs
/// Index into Solver's variable Vec.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VarIndex(pub usize);
```

```rust
// src/projection_solver/constraint.rs
/// Index into Solver's constraint Vec.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ConIndex(pub usize);
```

```rust
// src/projection_solver/block.rs
/// Index into Solver's block Vec.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BlockIndex(pub usize);
```

- [ ] **Step 3: Create remaining empty placeholder files**

Each with a minimal struct or placeholder to allow compilation.

- [ ] **Step 4: Add module to lib.rs**

```rust
pub mod projection_solver;
```

- [ ] **Step 5: Create test module structure**

```rust
// tests/projection_solver.rs
#[path = "projection_solver/basic_tests.rs"]
mod basic_tests;
#[path = "projection_solver/fixture_parser.rs"]
mod fixture_parser;
#[path = "projection_solver/fixture_tests.rs"]
mod fixture_tests;
```

Each test file imports from `msagl_rust::projection_solver::*`.

- [ ] **Step 6: Verify compilation**

Run: `cd msagl-rust && cargo check`
Expected: Compiles (possibly with dead_code warnings).

- [ ] **Step 7: Commit**

```bash
git add src/projection_solver/ src/lib.rs tests/projection_solver* tests/fixtures/
git commit -m "feat(msagl-rust): scaffold projection_solver module structure"
```

---

### Task 2: Data Types (Variable, Constraint, Parameters, Solution)

**Files:**
- Modify: `src/projection_solver/variable.rs`
- Modify: `src/projection_solver/constraint.rs`
- Modify: `src/projection_solver/parameters.rs`
- Modify: `src/projection_solver/solution.rs`
- Test: `tests/projection_solver/basic_tests.rs`

**Porting from:**
- TS: `Variable.ts` (167 lines), `Constraint.ts` (132 lines), `Parameters.ts` (142 lines), `Solution.ts` (75 lines), `SolverAlgorithm.ts` (22 lines)

- [ ] **Step 1: Write tests for Variable**

```rust
// tests/projection_solver/basic_tests.rs
use msagl_rust::projection_solver::variable::{Variable, VarIndex};
use msagl_rust::projection_solver::constraint::ConIndex;
use msagl_rust::projection_solver::block::BlockIndex;

#[test]
fn variable_default_fields() {
    let v = Variable::new(VarIndex(0), 5.0, 1.0, 1.0);
    assert_eq!(v.desired_pos, 5.0);
    assert_eq!(v.weight, 1.0);
    assert_eq!(v.scale, 1.0);
    assert_eq!(v.actual_pos, 5.0); // initialized to desired_pos
    assert_eq!(v.offset_in_block, 0.0);
    assert!(v.left_constraints.is_empty());
    assert!(v.right_constraints.is_empty());
}

#[test]
fn variable_dfdv_at_desired_pos() {
    let v = Variable::new(VarIndex(0), 5.0, 1.0, 1.0);
    // DfDv = 2 * weight * (actual - desired) / scale = 0
    assert_eq!(v.dfdv(), 0.0);
}

#[test]
fn variable_dfdv_displaced() {
    let mut v = Variable::new(VarIndex(0), 5.0, 2.0, 1.0);
    v.actual_pos = 7.0;
    // DfDv = 2 * 2.0 * (7.0 - 5.0) / 1.0 = 8.0
    assert_eq!(v.dfdv(), 8.0);
}
```

- [ ] **Step 2: Write tests for Constraint**

```rust
#[test]
fn constraint_violation_satisfied() {
    // left at 0, right at 5, gap 3 → violation = 0*1 + 3 - 5*1 = -2 (satisfied)
    let c = Constraint::new(ConIndex(0), VarIndex(0), VarIndex(1), 3.0, false);
    // Violation computed externally since we need variable positions
    // This tests the struct creation
    assert_eq!(c.gap, 3.0);
    assert!(!c.is_equality);
    assert!(!c.is_active);
    assert_eq!(c.lagrangian, 0.0);
}

#[test]
fn constraint_violation_calculation() {
    // violation = left.actual * left.scale + gap - right.actual * right.scale
    let left_actual = 2.0;
    let left_scale = 1.0;
    let right_actual = 4.0;
    let right_scale = 1.0;
    let gap = 3.0;
    let violation = left_actual * left_scale + gap - right_actual * right_scale;
    // 2 + 3 - 4 = 1.0 (violated!)
    assert_eq!(violation, 1.0);
}
```

- [ ] **Step 3: Write tests for Parameters**

```rust
use msagl_rust::projection_solver::parameters::Parameters;

#[test]
fn default_parameters() {
    let p = Parameters::default();
    assert_eq!(p.gap_tolerance, 1e-4);
    assert_eq!(p.qpsc_convergence_epsilon, 1e-5);
    assert_eq!(p.qpsc_convergence_quotient, 1e-6);
}
```

- [ ] **Step 4: Implement Variable**

```rust
// src/projection_solver/variable.rs
use super::constraint::ConIndex;
use super::block::BlockIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VarIndex(pub usize);

/// Neighbor pair for QPSC goal function: w_ij * (x_i - x_j)^2
#[derive(Clone, Debug)]
pub struct NeighborAndWeight {
    pub neighbor: VarIndex,
    pub weight: f64,
}

#[derive(Clone, Debug)]
pub struct Variable {
    pub index: VarIndex,
    pub desired_pos: f64,
    pub actual_pos: f64,
    pub weight: f64,
    pub scale: f64,
    pub offset_in_block: f64,
    pub block: BlockIndex,
    pub active_constraint_count: u32,
    pub left_constraints: Vec<ConIndex>,
    pub right_constraints: Vec<ConIndex>,
    pub neighbors: Vec<NeighborAndWeight>,
    pub ordinal: u32,
}

impl Variable {
    pub fn new(index: VarIndex, desired_pos: f64, weight: f64, scale: f64) -> Self {
        Self {
            index,
            desired_pos,
            actual_pos: desired_pos,
            weight,
            scale,
            offset_in_block: 0.0,
            block: BlockIndex(0),
            active_constraint_count: 0,
            left_constraints: Vec::new(),
            right_constraints: Vec::new(),
            neighbors: Vec::new(),
            ordinal: 0,
        }
    }

    /// Derivative of goal function: 2 * weight * (actual - desired) / scale
    #[inline]
    pub fn dfdv(&self) -> f64 {
        (2.0 * self.weight * (self.actual_pos - self.desired_pos)) / self.scale
    }
}
```

- [ ] **Step 5: Implement Constraint**

```rust
// src/projection_solver/constraint.rs
use super::variable::VarIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ConIndex(pub usize);

#[derive(Clone, Debug)]
pub struct Constraint {
    pub index: ConIndex,
    pub left: VarIndex,
    pub right: VarIndex,
    pub gap: f64,
    pub is_equality: bool,
    pub is_active: bool,
    pub is_unsatisfiable: bool,
    pub lagrangian: f64,
    pub vector_index: usize,
}

impl Constraint {
    pub fn new(index: ConIndex, left: VarIndex, right: VarIndex, gap: f64, is_equality: bool) -> Self {
        Self {
            index,
            left,
            right,
            gap,
            is_equality,
            is_active: false,
            is_unsatisfiable: false,
            lagrangian: 0.0,
            vector_index: 0,
        }
    }

    /// Compute violation using variable positions.
    /// Positive = violated, zero/negative = satisfied.
    #[inline]
    pub fn violation(&self, left_actual: f64, left_scale: f64, right_actual: f64, right_scale: f64) -> f64 {
        left_actual * left_scale + self.gap - right_actual * right_scale
    }
}
```

- [ ] **Step 6: Implement Parameters**

```rust
// src/projection_solver/parameters.rs

#[derive(Clone, Debug)]
pub struct Parameters {
    pub gap_tolerance: f64,
    pub qpsc_convergence_epsilon: f64,
    pub qpsc_convergence_quotient: f64,
    pub outer_project_iterations_limit: i32,
    pub inner_project_iterations_limit: i32,
    pub time_limit_ms: i32,
    pub advanced: AdvancedParameters,
}

#[derive(Clone, Debug)]
pub struct AdvancedParameters {
    pub force_qpsc: bool,
    pub scale_in_qpsc: bool,
    pub min_split_lagrangian_threshold: f64,
    pub use_violation_cache: bool,
    pub violation_cache_min_blocks_divisor: usize,
    pub violation_cache_min_blocks_count: usize,
}

impl Default for Parameters {
    fn default() -> Self {
        Self {
            gap_tolerance: 1e-4,
            qpsc_convergence_epsilon: 1e-5,
            qpsc_convergence_quotient: 1e-6,
            outer_project_iterations_limit: -1, // auto-computed
            inner_project_iterations_limit: -1, // auto-computed
            time_limit_ms: -1, // no limit
            advanced: AdvancedParameters::default(),
        }
    }
}

impl Default for AdvancedParameters {
    fn default() -> Self {
        Self {
            force_qpsc: false,
            scale_in_qpsc: true,
            min_split_lagrangian_threshold: -1e-7,
            use_violation_cache: true,
            violation_cache_min_blocks_divisor: 10,
            violation_cache_min_blocks_count: 100,
        }
    }
}
```

- [ ] **Step 7: Implement Solution**

```rust
// src/projection_solver/solution.rs

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolverAlgorithm {
    ProjectOnly,
    QpscWithScaling,
    QpscWithoutScaling,
}

#[derive(Clone, Debug)]
pub struct Solution {
    pub number_of_unsatisfiable_constraints: u32,
    pub outer_project_iterations: u32,
    pub inner_project_iterations_total: u64,
    pub min_inner_project_iterations: u32,
    pub max_inner_project_iterations: u32,
    pub max_constraint_tree_depth: u32,
    pub goal_function_value: f64,
    pub algorithm_used: SolverAlgorithm,
    pub time_limit_exceeded: bool,
    pub outer_project_iterations_limit_exceeded: bool,
    pub inner_project_iterations_limit_exceeded: bool,
}

impl Solution {
    pub fn new() -> Self {
        Self {
            number_of_unsatisfiable_constraints: 0,
            outer_project_iterations: 0,
            inner_project_iterations_total: 0,
            min_inner_project_iterations: u32::MAX,
            max_inner_project_iterations: 0,
            max_constraint_tree_depth: 0,
            goal_function_value: 0.0,
            algorithm_used: SolverAlgorithm::ProjectOnly,
            time_limit_exceeded: false,
            outer_project_iterations_limit_exceeded: false,
            inner_project_iterations_limit_exceeded: false,
        }
    }

    pub fn execution_limit_exceeded(&self) -> bool {
        self.time_limit_exceeded
            || self.outer_project_iterations_limit_exceeded
            || self.inner_project_iterations_limit_exceeded
    }
}

impl Default for Solution {
    fn default() -> Self { Self::new() }
}
```

- [ ] **Step 8: Run tests**

Run: `cd msagl-rust && cargo test --test projection_solver`
Expected: All basic tests pass.

- [ ] **Step 9: Commit**

```bash
git add src/projection_solver/ tests/projection_solver/
git commit -m "feat(msagl-rust): add projection solver data types (Variable, Constraint, Parameters, Solution)"
```

---

### Task 3: ConstraintVector (Partitioned Array)

**Files:**
- Modify: `src/projection_solver/constraint_vector.rs`
- Test: `tests/projection_solver/basic_tests.rs` (append)

**Porting from:** TS: `ConstraintVector.ts` (127 lines)

This is the O(1) active/inactive constraint partitioning array. Active constraints occupy the front of the array; inactive at the back. Activate/deactivate swaps a constraint across the partition boundary.

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::projection_solver::constraint_vector::ConstraintVector;
use msagl_rust::projection_solver::constraint::{Constraint, ConIndex};
use msagl_rust::projection_solver::variable::VarIndex;

#[test]
fn constraint_vector_activate_deactivate() {
    let mut cv = ConstraintVector::new(3);
    // Initially all inactive
    assert_eq!(cv.active_count(), 0);

    // Activate constraint 0
    cv.activate(ConIndex(0));
    assert_eq!(cv.active_count(), 1);

    // Activate constraint 1
    cv.activate(ConIndex(1));
    assert_eq!(cv.active_count(), 2);

    // Deactivate constraint 0
    cv.deactivate(ConIndex(0));
    assert_eq!(cv.active_count(), 1);
}
```

- [ ] **Step 2: Implement ConstraintVector**

```rust
// src/projection_solver/constraint_vector.rs
use super::constraint::ConIndex;

/// Partitioned constraint array. Active constraints at the front,
/// inactive at the back. Activate/deactivate is O(1) via swap.
pub struct ConstraintVector {
    /// Maps ConIndex → position in the partitioned array.
    positions: Vec<usize>,
    /// The partitioned array of ConIndex values.
    order: Vec<ConIndex>,
    /// Index of first active constraint. Active = [first_active..len).
    first_active: usize,
    pub max_constraint_tree_depth: u32,
    pub number_of_unsatisfiable_constraints: u32,
}

impl ConstraintVector {
    pub fn new(count: usize) -> Self {
        let order: Vec<ConIndex> = (0..count).map(ConIndex).collect();
        let positions: Vec<usize> = (0..count).collect();
        Self {
            positions,
            order,
            first_active: count, // all inactive initially
            max_constraint_tree_depth: 0,
            number_of_unsatisfiable_constraints: 0,
        }
    }

    pub fn active_count(&self) -> usize {
        self.order.len() - self.first_active
    }

    /// Move a constraint from inactive to active.
    pub fn activate(&mut self, ci: ConIndex) {
        self.first_active -= 1;
        self.swap_to(ci, self.first_active);
    }

    /// Move a constraint from active to inactive.
    pub fn deactivate(&mut self, ci: ConIndex) {
        self.swap_to(ci, self.first_active);
        self.first_active += 1;
    }

    fn swap_to(&mut self, ci: ConIndex, target_pos: usize) {
        let current_pos = self.positions[ci.0];
        if current_pos == target_pos {
            return;
        }
        let other_ci = self.order[target_pos];
        // Swap positions
        self.order[current_pos] = other_ci;
        self.order[target_pos] = ci;
        self.positions[ci.0] = target_pos;
        self.positions[other_ci.0] = current_pos;
    }

    /// Iterate over active constraint indices.
    pub fn active_constraints(&self) -> &[ConIndex] {
        &self.order[self.first_active..]
    }

    /// Iterate over all constraint indices.
    pub fn all_constraints(&self) -> &[ConIndex] {
        &self.order
    }
}
```

- [ ] **Step 3: Run tests, verify pass**

Run: `cd msagl-rust && cargo test --test projection_solver`

- [ ] **Step 4: Commit**

```bash
git add src/projection_solver/constraint_vector.rs tests/projection_solver/
git commit -m "feat(msagl-rust): add ConstraintVector with O(1) active/inactive partitioning"
```

---

### Task 4: ViolationCache

**Files:**
- Modify: `src/projection_solver/violation_cache.rs`
- Test: `tests/projection_solver/basic_tests.rs` (append)

**Porting from:** TS: `ViolationCache.ts` (165 lines), C#: `ViolationCache.cs` (266 lines)

Fixed-size cache of up to 20 most-violated constraints. Used to avoid scanning all constraints every Project iteration.

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::projection_solver::violation_cache::ViolationCache;
use msagl_rust::projection_solver::constraint::ConIndex;

#[test]
fn violation_cache_insert_and_find() {
    let mut cache = ViolationCache::new();
    cache.insert(ConIndex(5), 10.0);
    cache.insert(ConIndex(3), 20.0);
    cache.insert(ConIndex(7), 5.0);

    // Should find the one with highest violation > 15
    let result = cache.find_if_greater(15.0);
    assert_eq!(result, Some(ConIndex(3)));
}

#[test]
fn violation_cache_clear() {
    let mut cache = ViolationCache::new();
    cache.insert(ConIndex(0), 100.0);
    cache.clear();
    assert_eq!(cache.find_if_greater(0.0), None);
}

#[test]
fn violation_cache_capacity_limit() {
    let mut cache = ViolationCache::new();
    // Insert 25 items (capacity is 20)
    for i in 0..25 {
        cache.insert(ConIndex(i), i as f64);
    }
    // Only top 20 should remain; lowest violation >= 5
    assert!(cache.find_if_greater(4.0).is_some());
}
```

- [ ] **Step 2: Implement ViolationCache**

```rust
// src/projection_solver/violation_cache.rs
use super::constraint::ConIndex;
use super::block::BlockIndex;

const MAX_CACHE_SIZE: usize = 20;

struct CacheEntry {
    constraint: ConIndex,
    violation: f64,
}

/// Fixed-size cache of the most-violated constraints.
/// Avoids full constraint scans during Project iterations.
pub struct ViolationCache {
    entries: Vec<CacheEntry>,
    low_violation: f64,
}

impl ViolationCache {
    pub fn new() -> Self {
        Self {
            entries: Vec::with_capacity(MAX_CACHE_SIZE),
            low_violation: f64::MAX,
        }
    }

    pub fn clear(&mut self) {
        self.entries.clear();
        self.low_violation = f64::MAX;
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Insert a constraint with its violation value.
    /// If at capacity, replaces the entry with the lowest violation.
    pub fn insert(&mut self, ci: ConIndex, violation: f64) {
        if self.entries.len() < MAX_CACHE_SIZE {
            self.entries.push(CacheEntry { constraint: ci, violation });
            if violation < self.low_violation {
                self.low_violation = violation;
            }
        } else if violation > self.low_violation {
            // Replace the entry with the lowest violation
            let mut min_idx = 0;
            let mut min_val = self.entries[0].violation;
            for (i, e) in self.entries.iter().enumerate().skip(1) {
                if e.violation < min_val {
                    min_val = e.violation;
                    min_idx = i;
                }
            }
            self.entries[min_idx] = CacheEntry { constraint: ci, violation };
            // Recompute low_violation
            self.low_violation = self.entries.iter().map(|e| e.violation).fold(f64::MAX, f64::min);
        }
    }

    /// Find the constraint with violation > target_violation. Returns the one with max violation.
    pub fn find_if_greater(&self, target_violation: f64) -> Option<ConIndex> {
        let mut best: Option<(ConIndex, f64)> = None;
        for e in &self.entries {
            if e.violation > target_violation {
                if best.is_none() || e.violation > best.unwrap().1 {
                    best = Some((e.constraint, e.violation));
                }
            }
        }
        best.map(|(ci, _)| ci)
    }

    /// Remove all entries that reference the given block's constraints.
    /// `is_in_block` is a closure that checks if a constraint touches the block.
    /// Returns true if any entries remain after filtering.
    pub fn filter_block(&mut self, is_in_block: impl Fn(ConIndex) -> bool) -> bool {
        self.entries.retain(|e| !is_in_block(e.constraint));
        if self.entries.is_empty() {
            self.low_violation = f64::MAX;
            false
        } else {
            self.low_violation = self.entries.iter().map(|e| e.violation).fold(f64::MAX, f64::min);
            true
        }
    }
}
```

- [ ] **Step 3: Run tests, verify pass**

- [ ] **Step 4: Commit**

```bash
git add src/projection_solver/violation_cache.rs tests/projection_solver/
git commit -m "feat(msagl-rust): add ViolationCache for constraint violation optimization"
```

---

### Task 5: Block (Structure + Variable Management)

**Files:**
- Modify: `src/projection_solver/block.rs`
- Test: `tests/projection_solver/basic_tests.rs` (append)

**Porting from:** TS: `Block.ts` (649 lines), C#: `Block.cs` (737 lines)

This task implements the Block data structure and variable management. The complex algorithms (ComputeDfDv, Expand, Split) are in Tasks 6-7 as part of the Solver since they need access to all variables/constraints.

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::projection_solver::block::{Block, BlockIndex};
use msagl_rust::projection_solver::variable::{Variable, VarIndex};

#[test]
fn block_creation() {
    let block = Block::new(BlockIndex(0), VarIndex(0), 5.0, 1.0, 1.0);
    assert_eq!(block.variables.len(), 1);
    assert_eq!(block.reference_pos, 5.0);
    assert_eq!(block.scale, 1.0);
}

#[test]
fn block_add_variable_updates_sums() {
    let mut block = Block::new(BlockIndex(0), VarIndex(0), 5.0, 2.0, 1.0);
    // Add second variable with offset
    block.add_variable_with_offset(VarIndex(1), 3.0, 2.0, 1.0, 2.0);
    assert_eq!(block.variables.len(), 2);
}

#[test]
fn block_update_reference_pos() {
    // Single variable: ref_pos = desired_pos
    let block = Block::new(BlockIndex(0), VarIndex(0), 5.0, 1.0, 1.0);
    assert_eq!(block.reference_pos, 5.0);
}
```

- [ ] **Step 2: Implement Block**

```rust
// src/projection_solver/block.rs
use super::variable::VarIndex;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct BlockIndex(pub usize);

/// A block groups variables that are connected by active constraints.
/// All variables in a block move together, maintaining their relative offsets.
#[derive(Clone, Debug)]
pub struct Block {
    pub index: BlockIndex,
    pub variables: Vec<VarIndex>,
    pub reference_pos: f64,
    pub scale: f64,
    // Pre-computed sums for reference position formula:
    // reference_pos = (sum_ad - sum_ab) / sum_a2
    pub sum_a2: f64, // Σ a_i² * w_i  where a_i = scale / var.scale
    pub sum_ad: f64, // Σ a_i * d_i * w_i
    pub sum_ab: f64, // Σ a_i * b_i * w_i  where b_i = offset / var.scale
    pub vector_index: usize, // position in solver's block list
}

impl Block {
    /// Create a new block with a single variable.
    pub fn new(
        index: BlockIndex,
        var: VarIndex,
        desired_pos: f64,
        weight: f64,
        scale: f64,
    ) -> Self {
        let a = 1.0; // scale / var.scale when both are same
        Self {
            index,
            variables: vec![var],
            reference_pos: desired_pos,
            scale,
            sum_a2: a * a * weight,
            sum_ad: a * desired_pos * weight,
            sum_ab: 0.0, // offset = 0 for first variable
            vector_index: 0,
        }
    }

    /// Add a variable with a known offset from block reference position.
    pub fn add_variable_with_offset(
        &mut self,
        var: VarIndex,
        offset: f64,
        weight: f64,
        var_scale: f64,
        desired_pos: f64,
    ) {
        let a = self.scale / var_scale;
        let b = offset / var_scale;
        self.sum_a2 += a * a * weight;
        self.sum_ad += a * desired_pos * weight;
        self.sum_ab += a * b * weight;
        self.variables.push(var);
    }

    /// Recompute reference position from accumulated sums.
    pub fn update_reference_pos(&mut self) {
        if self.sum_a2 > 0.0 {
            self.reference_pos = (self.sum_ad - self.sum_ab) / self.sum_a2;
        }
    }

    /// Compute actual position for a variable given its offset and scale.
    #[inline]
    pub fn compute_variable_pos(&self, offset: f64, var_scale: f64) -> f64 {
        (self.scale * self.reference_pos + offset) / var_scale
    }

    /// Reset sums to zero (for rebuilding from scratch).
    pub fn reset_sums(&mut self) {
        self.sum_a2 = 0.0;
        self.sum_ad = 0.0;
        self.sum_ab = 0.0;
    }

    /// Add a variable's contribution to the block sums.
    pub fn add_to_sums(&mut self, offset: f64, weight: f64, var_scale: f64, desired_pos: f64) {
        let a = self.scale / var_scale;
        let b = offset / var_scale;
        self.sum_a2 += a * a * weight;
        self.sum_ad += a * desired_pos * weight;
        self.sum_ab += a * b * weight;
    }
}
```

- [ ] **Step 3: Run tests, verify pass**

- [ ] **Step 4: Commit**

```bash
git add src/projection_solver/block.rs tests/projection_solver/
git commit -m "feat(msagl-rust): add Block structure with variable management and reference position"
```

---

### Task 6: Solver Core — Setup, MergeBlocks, Project, SplitBlocks

**Files:**
- Modify: `src/projection_solver/solver.rs`
- Test: `tests/projection_solver/basic_tests.rs` (append)

**Porting from:** TS: `Solver.ts` (1,012 lines), C#: `Solver.cs` (1,342 lines)

This is the largest and most complex task. It implements the main solver loop including:
- Variable/constraint setup
- Block merging when a violated constraint spans two blocks
- ComputeDfDv tree traversal
- Block.Expand (intra-block violation resolution)
- Project loop (find max violation, fix it)
- SplitBlocks (decompose over-constrained blocks)
- The SolveByStandaloneProject main loop

**Important:** This file will be ~400-500 lines. The implementer should port faithfully from the TS source, following the same algorithm structure. Read `Solver.ts` and `Block.ts` carefully.

- [ ] **Step 1: Write basic solver tests**

```rust
use msagl_rust::projection_solver::solver::Solver;
use msagl_rust::projection_solver::parameters::Parameters;

#[test]
fn two_vars_satisfied_constraint() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(10.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 3.0, false);

    let solution = solver.solve(None);
    // Already satisfied: gap 3, distance 10
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!(pos1 - pos0 >= 3.0 - 1e-4);
    assert!((pos0 - 0.0).abs() < 1e-4);
    assert!((pos1 - 10.0).abs() < 1e-4);
}

#[test]
fn two_vars_violated_constraint() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(5.0, 1.0, 1.0);
    let v1 = solver.add_variable(6.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 3.0, false);

    let solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    // Must satisfy: pos1 - pos0 >= 3.0
    assert!(pos1 - pos0 >= 3.0 - 1e-4);
    // Should split the difference: ~4.0 and ~7.0 (equal weight)
    assert!((pos0 - 4.0).abs() < 1e-3);
    assert!((pos1 - 7.0).abs() < 1e-3);
}

#[test]
fn three_vars_chain() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(0.0, 1.0, 1.0);
    let v2 = solver.add_variable(0.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 5.0, false);
    solver.add_constraint(v1, v2, 5.0, false);

    let solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    let pos2 = solver.variable(v2).actual_pos;
    assert!(pos1 - pos0 >= 5.0 - 1e-4);
    assert!(pos2 - pos1 >= 5.0 - 1e-4);
}

#[test]
fn equality_constraint() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(10.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 5.0, true); // equality: v1 - v0 == 5

    let solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!((pos1 - pos0 - 5.0).abs() < 1e-4);
}

#[test]
fn weighted_variables() {
    let mut solver = Solver::new();
    // Heavy variable at 0 should barely move; light variable at 2 moves more
    let v0 = solver.add_variable(0.0, 100.0, 1.0);
    let v1 = solver.add_variable(2.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 5.0, false);

    let solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!(pos1 - pos0 >= 5.0 - 1e-4);
    // Heavy v0 should move very little
    assert!(pos0.abs() < 0.5);
}
```

- [ ] **Step 2: Implement Solver**

The Solver implementation is complex. The implementer should:

1. Read `Solver.ts` lines 1-100 for the public API structure
2. Read `Solver.ts` lines ~200-400 for `SolvePar` / `SolveByStandaloneProject`
3. Read `Solver.ts` lines ~400-600 for `RunProject` / `GetMaxViolatedConstraint`
4. Read `Solver.ts` lines ~600-800 for `MergeBlocks`
5. Read `Block.ts` lines ~100-350 for `ComputeDfDv` / `Expand`
6. Read `Block.ts` lines ~350-500 for `Split` / `TransferConnectedVariables`

Key structural decisions for Rust:

```rust
// src/projection_solver/solver.rs
pub struct Solver {
    variables: Vec<Variable>,
    constraints: Vec<Constraint>,
    blocks: Vec<Block>,
    constraint_vector: ConstraintVector,
    violation_cache: ViolationCache,
    equality_constraints: Vec<ConIndex>,
    is_qpsc: bool,
    // For DfDv traversal
    dfdv_stack: Vec<DfDvNode>,
}

/// DfDvNode for stack-based constraint tree traversal.
/// Replaces recursive calls with an explicit stack.
struct DfDvNode {
    variable_to_eval: VarIndex,
    variable_done_eval: VarIndex,
    constraint_to_eval: Option<ConIndex>,
    is_left_to_right: bool,
    children_have_been_pushed: bool,
    lagrangian_sum: f64,
}
```

The `Solver` methods operate on `&mut self`, accessing variables/constraints/blocks by index. For the ComputeDfDv traversal and Expand/Split algorithms, the implementer must use index-based access rather than references to avoid borrow checker conflicts.

**Critical algorithm: Project loop**
```rust
fn run_project(&mut self, params: &Parameters) -> bool {
    loop {
        let (max_con, max_violation) = self.get_max_violated_constraint(params);
        if max_con.is_none() { return found_any; }
        let ci = max_con.unwrap();

        let left_block = self.variables[self.constraints[ci.0].left.0].block;
        let right_block = self.variables[self.constraints[ci.0].right.0].block;

        if left_block == right_block {
            self.expand(ci, max_violation);
        } else {
            self.merge_blocks(ci);
        }
    }
}
```

- [ ] **Step 3: Run tests, verify pass**

Run: `cd msagl-rust && cargo test --test projection_solver`

- [ ] **Step 4: Commit**

```bash
git add src/projection_solver/solver.rs tests/projection_solver/
git commit -m "feat(msagl-rust): add Solver core with Project/Split/Merge constraint satisfaction"
```

---

### Task 7: QPSC (Gradient Projection)

**Files:**
- Modify: `src/projection_solver/qpsc.rs`
- Modify: `src/projection_solver/solver.rs` (add `solve_qpsc` method)
- Test: `tests/projection_solver/basic_tests.rs` (append)

**Porting from:** TS: `QPSC.ts` (517 lines), C#: `QPSC.cs` (790 lines)

QPSC implements gradient projection for quadratic programming with separation constraints. It's used when variables have neighbor relationships (weighted (x_i - x_j)^2 terms in the goal function).

- [ ] **Step 1: Write tests**

```rust
#[test]
fn solver_with_neighbors() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(10.0, 1.0, 1.0);
    let v2 = solver.add_variable(20.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 3.0, false);
    solver.add_constraint(v1, v2, 3.0, false);
    solver.add_neighbor_pair(v0, v2, 1.0);

    let solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    let pos2 = solver.variable(v2).actual_pos;
    // Constraints must be satisfied
    assert!(pos1 - pos0 >= 3.0 - 1e-4);
    assert!(pos2 - pos1 >= 3.0 - 1e-4);
    // Neighbor force should pull v0 and v2 closer together
}
```

- [ ] **Step 2: Implement QPSC**

```rust
// src/projection_solver/qpsc.rs

/// Sparse matrix cell for the Hessian.
struct MatrixCell {
    var_index: usize,
    value: f64,
}

/// Gradient projection solver for quadratic programming with separation constraints.
pub struct Qpsc {
    matrix_q: Vec<Vec<MatrixCell>>,  // Sparse Hessian rows
    vector_b: Vec<f64>,              // Linear term: -2 * w_i * d_i
    gradient: Vec<f64>,              // g = Qy + b
    vector_qg: Vec<f64>,            // Q * g (for step size)
    prev_y: Vec<f64>,               // Previous positions
    curr_y: Vec<f64>,               // Current positions
    prev_function_value: f64,
    num_vars: usize,
}
```

The implementer should read `QPSC.ts` carefully for:
- `AddVariable()` — builds sparse matrix rows
- `VariablesComplete()` — applies diagonal scaling
- `PreProject()` — gradient computation, convergence test, alpha step
- `PostProject()` — beta step computation
- `QpscComplete()` — restore original weights/scales

- [ ] **Step 3: Wire QPSC into Solver**

Add `solve_qpsc()` method to `Solver` that follows this loop:
1. `qpsc_make_feasible()` — initial Project pass
2. Create Qpsc, add variables, call `variables_complete()`
3. Reinitialize blocks (each variable in own block)
4. Loop: `pre_project()` → `split_blocks()` → `run_project()` → `post_project()`

- [ ] **Step 4: Run tests, verify pass**

- [ ] **Step 5: Commit**

```bash
git add src/projection_solver/qpsc.rs src/projection_solver/solver.rs tests/projection_solver/
git commit -m "feat(msagl-rust): add QPSC gradient projection for neighbor-aware solving"
```

---

### Task 8: SolverShell + UniformOneDimensionalSolver

**Files:**
- Modify: `src/projection_solver/solver_shell.rs`
- Modify: `src/projection_solver/uniform_solver.rs`
- Test: `tests/projection_solver/basic_tests.rs` (append)

**Porting from:** TS: `SolverShell.ts` (300 lines), `UniformOneDimensionalSolver.ts` (161 lines), `UniformSolverVar.ts` (44 lines)

SolverShell is the high-level API that the nudging pipeline uses. It wraps Solver with a simpler interface (variable IDs instead of indices).

UniformOneDimensionalSolver is called from the nudging pipeline's Nudger. It creates a SolverShell, adds variables with upper/lower bounds, and solves.

- [ ] **Step 1: Write tests**

```rust
use msagl_rust::projection_solver::solver_shell::SolverShell;

#[test]
fn solver_shell_basic() {
    let mut shell = SolverShell::new();
    let v0 = shell.add_variable(None, 0.0, 1.0, 1.0);
    let v1 = shell.add_variable(None, 10.0, 1.0, 1.0);
    shell.add_left_right_constraint(v0, v1, 3.0);

    shell.solve();
    let pos0 = shell.get_variable_resolved_position(v0);
    let pos1 = shell.get_variable_resolved_position(v1);
    assert!(pos1 - pos0 >= 3.0 - 1e-4);
}
```

- [ ] **Step 2: Implement SolverShell and UniformOneDimensionalSolver**

`SolverShell` wraps `Solver`:
- Maps user variable IDs → VarIndex
- Provides `add_variable`, `add_left_right_constraint`, `add_goal_two_variables_are_close`
- Calls `solver.solve()` and reads back positions

`UniformOneDimensionalSolver`:
- Creates SolverShell
- Adds variables with optional upper/lower bounds
- Implements bounds via additional variables + constraints
- Returns solved positions

- [ ] **Step 3: Run tests, verify pass**

- [ ] **Step 4: Commit**

```bash
git add src/projection_solver/solver_shell.rs src/projection_solver/uniform_solver.rs tests/projection_solver/
git commit -m "feat(msagl-rust): add SolverShell and UniformOneDimensionalSolver wrappers"
```

---

### Task 9: Fixture Parser + Fixture-Driven Tests

**Files:**
- Create: `tests/projection_solver/fixture_parser.rs`
- Create: `tests/projection_solver/fixture_tests.rs`
- Create: `tests/fixtures/projection_solver/` (converted JSON files)

**Source:** C# fixture files at `MSAGL-Reference/GraphLayout/Test/MSAGLTests/Resources/Constraints/ProjectionSolver/Data/`

The C# fixture format is plain text with sections:
```
BEGIN VARIABLES
[id] [desired_pos] [size] [weight] [optional_scale]
END VARIABLES
```

**Note:** The `size` column (always 0.0 for projection solver data) must be skipped by the parser. The actual columns are: ordinal, desired_pos, size, weight, and optionally scale (defaults to 1.0 if absent).

```

BEGIN CONSTRAINTS
[left_id] [right_id] [=?][gap]
END CONSTRAINTS

BEGIN NEIGHBOURS
[var1] [var2] [weight]
END NEIGHBOURS

BEGIN RESULTS
[id] [expected_pos]
END RESULTS
```

- [ ] **Step 1: Write fixture parser**

```rust
// tests/projection_solver/fixture_parser.rs

pub struct FixtureVariable {
    pub id: usize,
    pub desired_pos: f64,
    pub weight: f64,
    pub scale: f64,
}

pub struct FixtureConstraint {
    pub left: usize,
    pub right: usize,
    pub gap: f64,
    pub is_equality: bool,
}

pub struct FixtureNeighbor {
    pub var1: usize,
    pub var2: usize,
    pub weight: f64,
}

pub struct FixtureResult {
    pub id: usize,
    pub expected_pos: f64,
}

pub struct Fixture {
    pub variables: Vec<FixtureVariable>,
    pub constraints: Vec<FixtureConstraint>,
    pub neighbors: Vec<FixtureNeighbor>,
    pub results: Vec<FixtureResult>,
    pub expected_goal: Option<f64>,
}

pub fn parse_fixture(content: &str) -> Fixture {
    // Parse BEGIN/END sections
    // Handle variable lines: "id desired_pos size weight [scale]" (skip size column)
    // Handle constraint lines: "left right [=]gap"
    // Handle neighbor lines: "var1 var2 weight"
    // Handle result lines: "id expected_pos"
    // Handle header Goal line
    todo!()
}
```

The implementer should read 2-3 fixture files to understand the exact format, then implement the parser.

- [ ] **Step 2: Write fixture runner**

```rust
// tests/projection_solver/fixture_tests.rs
use super::fixture_parser::{parse_fixture, Fixture};
use msagl_rust::projection_solver::solver::Solver;

fn run_fixture(fixture: &Fixture) {
    let mut solver = Solver::new();
    let mut var_map = Vec::new();

    for v in &fixture.variables {
        let vi = solver.add_variable(v.desired_pos, v.weight, v.scale);
        var_map.push(vi);
    }

    for c in &fixture.constraints {
        solver.add_constraint(var_map[c.left], var_map[c.right], c.gap, c.is_equality);
    }

    for n in &fixture.neighbors {
        solver.add_neighbor_pair(var_map[n.var1], var_map[n.var2], n.weight);
    }

    let solution = solver.solve(None);

    // Verify each variable's position
    for r in &fixture.results {
        let actual = solver.variable(var_map[r.id]).actual_pos;
        assert!(
            (actual - r.expected_pos).abs() < 0.01,
            "Variable {} expected {:.5} but got {:.5}",
            r.id, r.expected_pos, actual
        );
    }

    // Verify goal function value if expected
    if let Some(expected_goal) = fixture.expected_goal {
        assert!(
            (solution.goal_function_value - expected_goal).abs() / expected_goal.abs().max(1.0) < 0.01,
            "Goal function expected {:.5} but got {:.5}",
            expected_goal, solution.goal_function_value
        );
    }
}
```

- [ ] **Step 3: Copy fixture files and create test functions**

Copy the 86 fixture files from `MSAGL-Reference/.../Data/` to `tests/fixtures/projection_solver/`.

Generate a `#[test]` function for each file:
```rust
#[test]
fn fixture_neighbors_vars10() {
    let content = include_str!("../fixtures/projection_solver/Neighbors_Vars10_ConstraintsMax3_NeighborsMax3_WeightMax100.txt");
    let fixture = parse_fixture(content);
    run_fixture(&fixture);
}
```

- [ ] **Step 4: Run fixture tests, verify results match C# expected values**

Run: `cd msagl-rust && cargo test --test projection_solver -- fixture`
Expected: All 86 fixture tests pass within tolerance.

- [ ] **Step 5: Commit**

```bash
git add tests/projection_solver/fixture_parser.rs tests/projection_solver/fixture_tests.rs tests/fixtures/
git commit -m "feat(msagl-rust): add fixture parser and 86 projection solver fixture tests"
```

---

### Task 10: Final Integration

**Files:** Various (lint fixes)

- [ ] **Step 1: Run full test suite**

Run: `cd msagl-rust && cargo test`
Expected: All tests pass (geometry + projection solver + fixtures).

- [ ] **Step 2: Run clippy**

Run: `cd msagl-rust && cargo clippy -- -D warnings`

- [ ] **Step 3: Fix any clippy issues**

- [ ] **Step 4: Verify final state**

Run: `cd msagl-rust && cargo test && cargo clippy -- -D warnings`
Expected: Clean pass.

- [ ] **Step 5: Commit**

```bash
git add -A msagl-rust/
git commit -m "chore(msagl-rust): Phase 2 complete — projection solver with 86 fixture tests"
```

---

## Summary

| Task | What | Key Files | Tests |
|------|------|-----------|-------|
| 1 | Module scaffolding | mod.rs, placeholders | 0 |
| 2 | Data types | variable.rs, constraint.rs, parameters.rs, solution.rs | ~8 |
| 3 | ConstraintVector | constraint_vector.rs | ~3 |
| 4 | ViolationCache | violation_cache.rs | ~3 |
| 5 | Block | block.rs | ~3 |
| 6 | Solver core | solver.rs (largest file, ~400-500 lines) | ~5 |
| 7 | QPSC | qpsc.rs | ~2 |
| 8 | SolverShell + Uniform | solver_shell.rs, uniform_solver.rs | ~2 |
| 9 | Fixture parser + tests | fixture_parser.rs, fixture_tests.rs | 86 |
| 10 | Integration | lint fixes | 0 |
| **Total** | | **12 source files** | **~110+ tests** |

## What Comes Next

**Phase 3: Visibility** — Visibility graph, obstacle tree, scan line. Plan will be written separately.
