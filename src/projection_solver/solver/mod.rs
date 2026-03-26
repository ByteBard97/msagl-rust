use super::block::{Block, BlockIndex};
use super::constraint::{ConIndex, Constraint};
use super::constraint_vector::ConstraintVector;
use super::parameters::Parameters;
use super::solution::Solution;
use super::variable::{NeighborAndWeight, VarIndex, Variable};
use super::violation_cache::ViolationCache;

mod dfdv;
mod qpsc_solve;
mod solver_impl;

/// Loaded constraint info before solve() converts to arrays.
struct LoadedConstraint {
    left: VarIndex,
    right: VarIndex,
    gap: f64,
    is_equality: bool,
}

/// The projection solver. Accepts variables/constraints and solves for
/// minimum-displacement positions satisfying all constraints.
pub struct Solver {
    pub(crate) variables: Vec<Variable>,
    pub(crate) constraints: Vec<Constraint>,
    pub(crate) blocks: Vec<Block>,
    constraint_vector: ConstraintVector,
    violation_cache: ViolationCache,
    equality_constraints: Vec<ConIndex>,
    is_qpsc: bool,
    // Load-phase storage
    loaded_constraints: Vec<LoadedConstraint>,
    next_variable_ordinal: u32,
    is_setup: bool,
    // Block vector management (swap-remove)
    block_vector_indices: Vec<usize>, // block index -> position in blocks_order
    blocks_order: Vec<BlockIndex>,    // dense list for iteration
    // Params and solution
    solver_params: Parameters,
    solver_solution: Solution,
    violation_cache_min_block_cutoff: usize,
}

impl Default for Solver {
    fn default() -> Self {
        Self::new()
    }
}

impl Solver {
    pub fn new() -> Self {
        Self {
            variables: Vec::new(),
            constraints: Vec::new(),
            blocks: Vec::new(),
            constraint_vector: ConstraintVector::new(0),
            violation_cache: ViolationCache::new(),
            equality_constraints: Vec::new(),
            is_qpsc: false,
            loaded_constraints: Vec::new(),
            next_variable_ordinal: 0,
            is_setup: false,
            block_vector_indices: Vec::new(),
            blocks_order: Vec::new(),
            solver_params: Parameters::default(),
            solver_solution: Solution::new(),
            violation_cache_min_block_cutoff: usize::MAX,
        }
    }

    /// Add a variable with desired position, weight, and scale. Returns its index.
    pub fn add_variable(&mut self, desired_pos: f64, weight: f64, scale: f64) -> VarIndex {
        let vi = VarIndex(self.variables.len());
        let mut var = Variable::new(vi, desired_pos, weight, scale);
        var.ordinal = self.next_variable_ordinal;
        self.next_variable_ordinal += 1;

        // Create a block for this variable
        let bi = BlockIndex(self.blocks.len());
        var.block = bi;
        self.variables.push(var);

        let block = Block::new(bi, vi, desired_pos, weight, scale);
        // Track in block vector
        self.block_vector_indices.push(self.blocks_order.len());
        self.blocks_order.push(bi);
        self.blocks.push(block);

        vi
    }

    /// Add a constraint: left + gap <= right (or == if is_equality).
    pub fn add_constraint(
        &mut self,
        left: VarIndex,
        right: VarIndex,
        gap: f64,
        is_equality: bool,
    ) -> ConIndex {
        let ci = ConIndex(self.loaded_constraints.len());
        if is_equality {
            self.equality_constraints.push(ci);
        }
        self.loaded_constraints.push(LoadedConstraint {
            left,
            right,
            gap,
            is_equality,
        });
        ci
    }

    /// Add a neighbor pair for QPSC goal functions.
    pub fn add_neighbor_pair(&mut self, v1: VarIndex, v2: VarIndex, weight: f64) {
        self.variables[v1.0].neighbors.push(NeighborAndWeight {
            neighbor: v2,
            weight,
        });
        self.variables[v2.0].neighbors.push(NeighborAndWeight {
            neighbor: v1,
            weight,
        });
        self.is_qpsc = true;
    }

    /// Get a reference to a variable by index.
    pub fn variable(&self, vi: VarIndex) -> &Variable {
        &self.variables[vi.0]
    }

    /// Solve the constraint system, returning solution statistics.
    pub fn solve(&mut self, params: Option<Parameters>) -> Solution {
        if let Some(p) = params {
            self.solver_params = p;
        }

        let num_vars = self.variables.len();
        let num_cons = self.loaded_constraints.len();

        // Set default iteration limits
        if self.solver_params.outer_project_iterations_limit < 0 {
            self.solver_params.outer_project_iterations_limit =
                100 * ((num_vars as f64).log2().floor() as i32 + 1);
        }
        if self.solver_params.inner_project_iterations_limit < 0 {
            self.solver_params.inner_project_iterations_limit = num_cons as i32 * 2
                + 100 * (std::cmp::max(0, (num_cons as f64).log2().floor() as i32) + 1);
        }

        self.solver_solution = Solution::new();
        self.solver_solution.min_inner_project_iterations = u32::MAX;

        // If no constraints, nothing to do (non-QPSC path)
        if num_cons == 0 && !self.is_qpsc {
            return self.solver_solution.clone();
        }

        if !self.is_setup {
            self.setup_constraints();
            self.is_setup = true;
        }

        self.constraint_vector.number_of_unsatisfiable_constraints = 0;

        // Merge equality constraints
        self.merge_equality_constraints();

        // Setup violation cache cutoff
        if self.solver_params.advanced.use_violation_cache
            && self
                .solver_params
                .advanced
                .violation_cache_min_blocks_divisor
                > 0
        {
            self.violation_cache_min_block_cutoff = std::cmp::min(
                self.blocks_order.len()
                    / self
                        .solver_params
                        .advanced
                        .violation_cache_min_blocks_divisor,
                self.solver_params.advanced.violation_cache_min_blocks_count,
            );
        }

        if self.is_qpsc || self.solver_params.advanced.force_qpsc {
            self.solve_qpsc();
        } else {
            self.solve_by_standalone_project();
            self.calculate_goal_function_value();
        }

        // Fix min iterations
        if self.solver_solution.min_inner_project_iterations
            > self.solver_solution.max_inner_project_iterations
        {
            self.solver_solution.min_inner_project_iterations =
                self.solver_solution.max_inner_project_iterations;
        }

        self.solver_solution.number_of_unsatisfiable_constraints =
            self.constraint_vector.number_of_unsatisfiable_constraints;
        self.solver_solution.max_constraint_tree_depth =
            self.constraint_vector.max_constraint_tree_depth;

        self.solver_solution.clone()
    }

    /// Convert loaded constraints into the constraint/variable arrays.
    fn setup_constraints(&mut self) {
        let num_cons = self.loaded_constraints.len();
        self.constraints.clear();
        self.constraints.reserve(num_cons);

        for i in 0..num_cons {
            let lc = &self.loaded_constraints[i];
            let c = Constraint::new(ConIndex(i), lc.left, lc.right, lc.gap, lc.is_equality);
            self.constraints.push(c);
        }

        // Build per-variable left/right constraint lists
        for vi in 0..self.variables.len() {
            self.variables[vi].left_constraints.clear();
            self.variables[vi].right_constraints.clear();
        }
        for ci in 0..num_cons {
            let left = self.constraints[ci].left;
            let right = self.constraints[ci].right;
            self.variables[left.0].left_constraints.push(ConIndex(ci));
            self.variables[right.0].right_constraints.push(ConIndex(ci));
        }

        self.constraint_vector = ConstraintVector::new(num_cons);
    }

    /// Main standalone project loop: project then split, repeat.
    fn solve_by_standalone_project(&mut self) {
        loop {
            if !self.run_project() {
                return;
            }
            if !self.split_blocks() {
                break;
            }
        }
    }

    fn run_project(&mut self) -> bool {
        self.solver_solution.outer_project_iterations += 1;
        self.project();
        !self.check_for_limits_exceeded()
    }

    fn check_for_limits_exceeded(&self) -> bool {
        if self.solver_params.outer_project_iterations_limit > 0
            && self.solver_solution.outer_project_iterations
                >= self.solver_params.outer_project_iterations_limit as u32
        {
            return true;
        }
        if self.solver_solution.inner_project_iterations_limit_exceeded {
            return true;
        }
        false
    }

    fn calculate_goal_function_value(&mut self) {
        let mut value = 0.0;
        for vi in 0..self.variables.len() {
            let v = &self.variables[vi];
            value += v.weight * (v.actual_pos * v.actual_pos);
            value -= 2.0 * (v.weight * (v.desired_pos * v.actual_pos));
        }
        self.solver_solution.goal_function_value = value;
    }

    // Block vector helpers (swap-remove pattern)
    fn add_block_to_vector(&mut self, bi: BlockIndex) {
        if bi.0 >= self.block_vector_indices.len() {
            self.block_vector_indices.resize(bi.0 + 1, usize::MAX);
        }
        self.block_vector_indices[bi.0] = self.blocks_order.len();
        self.blocks_order.push(bi);
    }

    fn remove_block_from_vector(&mut self, bi: BlockIndex) {
        let pos = self.block_vector_indices[bi.0];
        let last = *self.blocks_order.last().unwrap();
        self.blocks_order[pos] = last;
        self.block_vector_indices[last.0] = pos;
        self.blocks_order.pop();
        self.block_vector_indices[bi.0] = usize::MAX;
    }

    fn active_block_count(&self) -> usize {
        self.blocks_order.len()
    }

    /// Activate a constraint: swap in constraint_vector, update active counts.
    fn activate_constraint(&mut self, ci: ConIndex) {
        self.constraint_vector.activate(ci);
        self.constraints[ci.0].is_active = true;
        let left = self.constraints[ci.0].left;
        let right = self.constraints[ci.0].right;
        self.variables[left.0].active_constraint_count += 1;
        self.variables[right.0].active_constraint_count += 1;
    }

    /// Deactivate a constraint: swap in constraint_vector, update active counts.
    fn deactivate_constraint(&mut self, ci: ConIndex) {
        self.constraint_vector.deactivate(ci);
        self.constraints[ci.0].is_active = false;
        let left = self.constraints[ci.0].left;
        let right = self.constraints[ci.0].right;
        self.variables[left.0].active_constraint_count -= 1;
        self.variables[right.0].active_constraint_count -= 1;
    }

    /// Compute violation of a constraint from current variable positions.
    fn constraint_violation(&self, ci: ConIndex) -> f64 {
        let c = &self.constraints[ci.0];
        let left = &self.variables[c.left.0];
        let right = &self.variables[c.right.0];
        left.actual_pos * left.scale + c.gap - right.actual_pos * right.scale
    }
}
