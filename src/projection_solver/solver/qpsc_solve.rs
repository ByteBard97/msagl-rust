//! QPSC integration into the Solver.
//!
//! Implements `solve_qpsc()` which uses the `Qpsc` state machine to drive
//! gradient-projected constraint solving when variables have neighbor pairs.

use super::super::block::{Block, BlockIndex};
use super::super::qpsc::Qpsc;
use super::super::solution::SolverAlgorithm;
use super::super::variable::VarIndex;
use super::Solver;

impl Solver {
    /// Run the full QPSC solve loop. Called when variables have neighbor
    /// relationships (or when `force_qpsc` is set).
    pub(super) fn solve_qpsc(&mut self) {
        self.solver_solution.algorithm_used = if self.solver_params.advanced.scale_in_qpsc {
            SolverAlgorithm::QpscWithScaling
        } else {
            SolverAlgorithm::QpscWithoutScaling
        };

        // Step 1: Make feasible -- one Project pass for a feasible starting point.
        if !self.qpsc_make_feasible() {
            return;
        }

        // Step 2: Initialize QPSC state.
        let mut qpsc = Qpsc::new(&self.solver_params, self.variables.len());

        // Add all variables (iterate blocks to visit each variable once).
        for block_i in 0..self.blocks_order.len() {
            let bi = self.blocks_order[block_i];
            for var_i in 0..self.blocks[bi.0].variables.len() {
                let vi = self.blocks[bi.0].variables[var_i];
                qpsc.add_variable(vi.0, &self.variables);
            }
        }

        // Step 3: Finalize QPSC setup -- apply diagonal scaling to variables.
        qpsc.variables_complete(&mut self.variables);
        qpsc.apply_scaling(&self.variables);

        // Step 4: Reinitialize blocks -- each variable gets its own block with
        // the new (scaled) weight=1 and desired_pos=actual_pos.
        self.reinitialize_blocks();

        // Step 5: Re-merge equality constraints in the fresh block structure.
        self.merge_equality_constraints();

        // Step 6: Main QPSC iteration loop.
        let mut found_split = false;
        loop {
            // Gradient step: compute alpha and update desired positions.
            if !qpsc.pre_project(&mut self.variables) && !found_split {
                break;
            }

            // Split blocks with negative Lagrangian multipliers.
            found_split = self.split_blocks();

            // Project: satisfy constraints by merging violated blocks.
            if !self.run_project() {
                break;
            }

            // Beta step: adjust positions based on projection movement.
            if !qpsc.post_project(&self.variables) && !found_split {
                break;
            }
        }

        // Step 7: Restore original variable state and record goal function value.
        self.solver_solution.goal_function_value = qpsc.qpsc_complete(&mut self.variables);
    }

    /// Run a single Project pass to establish feasibility.
    /// Returns false if iteration limits were exceeded.
    fn qpsc_make_feasible(&mut self) -> bool {
        self.run_project()
    }

    /// Discard the current block structure and put each variable into its own
    /// fresh block. This is needed after QPSC has modified variable weights
    /// and desired positions, since the old block structure didn't account for
    /// neighbor forces.
    fn reinitialize_blocks(&mut self) {
        // Collect all variable indices from all current blocks.
        let mut all_vars: Vec<VarIndex> = Vec::new();
        for block_i in 0..self.blocks_order.len() {
            let bi = self.blocks_order[block_i];
            for var_i in 0..self.blocks[bi.0].variables.len() {
                all_vars.push(self.blocks[bi.0].variables[var_i]);
            }
        }

        // Clear block tracking.
        self.blocks_order.clear();
        self.block_vector_indices.clear();

        // Reinitialize each variable: reset offset and active constraint count.
        for &vi in &all_vars {
            self.variables[vi.0].active_constraint_count = 0;
            self.variables[vi.0].offset_in_block = 0.0;

            // Create a new block for this variable.
            let bi = BlockIndex(self.blocks.len());
            let v = &self.variables[vi.0];
            let block = Block::new(bi, vi, v.desired_pos, v.weight, v.scale);
            self.blocks.push(block);
            self.variables[vi.0].block = bi;

            // Track in block vector.
            if bi.0 >= self.block_vector_indices.len() {
                self.block_vector_indices.resize(bi.0 + 1, usize::MAX);
            }
            self.block_vector_indices[bi.0] = self.blocks_order.len();
            self.blocks_order.push(bi);
        }

        // Reinitialize constraint vector (deactivate all constraints).
        self.constraint_vector.reinitialize();
        for c in &mut self.constraints {
            c.is_active = false;
            c.is_unsatisfiable = false;
            c.lagrangian = 0.0;
        }
        self.violation_cache.clear();
    }
}
