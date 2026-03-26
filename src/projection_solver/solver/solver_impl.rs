use super::super::block::{Block, BlockIndex};
use super::super::constraint::ConIndex;
use super::Solver;

impl Solver {
    pub(super) fn merge_equality_constraints(&mut self) {
        for i in 0..self.equality_constraints.len() {
            let ci = self.equality_constraints[i];
            let left_vi = self.constraints[ci.0].left;
            let right_vi = self.constraints[ci.0].right;
            let left_block = self.variables[left_vi.0].block;
            let right_block = self.variables[right_vi.0].block;

            if left_block == right_block {
                // Already same block -- check for unsatisfiable
                let vio = self.constraint_violation(ci).abs();
                if vio > self.solver_params.gap_tolerance {
                    self.constraints[ci.0].is_unsatisfiable = true;
                    self.constraint_vector.number_of_unsatisfiable_constraints += 1;
                }
                continue;
            }

            self.merge_blocks(ci);
        }
    }

    pub(super) fn project(&mut self) {
        if self.constraints.is_empty() {
            return;
        }

        self.violation_cache.clear();
        let mut last_modified_block: Option<BlockIndex> = None;
        let use_cache = self.active_block_count() > self.violation_cache_min_block_cutoff;
        let mut iterations: u32 = 1;

        let mut max_violated = self.get_max_violated_constraint(last_modified_block, use_cache);
        if max_violated.is_none() {
            return;
        }

        while let Some(ci) = max_violated {
            let left_vi = self.constraints[ci.0].left;
            let right_vi = self.constraints[ci.0].right;
            let left_block = self.variables[left_vi.0].block;
            let right_block = self.variables[right_vi.0].block;

            if left_block == right_block {
                self.expand(ci);
                if self.constraints[ci.0].is_unsatisfiable {
                    self.violation_cache.clear();
                }
                last_modified_block = Some(self.variables[left_vi.0].block);
            } else {
                let merged_block = self.merge_blocks(ci);
                last_modified_block = Some(merged_block);
            }

            if self.solver_params.inner_project_iterations_limit > 0
                && iterations >= self.solver_params.inner_project_iterations_limit as u32
            {
                self.solver_solution.inner_project_iterations_limit_exceeded = true;
                break;
            }

            let use_cache = self.active_block_count() > self.violation_cache_min_block_cutoff;
            if !use_cache {
                self.violation_cache.clear();
            }

            iterations += 1;
            max_violated = self.get_max_violated_constraint(last_modified_block, use_cache);
        }

        self.solver_solution.inner_project_iterations_total += iterations as u64;
        if self.solver_solution.max_inner_project_iterations < iterations {
            self.solver_solution.max_inner_project_iterations = iterations;
        }
        if self.solver_solution.min_inner_project_iterations > iterations {
            self.solver_solution.min_inner_project_iterations = iterations;
        }
    }

    /// C#-compatible epsilon for approximate comparison.
    const DISTANCE_EPSILON: f64 = 1e-6;

    fn get_max_violated_constraint(
        &mut self,
        last_modified_block: Option<BlockIndex>,
        use_cache: bool,
    ) -> Option<ConIndex> {
        let max_violation = self.solver_params.gap_tolerance;

        // Try the violation cache path first (C#'s SearchViolationCache).
        if let Some(result) = self.search_violation_cache(last_modified_block, max_violation) {
            return Some(result);
        }

        // Fall through to full scan (C#'s SearchAllConstraints).
        self.search_all_constraints(max_violation, use_cache)
    }

    /// Matches C#'s SearchViolationCache.
    fn search_violation_cache(
        &mut self,
        last_modified_block: Option<BlockIndex>,
        mut max_violation: f64,
    ) -> Option<ConIndex> {
        let lmb = last_modified_block?;

        // Only use cache if block is small enough.
        if self.blocks[lmb.0].variables.len() >= self.variables.len() / 2 {
            return None;
        }

        // Filter the cache: remove entries for lastModifiedBlock.
        // Closures need to borrow different fields.
        let constraints = &self.constraints;
        let variables = &self.variables;
        let filter_result = self.violation_cache.filter_block(
            |ci| {
                let c = &constraints[ci.0];
                let lb = variables[c.left.0].block;
                let rb = variables[c.right.0].block;
                lb == lmb || rb == lmb || c.is_active || c.is_unsatisfiable
            },
            |ci| {
                let c = &constraints[ci.0];
                let left = &variables[c.left.0];
                let right = &variables[c.right.0];
                left.actual_pos * left.scale + c.gap - right.actual_pos * right.scale
            },
        );

        if !filter_result {
            return None;
        }

        let mut max_ci: Option<ConIndex> = None;

        // Scan constraints in lastModifiedBlock.
        for var_i in 0..self.blocks[lmb.0].variables.len() {
            let vi = self.blocks[lmb.0].variables[var_i];
            // Left constraints of this variable.
            for con_i in 0..self.variables[vi.0].left_constraints.len() {
                let ci = self.variables[vi.0].left_constraints[con_i];
                let c = &self.constraints[ci.0];
                if !c.is_active && !c.is_unsatisfiable {
                    let violation = self.constraint_violation(ci);
                    if violation - max_violation >= Self::DISTANCE_EPSILON {
                        if let Some(prev_ci) = max_ci {
                            if max_violation > self.violation_cache.low_violation() {
                                let constraints = &self.constraints;
                                let variables = &self.variables;
                                self.violation_cache.insert(prev_ci, max_violation, &|ci| {
                                    let c = &constraints[ci.0];
                                    let l = &variables[c.left.0];
                                    let r = &variables[c.right.0];
                                    l.actual_pos * l.scale + c.gap - r.actual_pos * r.scale
                                });
                            }
                        }
                        max_violation = violation;
                        max_ci = Some(ci);
                    }
                }
            }

            // Right constraints where left var is NOT in this block.
            for con_i in 0..self.variables[vi.0].right_constraints.len() {
                let ci = self.variables[vi.0].right_constraints[con_i];
                let c = &self.constraints[ci.0];
                if !c.is_active && !c.is_unsatisfiable && self.variables[c.left.0].block != lmb {
                    let violation = self.constraint_violation(ci);
                    if violation - max_violation >= Self::DISTANCE_EPSILON {
                        if let Some(prev_ci) = max_ci {
                            if max_violation > self.violation_cache.low_violation() {
                                let constraints = &self.constraints;
                                let variables = &self.variables;
                                self.violation_cache.insert(prev_ci, max_violation, &|ci| {
                                    let c = &constraints[ci.0];
                                    let l = &variables[c.left.0];
                                    let r = &variables[c.right.0];
                                    l.actual_pos * l.scale + c.gap - r.actual_pos * r.scale
                                });
                            }
                        }
                        max_violation = violation;
                        max_ci = Some(ci);
                    }
                }
            }
        }

        // Check cached entries.
        let constraints = &self.constraints;
        let variables = &self.variables;
        let get_vio = |ci: ConIndex| -> f64 {
            let c = &constraints[ci.0];
            let l = &variables[c.left.0];
            let r = &variables[c.right.0];
            l.actual_pos * l.scale + c.gap - r.actual_pos * r.scale
        };
        if let Some(cached_ci) = self
            .violation_cache
            .find_if_greater(max_violation, &get_vio)
        {
            if let Some(prev_ci) = max_ci {
                if max_violation > self.violation_cache.low_violation() {
                    self.violation_cache
                        .insert(prev_ci, max_violation, &get_vio);
                }
            }
            max_ci = Some(cached_ci);
        }

        max_ci
    }

    /// Full scan of all constraints. Matches C#'s SearchAllConstraints.
    fn search_all_constraints(
        &mut self,
        mut max_violation: f64,
        use_cache: bool,
    ) -> Option<ConIndex> {
        let mut max_ci: Option<ConIndex> = None;
        self.violation_cache.clear();

        let active_count = self.constraint_vector.active_count();
        let total_count = self.constraint_vector.all_constraints().len();
        let inactive_count = total_count - active_count;

        for idx in 0..inactive_count {
            let ci = self.constraint_vector.all_constraints()[idx];
            let c = &self.constraints[ci.0];
            if c.is_active {
                break;
            }
            if c.is_unsatisfiable {
                continue;
            }
            let violation = self.constraint_violation(ci);

            if violation - max_violation >= Self::DISTANCE_EPSILON {
                if use_cache {
                    if let Some(prev_ci) = max_ci {
                        if max_violation > self.violation_cache.low_violation() {
                            let constraints = &self.constraints;
                            let variables = &self.variables;
                            self.violation_cache.insert(prev_ci, max_violation, &|ci| {
                                let c = &constraints[ci.0];
                                let l = &variables[c.left.0];
                                let r = &variables[c.right.0];
                                l.actual_pos * l.scale + c.gap - r.actual_pos * r.scale
                            });
                        }
                    }
                }
                max_violation = violation;
                max_ci = Some(ci);
            } else if use_cache {
                // Cache non-max violations too.
                if max_ci != Some(ci)
                    && (!self.violation_cache.is_full()
                        || violation > self.violation_cache.low_violation())
                {
                    let constraints = &self.constraints;
                    let variables = &self.variables;
                    if violation > self.violation_cache.low_violation() {
                        self.violation_cache.insert(ci, violation, &|ci| {
                            let c = &constraints[ci.0];
                            let l = &variables[c.left.0];
                            let r = &variables[c.right.0];
                            l.actual_pos * l.scale + c.gap - r.actual_pos * r.scale
                        });
                    }
                }
            }
        }

        max_ci
    }

    pub(super) fn merge_blocks(&mut self, violated_ci: ConIndex) -> BlockIndex {
        let left_vi = self.constraints[violated_ci.0].left;
        let right_vi = self.constraints[violated_ci.0].right;
        let mut block_to = self.variables[left_vi.0].block;
        let mut block_from = self.variables[right_vi.0].block;

        let mut distance = self.variables[left_vi.0].offset_in_block
            + self.constraints[violated_ci.0].gap
            - self.variables[right_vi.0].offset_in_block;

        // Move from smaller block to larger
        if self.blocks[block_from.0].variables.len() > self.blocks[block_to.0].variables.len() {
            std::mem::swap(&mut block_to, &mut block_from);
            distance = -distance;
        }

        for i in 0..self.blocks[block_from.0].variables.len() {
            let vi = self.blocks[block_from.0].variables[i];
            self.variables[vi.0].offset_in_block += distance;
            let v = &self.variables[vi.0];
            let offset = v.offset_in_block;
            let weight = v.weight;
            let var_scale = v.scale;
            let desired = v.desired_pos;
            self.blocks[block_to.0]
                .add_variable_with_offset(vi, offset, weight, var_scale, desired);
            self.variables[vi.0].block = block_to;
        }

        self.update_reference_pos_from_sums(block_to);
        self.activate_constraint(violated_ci);
        self.remove_block_from_vector(block_from);

        block_to
    }

    pub(super) fn split_blocks(&mut self) -> bool {
        let mut new_blocks: Vec<BlockIndex> = Vec::new();
        let num_blocks = self.blocks_order.len();

        for block_i in 0..num_blocks {
            let bi = self.blocks_order[block_i];
            if let Some(new_bi) = self.split_block(bi) {
                new_blocks.push(new_bi);
            }
        }

        for bi in &new_blocks {
            self.add_block_to_vector(*bi);
        }

        !new_blocks.is_empty()
    }

    fn split_block(&mut self, bi: BlockIndex) -> Option<BlockIndex> {
        if self.is_qpsc {
            // In QPSC mode, PreProject has updated desired positions, so
            // recompute block reference positions before splitting.
            self.full_update_reference_pos(bi);
        }

        if self.blocks[bi.0].variables.len() < 2 {
            return None;
        }

        let first_var = self.blocks[bi.0].variables[0];
        self.compute_dfdv(first_var, None);

        let min_threshold = self.solver_params.advanced.min_split_lagrangian_threshold;
        let mut min_lagrangian = min_threshold;
        let mut min_ci: Option<ConIndex> = None;

        for var_i in 0..self.blocks[bi.0].variables.len() {
            let vi = self.blocks[bi.0].variables[var_i];
            for con_i in 0..self.variables[vi.0].left_constraints.len() {
                let ci = self.variables[vi.0].left_constraints[con_i];
                let c = &self.constraints[ci.0];
                if c.is_active && !c.is_equality && c.lagrangian < min_lagrangian {
                    min_ci = Some(ci);
                    min_lagrangian = c.lagrangian;
                }
            }
        }

        let split_ci = min_ci?;
        self.split_on_constraint(bi, split_ci)
    }

    fn split_on_constraint(&mut self, bi: BlockIndex, ci: ConIndex) -> Option<BlockIndex> {
        self.deactivate_constraint(ci);

        let right_var = self.constraints[ci.0].right;
        let left_var = self.constraints[ci.0].left;

        let new_bi = BlockIndex(self.blocks.len());
        self.blocks.push(Block {
            index: new_bi,
            variables: Vec::new(),
            reference_pos: 0.0,
            scale: 1.0,
            sum_a2: 0.0,
            sum_ad: 0.0,
            sum_ab: 0.0,
            vector_index: 0,
        });

        let connected_vars = self.get_connected_variables(right_var, Some(left_var));

        if connected_vars.is_empty() {
            return None;
        }

        for &vi in &connected_vars {
            self.variables[vi.0].block = new_bi;
        }
        self.blocks[new_bi.0].variables = connected_vars.clone();

        self.blocks[bi.0]
            .variables
            .retain(|&vi| self.variables[vi.0].block != new_bi);

        if self.blocks[bi.0].variables.is_empty() {
            for &vi in &connected_vars {
                self.variables[vi.0].block = bi;
            }
            self.blocks[bi.0].variables = connected_vars;
            self.blocks[new_bi.0].variables.clear();
            return None;
        }

        self.full_update_reference_pos(bi);
        self.full_update_reference_pos(new_bi);

        Some(new_bi)
    }

    fn expand(&mut self, violated_ci: ConIndex) {
        let left_var = self.constraints[violated_ci.0].left;
        let right_var = self.constraints[violated_ci.0].right;

        let path = self.compute_dfdv_with_path(left_var, right_var);

        let mut min_lagrangian_ci: Option<ConIndex> = None;
        let mut min_lag = f64::MAX;

        for item in &path {
            if item.is_forward {
                let c = &self.constraints[item.constraint.0];
                if !c.is_equality && c.lagrangian < min_lag {
                    min_lag = c.lagrangian;
                    min_lagrangian_ci = Some(item.constraint);
                }
            }
        }

        if let Some(min_ci) = min_lagrangian_ci {
            self.deactivate_constraint(min_ci);
        } else {
            self.constraints[violated_ci.0].is_unsatisfiable = true;
            self.constraint_vector.number_of_unsatisfiable_constraints += 1;
            return;
        }

        let connected = self.get_connected_variables(right_var, Some(left_var));
        let violation = self.constraint_violation(violated_ci);
        for &vi in &connected {
            self.variables[vi.0].offset_in_block += violation;
        }

        self.activate_constraint(violated_ci);
        self.constraints[violated_ci.0].lagrangian = 0.0;

        let block = self.variables[left_var.0].block;
        self.full_update_reference_pos(block);
    }

    pub(super) fn full_update_reference_pos(&mut self, bi: BlockIndex) {
        if self.blocks[bi.0].variables.is_empty() {
            return;
        }

        let first_var = self.blocks[bi.0].variables[0];
        self.blocks[bi.0].scale = self.variables[first_var.0].scale;
        self.blocks[bi.0].reset_sums();

        for i in 0..self.blocks[bi.0].variables.len() {
            let vi = self.blocks[bi.0].variables[i];
            let v = &self.variables[vi.0];
            self.blocks[bi.0].add_to_sums(v.offset_in_block, v.weight, v.scale, v.desired_pos);
        }

        self.update_reference_pos_from_sums(bi);
    }

    /// Update reference pos from current sums and update variable positions.
    fn update_reference_pos_from_sums(&mut self, bi: BlockIndex) {
        self.blocks[bi.0].update_reference_pos();
        let scaled_ref = self.blocks[bi.0].scale * self.blocks[bi.0].reference_pos;
        for i in 0..self.blocks[bi.0].variables.len() {
            let vi = self.blocks[bi.0].variables[i];
            let v = &self.variables[vi.0];
            let new_pos = (scaled_ref + v.offset_in_block) / v.scale;
            self.variables[vi.0].actual_pos = new_pos;
        }
    }
}
