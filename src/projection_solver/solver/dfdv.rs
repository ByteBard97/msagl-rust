use super::super::constraint::ConIndex;
use super::super::variable::VarIndex;
use super::Solver;

/// Node for stack-based DfDv tree traversal.
/// Faithfully matches C#'s DfDvNode structure.
pub(super) struct DfDvNode {
    /// The variable being evaluated at this tree node.
    variable_to_eval: VarIndex,
    /// The variable we came from (prevents backtracking).
    variable_done_eval: Option<VarIndex>,
    /// The constraint connecting this node to its parent.
    constraint_to_eval: ConIndex,
    /// Whether this node's constraint is a dummy (root node).
    is_dummy_constraint: bool,
    /// Whether children have already been pushed for this node.
    children_have_been_pushed: bool,
    /// Index of parent node in the node pool (for Lagrangian propagation).
    parent_idx: usize,
}

impl DfDvNode {
    /// Whether traversal is left-to-right (variable_to_eval == constraint.right).
    fn is_left_to_right(&self, solver: &Solver) -> bool {
        if self.is_dummy_constraint {
            return true; // Root node convention
        }
        solver.constraints[self.constraint_to_eval.0].right == self.variable_to_eval
    }
}

/// Direction pair for constraint path in Expand.
pub(super) struct ConstraintDirectionPair {
    pub constraint: ConIndex,
    pub is_forward: bool,
}

/// Sentinel index for the dummy parent node.
const DUMMY_PARENT_IDX: usize = usize::MAX;

impl Solver {
    /// Compute DfDv (Lagrangian multipliers) for all active constraints
    /// in the block containing `initial_var`.
    pub(super) fn compute_dfdv(&mut self, initial_var: VarIndex, _exclude: Option<VarIndex>) {
        self.reset_block_lagrangians(initial_var);
        self.dfs_compute_dfdv(initial_var, None);
    }

    /// Compute DfDv and also find the path from `initial_var` to `target_var`
    /// along the active constraint spanning tree. Returns the path as a list
    /// of constraint-direction pairs.
    pub(super) fn compute_dfdv_with_path(
        &mut self,
        initial_var: VarIndex,
        target_var: VarIndex,
    ) -> Vec<ConstraintDirectionPair> {
        self.reset_block_lagrangians(initial_var);
        self.dfs_compute_dfdv(initial_var, Some(target_var))
    }

    /// Get all variables connected to `start_var` via active constraints,
    /// excluding the subtree through `exclude_var`.
    ///
    /// Uses a generation counter (`visited_gen` / `visited_generation`) instead
    /// of allocating a `vec![false; n]` on every call. With 24K variables and
    /// 5K calls per solve, this avoids ~120MB of transient allocation.
    pub(super) fn get_connected_variables(
        &mut self,
        start_var: VarIndex,
        exclude_var: Option<VarIndex>,
    ) -> Vec<VarIndex> {
        self.visited_generation += 1;
        let gen = self.visited_generation;

        let mut result: Vec<VarIndex> = Vec::new();
        let mut stack: Vec<VarIndex> = Vec::new();

        self.visited_gen[start_var.0] = gen;
        if let Some(ev) = exclude_var {
            self.visited_gen[ev.0] = gen;
        }

        result.push(start_var);
        stack.push(start_var);

        while let Some(vi) = stack.pop() {
            // visited_gen is a separate Vec from variables/constraints,
            // so no borrow conflict when iterating constraint lists.
            for &ci in &self.variables[vi.0].left_constraints {
                if self.constraints[ci.0].is_active {
                    let right = self.constraints[ci.0].right;
                    if self.visited_gen[right.0] != gen {
                        self.visited_gen[right.0] = gen;
                        result.push(right);
                        stack.push(right);
                    }
                }
            }
            for &ci in &self.variables[vi.0].right_constraints {
                if self.constraints[ci.0].is_active {
                    let left = self.constraints[ci.0].left;
                    if self.visited_gen[left.0] != gen {
                        self.visited_gen[left.0] = gen;
                        result.push(left);
                        stack.push(left);
                    }
                }
            }
        }

        result
    }

    // ----- private helpers -----

    /// Reset lagrangians for all constraints whose left variable is in this block.
    fn reset_block_lagrangians(&mut self, var_in_block: VarIndex) {
        let block = self.variables[var_in_block.0].block;
        for var_i in 0..self.blocks[block.0].variables.len() {
            let vi = self.blocks[block.0].variables[var_i];
            for con_i in 0..self.variables[vi.0].left_constraints.len() {
                let ci = self.variables[vi.0].left_constraints[con_i];
                self.constraints[ci.0].lagrangian = 0.0;
            }
        }
    }

    /// DFS traversal matching C#'s ComputeDfDv exactly.
    ///
    /// Uses an explicit stack. Each iteration peeks at the top node:
    /// - If children haven't been pushed yet, push them (DFS order).
    /// - If children were just pushed, continue to process them first.
    /// - Once all children are processed, pop and compute Lagrangian.
    ///
    /// Leaf nodes (active_constraint_count == 1) are processed directly
    /// without being pushed onto the stack, matching C#'s optimization.
    fn dfs_compute_dfdv(
        &mut self,
        initial_var: VarIndex,
        target_var: Option<VarIndex>,
    ) -> Vec<ConstraintDirectionPair> {
        let mut constraint_path: Vec<ConstraintDirectionPair> = Vec::new();
        let mut path_found = false;

        // Reuse pooled Vecs from Solver to avoid repeated allocation.
        // Swap them out so we own them locally (no borrow on self).
        let mut nodes = std::mem::take(&mut self.dfdv_node_pool);
        let mut stack = std::mem::take(&mut self.dfdv_stack_pool);
        nodes.clear();
        stack.clear();

        // Create the first (root) node with a dummy constraint.
        nodes.push(DfDvNode {
            variable_to_eval: initial_var,
            variable_done_eval: None,
            constraint_to_eval: ConIndex(usize::MAX),
            is_dummy_constraint: true,
            children_have_been_pushed: false,
            parent_idx: DUMMY_PARENT_IDX,
        });

        let first_node_idx = 0;
        stack.push(first_node_idx);

        while let Some(&node_idx) = stack.last() {
            let prev_stack_len = stack.len();

            if !nodes[node_idx].children_have_been_pushed {
                nodes[node_idx].children_have_been_pushed = true;

                let vi = nodes[node_idx].variable_to_eval;
                let done_vi = nodes[node_idx].variable_done_eval;

                // Left constraints: variable is on left, traverse to right
                for con_i in 0..self.variables[vi.0].left_constraints.len() {
                    let ci = self.variables[vi.0].left_constraints[con_i];
                    if self.constraints[ci.0].is_active {
                        let right = self.constraints[ci.0].right;
                        if done_vi.is_none() || right != done_vi.unwrap() {
                            self.constraints[ci.0].lagrangian = 0.0;
                            let child_idx = nodes.len();
                            nodes.push(DfDvNode {
                                variable_to_eval: right,
                                variable_done_eval: Some(vi),
                                constraint_to_eval: ci,
                                is_dummy_constraint: false,
                                children_have_been_pushed: false,
                                parent_idx: node_idx,
                            });

                            // Leaf optimization: process directly if only 1 active constraint
                            if self.variables[right.0].active_constraint_count == 1 {
                                self.process_dfdv_node(
                                    &nodes,
                                    child_idx,
                                    target_var,
                                    &mut constraint_path,
                                    &mut path_found,
                                );
                            } else {
                                stack.push(child_idx);
                            }
                        }
                    }
                }

                // Right constraints: variable is on right, traverse to left
                for con_i in 0..self.variables[vi.0].right_constraints.len() {
                    let ci = self.variables[vi.0].right_constraints[con_i];
                    if self.constraints[ci.0].is_active {
                        let left = self.constraints[ci.0].left;
                        if done_vi.is_none() || left != done_vi.unwrap() {
                            self.constraints[ci.0].lagrangian = 0.0;
                            let child_idx = nodes.len();
                            nodes.push(DfDvNode {
                                variable_to_eval: left,
                                variable_done_eval: Some(vi),
                                constraint_to_eval: ci,
                                is_dummy_constraint: false,
                                children_have_been_pushed: false,
                                parent_idx: node_idx,
                            });

                            if self.variables[left.0].active_constraint_count == 1 {
                                self.process_dfdv_node(
                                    &nodes,
                                    child_idx,
                                    target_var,
                                    &mut constraint_path,
                                    &mut path_found,
                                );
                            } else {
                                stack.push(child_idx);
                            }
                        }
                    }
                }

                // If children were pushed, process them first (DFS).
                if stack.len() > prev_stack_len {
                    continue;
                }
            }

            // All children processed. Pop and compute this node's Lagrangian.
            stack.pop();
            self.process_dfdv_node(
                &nodes,
                node_idx,
                target_var,
                &mut constraint_path,
                &mut path_found,
            );

            if node_idx == first_node_idx {
                break;
            }
        }

        // Return pooled Vecs to Solver for reuse in future calls.
        self.dfdv_node_pool = nodes;
        self.dfdv_stack_pool = stack;

        constraint_path
    }

    /// Process a single DfDv node: compute its Lagrangian contribution
    /// and propagate to parent. Also check for constraint path target.
    ///
    /// Faithfully matches C#'s `ProcessDfDvLeafNode`.
    fn process_dfdv_node(
        &mut self,
        nodes: &[DfDvNode],
        node_idx: usize,
        target_var: Option<VarIndex>,
        constraint_path: &mut Vec<ConstraintDirectionPair>,
        path_found: &mut bool,
    ) {
        let node = &nodes[node_idx];

        // Skip the root node's dummy constraint
        if node.is_dummy_constraint {
            return;
        }

        let ci = node.constraint_to_eval;
        let vi = node.variable_to_eval;
        let dfdv = self.variables[vi.0].dfdv();
        let is_ltr = node.is_left_to_right(self);
        let parent_idx = node.parent_idx;

        if is_ltr {
            self.constraints[ci.0].lagrangian += dfdv;
            if parent_idx != DUMMY_PARENT_IDX {
                let parent = &nodes[parent_idx];
                if !parent.is_dummy_constraint {
                    let parent_ci = parent.constraint_to_eval;
                    self.constraints[parent_ci.0].lagrangian += self.constraints[ci.0].lagrangian;
                }
            }
        } else {
            self.constraints[ci.0].lagrangian = -(self.constraints[ci.0].lagrangian + dfdv);
            if parent_idx != DUMMY_PARENT_IDX {
                let parent = &nodes[parent_idx];
                if !parent.is_dummy_constraint {
                    let parent_ci = parent.constraint_to_eval;
                    self.constraints[parent_ci.0].lagrangian -= self.constraints[ci.0].lagrangian;
                }
            }
        }

        // Check for constraint path target.
        if let Some(target) = target_var {
            if vi == target && !*path_found {
                *path_found = true;
                let mut cur_idx = node_idx;
                while cur_idx != DUMMY_PARENT_IDX {
                    let cur = &nodes[cur_idx];
                    if !cur.is_dummy_constraint {
                        constraint_path.push(ConstraintDirectionPair {
                            constraint: cur.constraint_to_eval,
                            is_forward: cur.is_left_to_right(self),
                        });
                    }
                    cur_idx = cur.parent_idx;
                }
            }
        }
    }
}
