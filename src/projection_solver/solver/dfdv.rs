use super::super::constraint::ConIndex;
use super::super::variable::VarIndex;
use super::Solver;

/// Node for stack-based DfDv tree traversal.
struct DfDvNode {
    variable_to_eval: VarIndex,
    variable_done_eval: Option<VarIndex>,
    constraint_to_eval: ConIndex,
    is_left_to_right: bool,
    children_have_been_pushed: bool,
}

/// Direction pair for constraint path in Expand.
pub(super) struct ConstraintDirectionPair {
    pub constraint: ConIndex,
    pub is_forward: bool,
}

/// Tree entry for breadth-first expansion, then post-order processing.
struct TreeEntry {
    node: DfDvNode,
    parent_idx: usize,
}

impl Solver {
    /// Compute DfDv (Lagrangian multipliers) for all active constraints
    /// in the block containing `initial_var`.
    pub(super) fn compute_dfdv(&mut self, initial_var: VarIndex, _exclude: Option<VarIndex>) {
        self.reset_block_lagrangians(initial_var);
        let dummy_ci = ConIndex(usize::MAX);
        let mut entries = self.build_constraint_tree(initial_var, dummy_ci);
        self.process_post_order(&mut entries, dummy_ci);
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
        let dummy_ci = ConIndex(usize::MAX);
        let mut entries = self.build_constraint_tree(initial_var, dummy_ci);
        self.process_post_order_with_path(&mut entries, dummy_ci, target_var)
    }

    /// Get all variables connected to `start_var` via active constraints,
    /// excluding the subtree through `exclude_var`.
    pub(super) fn get_connected_variables(
        &self,
        start_var: VarIndex,
        exclude_var: Option<VarIndex>,
    ) -> Vec<VarIndex> {
        let mut result: Vec<VarIndex> = Vec::new();
        let mut stack: Vec<VarIndex> = Vec::new();
        let mut visited = vec![false; self.variables.len()];

        visited[start_var.0] = true;
        if let Some(ev) = exclude_var {
            visited[ev.0] = true;
        }

        result.push(start_var);
        stack.push(start_var);

        while let Some(vi) = stack.pop() {
            for &ci in &self.variables[vi.0].left_constraints {
                if self.constraints[ci.0].is_active {
                    let right = self.constraints[ci.0].right;
                    if !visited[right.0] {
                        visited[right.0] = true;
                        result.push(right);
                        stack.push(right);
                    }
                }
            }
            for &ci in &self.variables[vi.0].right_constraints {
                if self.constraints[ci.0].is_active {
                    let left = self.constraints[ci.0].left;
                    if !visited[left.0] {
                        visited[left.0] = true;
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
        let vars: Vec<VarIndex> = self.blocks[block.0].variables.clone();
        for &vi in &vars {
            for &ci in &self.variables[vi.0].left_constraints.clone() {
                self.constraints[ci.0].lagrangian = 0.0;
            }
        }
    }

    /// Build a tree of active-constraint connections by breadth-first expansion
    /// from `initial_var`. Returns entries suitable for post-order processing.
    fn build_constraint_tree(
        &mut self,
        initial_var: VarIndex,
        dummy_ci: ConIndex,
    ) -> Vec<TreeEntry> {
        let mut entries: Vec<TreeEntry> = Vec::new();
        entries.push(TreeEntry {
            node: DfDvNode {
                variable_to_eval: initial_var,
                variable_done_eval: None,
                constraint_to_eval: dummy_ci,
                is_left_to_right: true,
                children_have_been_pushed: false,
            },
            parent_idx: usize::MAX,
        });

        let mut idx = 0;
        while idx < entries.len() {
            if entries[idx].node.children_have_been_pushed {
                idx += 1;
                continue;
            }
            entries[idx].node.children_have_been_pushed = true;

            let vi = entries[idx].node.variable_to_eval;
            let done_vi = entries[idx].node.variable_done_eval;
            let current_idx = idx;

            let left_cons: Vec<ConIndex> = self.variables[vi.0].left_constraints.clone();
            for ci in left_cons {
                if self.constraints[ci.0].is_active {
                    let right = self.constraints[ci.0].right;
                    if done_vi.is_none() || right != done_vi.unwrap() {
                        self.constraints[ci.0].lagrangian = 0.0;
                        entries.push(TreeEntry {
                            node: DfDvNode {
                                variable_to_eval: right,
                                variable_done_eval: Some(vi),
                                constraint_to_eval: ci,
                                is_left_to_right: true,
                                children_have_been_pushed: false,
                            },
                            parent_idx: current_idx,
                        });
                    }
                }
            }

            let right_cons: Vec<ConIndex> = self.variables[vi.0].right_constraints.clone();
            for ci in right_cons {
                if self.constraints[ci.0].is_active {
                    let left = self.constraints[ci.0].left;
                    if done_vi.is_none() || left != done_vi.unwrap() {
                        self.constraints[ci.0].lagrangian = 0.0;
                        entries.push(TreeEntry {
                            node: DfDvNode {
                                variable_to_eval: left,
                                variable_done_eval: Some(vi),
                                constraint_to_eval: ci,
                                is_left_to_right: false,
                                children_have_been_pushed: false,
                            },
                            parent_idx: current_idx,
                        });
                    }
                }
            }

            idx += 1;
        }

        entries
    }

    /// Process tree entries in post-order to compute Lagrangian multipliers.
    fn process_post_order(&mut self, entries: &mut [TreeEntry], dummy_ci: ConIndex) {
        for i in (0..entries.len()).rev() {
            let ci = entries[i].node.constraint_to_eval;
            if ci == dummy_ci {
                continue;
            }

            let vi = entries[i].node.variable_to_eval;
            let dfdv = self.variables[vi.0].dfdv();
            let is_ltr = entries[i].node.is_left_to_right;
            let parent_idx = entries[i].parent_idx;

            if is_ltr {
                self.constraints[ci.0].lagrangian += dfdv;
                if parent_idx != usize::MAX {
                    let parent_ci = entries[parent_idx].node.constraint_to_eval;
                    if parent_ci != dummy_ci {
                        self.constraints[parent_ci.0].lagrangian +=
                            self.constraints[ci.0].lagrangian;
                    }
                }
            } else {
                self.constraints[ci.0].lagrangian =
                    -(self.constraints[ci.0].lagrangian + dfdv);
                if parent_idx != usize::MAX {
                    let parent_ci = entries[parent_idx].node.constraint_to_eval;
                    if parent_ci != dummy_ci {
                        self.constraints[parent_ci.0].lagrangian -=
                            self.constraints[ci.0].lagrangian;
                    }
                }
            }
        }
    }

    /// Process tree entries in post-order, computing Lagrangians and finding
    /// the path to `target_var`.
    fn process_post_order_with_path(
        &mut self,
        entries: &mut [TreeEntry],
        dummy_ci: ConIndex,
        target_var: VarIndex,
    ) -> Vec<ConstraintDirectionPair> {
        let mut result_path: Vec<ConstraintDirectionPair> = Vec::new();
        let mut found_target = false;

        for i in (0..entries.len()).rev() {
            let ci = entries[i].node.constraint_to_eval;
            if ci == dummy_ci {
                continue;
            }

            let vi = entries[i].node.variable_to_eval;
            let dfdv = self.variables[vi.0].dfdv();
            let is_ltr = entries[i].node.is_left_to_right;
            let parent_idx = entries[i].parent_idx;

            if is_ltr {
                self.constraints[ci.0].lagrangian += dfdv;
                if parent_idx != usize::MAX {
                    let parent_ci = entries[parent_idx].node.constraint_to_eval;
                    if parent_ci.0 != usize::MAX {
                        self.constraints[parent_ci.0].lagrangian +=
                            self.constraints[ci.0].lagrangian;
                    }
                }
            } else {
                self.constraints[ci.0].lagrangian =
                    -(self.constraints[ci.0].lagrangian + dfdv);
                if parent_idx != usize::MAX {
                    let parent_ci = entries[parent_idx].node.constraint_to_eval;
                    if parent_ci.0 != usize::MAX {
                        self.constraints[parent_ci.0].lagrangian -=
                            self.constraints[ci.0].lagrangian;
                    }
                }
            }

            // Check if this node found the target
            if vi == target_var && !found_target {
                found_target = true;
                let mut cur = i;
                while cur != usize::MAX && entries[cur].parent_idx != usize::MAX {
                    let node = &entries[cur].node;
                    if node.constraint_to_eval.0 != usize::MAX {
                        result_path.push(ConstraintDirectionPair {
                            constraint: node.constraint_to_eval,
                            is_forward: node.is_left_to_right,
                        });
                    }
                    cur = entries[cur].parent_idx;
                }
            }
        }

        result_path
    }
}
