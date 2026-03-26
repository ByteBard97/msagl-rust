use std::collections::HashMap;

use ordered_float::OrderedFloat;

use super::parameters::Parameters;
use super::solver_shell::SolverShell;

/// One-dimensional projection solver with optional per-variable bounds.
///
/// Used by the nudging pipeline.  Bounds are implemented as high-weight
/// anchor variables constrained to the boundary position.  Bound values
/// are deduplicated: multiple variables that share the same bound position
/// reuse a single anchor variable, following the C# `UniformOneDimensionalSolver`.
pub struct UniformOneDimensionalSolver {
    shell: SolverShell,
    /// Actual shell IDs for user variables, in order of creation.
    /// Needed because anchor variables may be interleaved in the shell.
    user_var_ids: Vec<usize>,
    /// Maps bound position -> anchor shell variable ID for deduplication.
    /// Matching C#'s `boundsToInt` dictionary.
    bounds_to_anchor: HashMap<OrderedFloat<f64>, usize>,
}

impl UniformOneDimensionalSolver {
    pub fn new() -> Self {
        Self {
            shell: SolverShell::new(),
            user_var_ids: Vec::new(),
            bounds_to_anchor: HashMap::new(),
        }
    }

    /// Add a variable and return its ID.
    pub fn add_variable(&mut self, desired_pos: f64, weight: f64, scale: f64) -> usize {
        let id = self.shell.add_variable(None, desired_pos, weight, scale);
        self.user_var_ids.push(id);
        id
    }

    /// Add a constraint: `var[left] + gap <= var[right]`.
    pub fn add_constraint(&mut self, left: usize, right: usize, gap: f64) {
        self.shell.add_left_right_constraint(left, right, gap);
    }

    /// Constrain `var[var_id] <= bound` by anchoring a high-weight variable at `bound`.
    ///
    /// Anchor variables are deduplicated: if the same bound value has been used
    /// before (for any variable), the existing anchor is reused. This matches
    /// the C# `RegisterBoundVar` deduplication.
    pub fn set_upper_bound(&mut self, var_id: usize, bound: f64) {
        let anchor = self.get_or_create_anchor(bound);
        self.shell.add_left_right_constraint(var_id, anchor, 0.0);
    }

    /// Constrain `var[var_id] >= bound` by anchoring a high-weight variable at `bound`.
    ///
    /// Anchor variables are deduplicated: if the same bound value has been used
    /// before (for any variable), the existing anchor is reused. This matches
    /// the C# `RegisterBoundVar` deduplication.
    pub fn set_lower_bound(&mut self, var_id: usize, bound: f64) {
        let anchor = self.get_or_create_anchor(bound);
        self.shell.add_left_right_constraint(anchor, var_id, 0.0);
    }

    /// Add a goal that minimises the squared distance between two variables.
    pub fn add_goal_two_variables_are_close(&mut self, id1: usize, id2: usize, weight: f64) {
        self.shell
            .add_goal_two_variables_are_close(id1, id2, weight);
    }

    /// Replace the solver parameters.
    pub fn set_parameters(&mut self, params: Parameters) {
        self.shell.set_parameters(params);
    }

    /// Solve and return the resolved positions for user variables only.
    ///
    /// Returns one position per `add_variable` call, in creation order.
    /// Anchor variables (from bounds) are excluded from the result.
    pub fn solve(&mut self) -> Vec<f64> {
        self.shell.solve();
        self.user_var_ids
            .iter()
            .map(|&id| self.shell.get_variable_resolved_position(id))
            .collect()
    }

    /// Return the resolved position for a specific user variable.
    pub fn get_position(&self, id: usize) -> f64 {
        self.shell.get_variable_resolved_position(id)
    }

    /// Get or create an anchor variable for the given bound position.
    ///
    /// If an anchor at this exact position already exists, reuse it.
    /// Otherwise, create a new high-weight fixed variable.
    /// Matches C#'s `RegisterBoundVar` + `boundsToInt` dictionary.
    fn get_or_create_anchor(&mut self, bound: f64) -> usize {
        let key = OrderedFloat(bound);
        if let Some(&anchor_id) = self.bounds_to_anchor.get(&key) {
            anchor_id
        } else {
            let anchor_id = self.shell.add_variable(None, bound, 1e8, 1.0);
            self.bounds_to_anchor.insert(key, anchor_id);
            anchor_id
        }
    }
}

impl Default for UniformOneDimensionalSolver {
    fn default() -> Self {
        Self::new()
    }
}
