use super::parameters::Parameters;
use super::solver_shell::SolverShell;

/// One-dimensional projection solver with optional per-variable bounds.
///
/// Used by the nudging pipeline.  Bounds are implemented as high-weight
/// anchor variables constrained to the boundary position.
pub struct UniformOneDimensionalSolver {
    shell: SolverShell,
    /// Number of user-facing variables (anchor variables are not counted).
    num_user_vars: usize,
}

impl UniformOneDimensionalSolver {
    pub fn new() -> Self {
        Self {
            shell: SolverShell::new(),
            num_user_vars: 0,
        }
    }

    /// Add a variable and return its ID.
    pub fn add_variable(&mut self, desired_pos: f64, weight: f64, scale: f64) -> usize {
        let id = self.shell.add_variable(None, desired_pos, weight, scale);
        self.num_user_vars += 1;
        id
    }

    /// Add a constraint: `var[left] + gap <= var[right]`.
    pub fn add_constraint(&mut self, left: usize, right: usize, gap: f64) {
        self.shell.add_left_right_constraint(left, right, gap);
    }

    /// Constrain `var[var_id] <= bound` by anchoring a high-weight variable at `bound`.
    pub fn set_upper_bound(&mut self, var_id: usize, bound: f64) {
        let anchor = self.shell.add_variable(None, bound, 1e8, 1.0);
        self.shell.add_left_right_constraint(var_id, anchor, 0.0);
    }

    /// Constrain `var[var_id] >= bound` by anchoring a high-weight variable at `bound`.
    pub fn set_lower_bound(&mut self, var_id: usize, bound: f64) {
        let anchor = self.shell.add_variable(None, bound, 1e8, 1.0);
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

    /// Solve and return the resolved positions for user variables (IDs 0..num_user_vars).
    pub fn solve(&mut self) -> Vec<f64> {
        self.shell.solve();
        (0..self.num_user_vars)
            .map(|id| self.shell.get_variable_resolved_position(id))
            .collect()
    }

    /// Return the resolved position for a specific user variable.
    pub fn get_position(&self, id: usize) -> f64 {
        self.shell.get_variable_resolved_position(id)
    }
}

impl Default for UniformOneDimensionalSolver {
    fn default() -> Self {
        Self::new()
    }
}
