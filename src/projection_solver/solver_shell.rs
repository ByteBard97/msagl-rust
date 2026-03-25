use super::constraint::ConIndex;
use super::parameters::Parameters;
use super::solution::Solution;
use super::solver::Solver;
use super::variable::VarIndex;

/// High-level ID-based wrapper around Solver.
///
/// Callers work with `usize` IDs rather than `VarIndex` internals.
/// IDs can be assigned explicitly via `add_variable(Some(id), ...)` or
/// auto-assigned sequentially when `None` is passed.
pub struct SolverShell {
    solver: Solver,
    /// Maps user-facing ID → internal VarIndex.
    /// Slots for IDs that have not been assigned hold `VarIndex(usize::MAX)`.
    var_map: Vec<VarIndex>,
    parameters: Parameters,
}

impl SolverShell {
    pub fn new() -> Self {
        Self {
            solver: Solver::new(),
            var_map: Vec::new(),
            parameters: Parameters::default(),
        }
    }

    /// Add a variable. Returns the user ID.
    ///
    /// If `id` is `Some(n)`, the variable is registered under that ID.
    /// If `id` is `None`, the next available sequential ID is used.
    pub fn add_variable(
        &mut self,
        id: Option<usize>,
        desired_pos: f64,
        weight: f64,
        scale: f64,
    ) -> usize {
        let vi = self.solver.add_variable(desired_pos, weight, scale);
        let user_id = id.unwrap_or(self.var_map.len());
        // Extend the map with sentinel values to cover user_id
        while self.var_map.len() <= user_id {
            self.var_map.push(VarIndex(usize::MAX));
        }
        self.var_map[user_id] = vi;
        user_id
    }

    /// Add an inequality constraint: `var[left] + gap <= var[right]`.
    pub fn add_left_right_constraint(&mut self, left: usize, right: usize, gap: f64) -> ConIndex {
        self.solver
            .add_constraint(self.var_map[left], self.var_map[right], gap, false)
    }

    /// Add an equality constraint: `var[left] + gap == var[right]`.
    pub fn add_equality_constraint(&mut self, left: usize, right: usize, gap: f64) -> ConIndex {
        self.solver
            .add_constraint(self.var_map[left], self.var_map[right], gap, true)
    }

    /// Add a goal that minimises the squared distance between two variables.
    pub fn add_goal_two_variables_are_close(&mut self, id1: usize, id2: usize, weight: f64) {
        self.solver
            .add_neighbor_pair(self.var_map[id1], self.var_map[id2], weight);
    }

    /// Replace the solver parameters used by `solve()`.
    pub fn set_parameters(&mut self, params: Parameters) {
        self.parameters = params;
    }

    /// Run the solver and return the solution statistics.
    pub fn solve(&mut self) -> Solution {
        self.solver.solve(Some(self.parameters.clone()))
    }

    /// Return the solved position for the variable registered under `id`.
    pub fn get_variable_resolved_position(&self, id: usize) -> f64 {
        self.solver.variable(self.var_map[id]).actual_pos
    }
}

impl Default for SolverShell {
    fn default() -> Self {
        Self::new()
    }
}
