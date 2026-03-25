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
