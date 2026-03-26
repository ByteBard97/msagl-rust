//! One-dimensional projection solver with optional per-variable bounds.
//!
//! C# ref: UniformOneDimensionalSolver.cs (115 lines)
//!
//! CRITICAL PERFORMANCE FIX: C# deduplicates bound anchor variables via
//! Dictionary<double, int>. Multiple variables sharing the same bound value
//! share ONE anchor. The old Rust code created a NEW anchor per set_lower_bound/
//! set_upper_bound call, tripling the variable count and causing 3-9x slowdown.

use std::collections::HashMap;
use ordered_float::OrderedFloat;
use super::parameters::Parameters;
use super::solver_shell::SolverShell;

/// Per-variable bound storage — C# UniformSolverVar.LowBound / UpperBound
struct VarBounds {
    low_bound: f64,
    upper_bound: f64,
    is_fixed: bool,
    position: f64,
    ideal_position: f64,
    weight: f64,
    scale: f64,
    width: f64,
}

/// One-dimensional projection solver with deduped bound variables.
///
/// C# ref: UniformOneDimensionalSolver.cs
///
/// MUST store bounds as fields on VarBounds (not create anchor variables immediately).
/// MUST deduplicate anchor variables by bound value in solve() via HashMap.
/// MUST call create_variables_for_bounds() at the start of solve().
pub struct UniformOneDimensionalSolver {
    shell: SolverShell,
    /// Per-variable bounds and metadata — C# varList
    vars: Vec<VarBounds>,
    /// Separation gap for constraints — C# varSeparation
    var_separation: f64,
}

impl UniformOneDimensionalSolver {
    pub fn new() -> Self {
        Self {
            shell: SolverShell::new(),
            vars: Vec::new(),
            var_separation: 0.0,
        }
    }

    /// Add a variable and return its ID.
    pub fn add_variable(&mut self, desired_pos: f64, weight: f64, scale: f64) -> usize {
        let id = self.vars.len();
        self.vars.push(VarBounds {
            low_bound: f64::NEG_INFINITY,
            upper_bound: f64::INFINITY,
            is_fixed: false,
            position: desired_pos,
            ideal_position: desired_pos,
            weight,
            scale,
            width: 0.0,
        });
        id
    }

    /// Add a fixed variable (high weight, stays at position).
    pub fn add_fixed_variable(&mut self, position: f64) -> usize {
        let id = self.vars.len();
        self.vars.push(VarBounds {
            low_bound: f64::NEG_INFINITY,
            upper_bound: f64::INFINITY,
            is_fixed: true,
            position,
            ideal_position: position,
            weight: 1e8,
            scale: 1.0,
            width: 0.0,
        });
        id
    }

    /// Add a constraint: var[left] + gap <= var[right].
    pub fn add_constraint(&mut self, left: usize, right: usize, gap: f64) {
        self.var_separation = gap; // C# stores this for width-adjusted constraints
        self.shell.add_left_right_constraint(left, right, gap);
    }

    /// Store upper bound — C# SetUpperBound: v.UpperBound = Math.Min(bound, v.UpperBound)
    /// MUST NOT create an anchor variable here. Deduplication happens in solve().
    pub fn set_upper_bound(&mut self, var_id: usize, bound: f64) {
        self.vars[var_id].upper_bound = self.vars[var_id].upper_bound.min(bound);
    }

    /// Store lower bound — C# SetLowBound: v.LowBound = Math.Max(bound, v.LowBound)
    /// MUST NOT create an anchor variable here. Deduplication happens in solve().
    pub fn set_lower_bound(&mut self, var_id: usize, bound: f64) {
        self.vars[var_id].low_bound = self.vars[var_id].low_bound.max(bound);
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

    /// Solve — C# SolveByRegularSolver (lines 57-92)
    ///
    /// MUST call create_variables_for_bounds() first to deduplicate anchors.
    /// MUST create shell variables after bounds are resolved.
    /// MUST use HashMap<OrderedFloat<f64>, usize> for bound deduplication.
    pub fn solve(&mut self) -> Vec<f64> {
        todo!("Port C# SolveByRegularSolver lines 57-92. \
               1. Call create_variables_for_bounds() to deduplicate. \
               2. Create shell variables for each var (fixed or with ideal position). \
               3. For non-fixed vars with bounds, add constraints to shared anchors. \
               4. Call shell.solve(). \
               5. Read back positions.")
    }

    /// Create deduplicated bound anchor variables — C# CreateVariablesForBounds (lines 96-110)
    ///
    /// MUST use HashMap<OrderedFloat<f64>, usize> to share anchors.
    /// C# uses Dictionary<double, int> boundsToInt.
    /// Multiple variables with the same bound value MUST share one anchor variable.
    fn create_variables_for_bounds(&mut self) -> HashMap<OrderedFloat<f64>, usize> {
        todo!("Port C# CreateVariablesForBounds + RegisterBoundVar. \
               Iterate vars, collect unique bound values, create one fixed \
               anchor variable per unique bound via shell.add_variable().")
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
