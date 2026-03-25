//! QPSC (Quadratic Programming for Separation Constraints) gradient projection.
//!
//! Drives the gradient-projection portion of the projection solver when variables
//! have neighbor relationships. Solves goal functions of the form:
//!     f(X) = Sum w_i*(x_i - d_i)^2 + Sum w_ij*(x_i - x_j)^2
//!
//! Follows the IPSep-CoLa paper with diagonal scaling from
//! "Constrained Stress Majorization Using Diagonally Scaled Gradient Projection."

use super::parameters::Parameters;
use super::variable::Variable;

/// A nonzero entry in a sparse row of the Hessian matrix Q.
#[derive(Clone, Debug)]
struct MatrixCell {
    /// Column index (variable ordinal).
    column: usize,
    /// Cell value: 2*(summed weight), then possibly scaled.
    value: f64,
}

/// Saved variable state for restoration after QPSC completes.
#[derive(Clone, Debug)]
struct QpscVar {
    /// Index into the variables array.
    var_index: usize,
    /// Original weight before QPSC zeroed it to 1.
    orig_weight: f64,
    /// Original scale before QPSC diagonal scaling.
    orig_scale: f64,
    /// Original desired position.
    orig_desired_pos: f64,
}

/// The QPSC gradient projection state machine.
///
/// Lifecycle (driven by Solver):
///   1. `new()` + `add_variable()` per variable + `variables_complete()`
///   2. Per iteration: `pre_project()` -> split/project -> `post_project()`
///   3. `qpsc_complete()` to restore original variable state
pub struct Qpsc {
    /// Sparse Hessian Q (one row per variable, only nonzero entries).
    matrix_q: Vec<Vec<MatrixCell>>,
    /// Linear term b: -2*(w_i * d_i) per variable.
    vector_b: Vec<f64>,
    /// Gradient vector: g = Qy + b.
    gradient: Vec<f64>,
    /// Q*g product for alpha step-size computation.
    vector_qg: Vec<f64>,
    /// Previous positions (y-hat from the paper).
    prev_y: Vec<f64>,
    /// Current positions (y-bar from the paper).
    curr_y: Vec<f64>,
    /// Saved variable state for restoration.
    saved_vars: Vec<QpscVar>,
    /// Scratch row for building matrix during add_variable.
    scratch_row: Vec<MatrixCell>,
    /// Previous function value for convergence testing.
    prev_function_value: f64,
    /// Number of variables.
    num_vars: usize,
    /// Whether to apply diagonal scaling.
    use_scaling: bool,
    /// Whether this is the first call to pre_project.
    is_first_project: bool,
    /// Convergence parameters.
    convergence_epsilon: f64,
    convergence_quotient: f64,
}

impl Qpsc {
    /// Create a new QPSC instance for the given number of variables.
    pub fn new(params: &Parameters, num_vars: usize) -> Self {
        Self {
            matrix_q: vec![Vec::new(); num_vars],
            vector_b: vec![0.0; num_vars],
            gradient: vec![0.0; num_vars],
            vector_qg: vec![0.0; num_vars],
            prev_y: vec![0.0; num_vars],
            curr_y: vec![0.0; num_vars],
            saved_vars: Vec::with_capacity(num_vars),
            scratch_row: Vec::new(),
            prev_function_value: f64::MAX,
            num_vars,
            use_scaling: params.advanced.scale_in_qpsc,
            is_first_project: true,
            convergence_epsilon: params.qpsc_convergence_epsilon,
            convergence_quotient: params.qpsc_convergence_quotient,
        }
    }

    /// Register a variable and build its row in the sparse Hessian matrix Q.
    ///
    /// Uses `prev_y` as scratch storage for accumulating per-column weights,
    /// then compresses the nonzero entries into a sparse row.
    pub fn add_variable(&mut self, var_idx: usize, variables: &[Variable]) {
        let var = &variables[var_idx];
        let ordinal = var.ordinal as usize;

        // b[i] = -2 * w_i * d_i
        self.vector_b[ordinal] = -2.0 * var.weight * var.desired_pos;

        // Save original state
        self.saved_vars.push(QpscVar {
            var_index: var_idx,
            orig_weight: var.weight,
            orig_scale: var.scale,
            orig_desired_pos: var.desired_pos,
        });

        // Use prev_y as scratch to accumulate weights per column.
        // Diagonal: w_i + sum(w_ij)
        self.prev_y[ordinal] = var.weight;

        for nw in &var.neighbors {
            let neighbor_ordinal = variables[nw.neighbor.0].ordinal as usize;
            // Diagonal: add neighbor weight
            self.prev_y[ordinal] += nw.weight;
            // Off-diagonal: subtract neighbor weight
            self.prev_y[neighbor_ordinal] -= nw.weight;
        }

        // Compress nonzero entries into a sparse row, multiply by 2 for partial derivative.
        self.scratch_row.clear();
        for col in 0..self.num_vars {
            if self.prev_y[col] != 0.0 {
                self.scratch_row.push(MatrixCell {
                    column: col,
                    value: self.prev_y[col] * 2.0,
                });
                self.prev_y[col] = 0.0; // Reset scratch
            }
        }

        self.matrix_q[ordinal] = self.scratch_row.clone();
    }

    /// Finalize setup after all variables have been added.
    ///
    /// If scaling is enabled, computes diagonal scale factors and transforms
    /// Q to S*Q*S form. Sets each variable's desired_pos to its current actual_pos
    /// and weight to 1.0 for the QPSC iterations.
    pub fn variables_complete(&self, variables: &mut [Variable]) {
        for qvar in &self.saved_vars {
            let var = &mut variables[qvar.var_index];
            let ordinal = var.ordinal as usize;

            // Find diagonal entry
            if let Some(diag) = self.matrix_q[ordinal]
                .iter()
                .find(|c| c.column == ordinal)
            {
                if self.use_scaling {
                    var.scale = 1.0 / (diag.value.abs()).sqrt();
                    if !var.scale.is_finite() {
                        var.scale = 1.0;
                    }
                }
            }

            var.desired_pos = var.actual_pos;
            var.weight = 1.0;
        }
    }

    /// Apply scaling to b vector and Q matrix after variables_complete set scales.
    /// Must be called after variables_complete.
    pub fn apply_scaling(&mut self, variables: &[Variable]) {
        if !self.use_scaling {
            // Even without scaling, record current positions
            for qvar in &self.saved_vars {
                let var = &variables[qvar.var_index];
                let ordinal = var.ordinal as usize;
                self.curr_y[ordinal] = var.actual_pos;
                // Scale b by variable scale (which is 1.0 when not scaling)
                self.vector_b[ordinal] *= var.scale;
            }
            return;
        }

        // Scale b' = S*b and record positions
        for qvar in &self.saved_vars {
            let var = &variables[qvar.var_index];
            let ordinal = var.ordinal as usize;
            self.vector_b[ordinal] *= var.scale;
            self.curr_y[ordinal] = var.actual_pos;
        }

        // Build a scale lookup for all ordinals
        let mut scales = vec![1.0_f64; self.num_vars];
        for qvar in &self.saved_vars {
            let var = &variables[qvar.var_index];
            scales[var.ordinal as usize] = var.scale;
        }

        // Transform Q to S*Q*S:
        //   diagonal -> 1.0
        //   off-diag -> Q[i][j] * scale[i] * scale[j]
        for row_num in 0..self.matrix_q.len() {
            for cell in &mut self.matrix_q[row_num] {
                if cell.column == row_num {
                    cell.value = 1.0;
                } else {
                    cell.value *= scales[row_num] * scales[cell.column];
                }
            }
        }
    }

    /// Before Project: compute gradient, check convergence, compute alpha step.
    /// Updates variables' desired positions. Returns false if converged.
    pub fn pre_project(&mut self, variables: &mut [Variable]) -> bool {
        if self.is_first_project {
            // Equality merging may have moved variables; update curr_y.
            for qvar in &self.saved_vars {
                let var = &variables[qvar.var_index];
                self.curr_y[var.ordinal as usize] = var.actual_pos;
            }
        }

        // g = Q * curr_y
        matrix_vector_multiply(&self.matrix_q, &self.curr_y, &mut self.gradient);

        // Check convergence
        if self.has_converged() {
            return false;
        }

        // g = Q*y + b
        for i in 0..self.num_vars {
            self.gradient[i] += self.vector_b[i];
        }

        // alpha = g'g / g'Qg
        let alpha_num = dot(&self.gradient, &self.gradient);
        let mut alpha_den = 0.0;
        if alpha_num != 0.0 {
            matrix_vector_multiply(&self.matrix_q, &self.gradient, &mut self.vector_qg);
            alpha_den = dot(&self.vector_qg, &self.gradient);
        }

        if alpha_den == 0.0 {
            return false;
        }

        let alpha = alpha_num / alpha_den;

        // prev_y = curr_y (y-hat)
        self.prev_y.copy_from_slice(&self.curr_y);

        // new desired = prev_y - alpha * gradient
        // (use curr_y as temp since it's not needed until post_project)
        for i in 0..self.num_vars {
            self.curr_y[i] = self.prev_y[i] - alpha * self.gradient[i];
        }

        // Update variables' desired positions
        for qvar in &self.saved_vars {
            let var = &mut variables[qvar.var_index];
            var.desired_pos = self.curr_y[var.ordinal as usize];
        }

        true
    }

    /// After Project: compute beta step, update current positions.
    /// Returns false if beta <= 0 (no progress).
    pub fn post_project(&mut self, variables: &[Variable]) -> bool {
        // Update curr_y from projected positions
        for qvar in &self.saved_vars {
            let var = &variables[qvar.var_index];
            self.curr_y[var.ordinal as usize] = var.actual_pos;
        }

        // p = prev_y - curr_y (reuse curr_y as p-vector temporarily)
        for i in 0..self.num_vars {
            self.curr_y[i] = self.prev_y[i] - self.curr_y[i];
        }

        // beta = g'p / p'Qp
        let beta_num = dot(&self.gradient, &self.curr_y);
        let mut beta = 0.0;

        if beta_num != 0.0 {
            // Qp (reuse vector_qg)
            matrix_vector_multiply(&self.matrix_q, &self.curr_y, &mut self.vector_qg);
            let beta_den = dot(&self.vector_qg, &self.curr_y);
            beta = if beta_den == 0.0 {
                1.0
            } else {
                beta_num / beta_den
            };
            beta = beta.clamp(0.0, 1.0);
        }

        // curr_y = prev_y - beta * p
        for i in 0..self.num_vars {
            self.curr_y[i] = self.prev_y[i] - beta * self.curr_y[i];
        }

        self.is_first_project = false;
        beta > 0.0
    }

    /// Restore original variable state after QPSC is done.
    /// Returns the final goal function value.
    pub fn qpsc_complete(&self, variables: &mut [Variable]) -> f64 {
        for qvar in &self.saved_vars {
            let var = &mut variables[qvar.var_index];
            var.weight = qvar.orig_weight;
            var.desired_pos = qvar.orig_desired_pos;
            if self.use_scaling {
                // Unscale actual position
                var.actual_pos *= var.scale;
                var.scale = qvar.orig_scale;
            }
        }
        self.prev_function_value
    }

    // ===== Private helpers =====

    /// Check if the goal function has converged.
    fn has_converged(&mut self) -> bool {
        // f(x) = (y'Qy)/2 + b'y
        let current_value = self.get_function_value();

        let mut converged = false;
        if !self.is_first_project {
            let diff = self.prev_function_value - current_value;
            let mut quotient = 0.0;
            if diff != 0.0 {
                let divisor = if self.prev_function_value != 0.0 {
                    self.prev_function_value
                } else {
                    current_value
                };
                quotient = (diff / divisor).abs();
            }

            if diff.abs() < self.convergence_epsilon
                || quotient.abs() < self.convergence_quotient
            {
                converged = true;
            }
        }

        self.prev_function_value = current_value;
        converged
    }

    /// Compute f(x) = (y'Qy)/2 + b'y using the gradient vector (which holds Qy).
    fn get_function_value(&self) -> f64 {
        let qy_dot_y = dot(&self.gradient, &self.curr_y) / 2.0;
        qy_dot_y + dot(&self.vector_b, &self.curr_y)
    }
}

/// Sparse matrix-vector multiply: result[i] = sum(matrix[i][j] * rhs[j]).
fn matrix_vector_multiply(matrix: &[Vec<MatrixCell>], rhs: &[f64], result: &mut [f64]) {
    for (row_idx, row) in matrix.iter().enumerate() {
        let mut sum = 0.0;
        for cell in row {
            sum += cell.value * rhs[cell.column];
        }
        result[row_idx] = sum;
    }
}

/// Dot product of two vectors.
#[inline]
fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b.iter()).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::projection_solver::variable::{Variable, VarIndex};

    fn make_var(index: usize, desired: f64, weight: f64) -> Variable {
        let mut v = Variable::new(VarIndex(index), desired, weight, 1.0);
        v.ordinal = index as u32;
        v
    }

    #[test]
    fn qpsc_matrix_build_diagonal_only() {
        let params = Parameters::default();
        let mut qpsc = Qpsc::new(&params, 2);
        let vars = vec![make_var(0, 0.0, 1.0), make_var(1, 10.0, 1.0)];
        // No neighbors, so matrix is diagonal: Q[i][i] = 2*w_i
        qpsc.add_variable(0, &vars);
        qpsc.add_variable(1, &vars);
        assert_eq!(qpsc.matrix_q[0].len(), 1); // only diagonal
        assert_eq!(qpsc.matrix_q[0][0].column, 0);
        assert!((qpsc.matrix_q[0][0].value - 2.0).abs() < 1e-10);
    }

    #[test]
    fn qpsc_matrix_build_with_neighbors() {
        use crate::projection_solver::variable::NeighborAndWeight;
        let params = Parameters::default();
        let mut qpsc = Qpsc::new(&params, 2);
        let mut vars = vec![make_var(0, 0.0, 1.0), make_var(1, 10.0, 1.0)];
        vars[0].neighbors.push(NeighborAndWeight {
            neighbor: VarIndex(1),
            weight: 2.0,
        });
        vars[1].neighbors.push(NeighborAndWeight {
            neighbor: VarIndex(0),
            weight: 2.0,
        });

        qpsc.add_variable(0, &vars);
        qpsc.add_variable(1, &vars);

        // Row 0: diag = 2*(1+2)=6, off-diag = 2*(-2)=-4
        assert_eq!(qpsc.matrix_q[0].len(), 2);
        let diag = qpsc.matrix_q[0].iter().find(|c| c.column == 0).unwrap();
        let off = qpsc.matrix_q[0].iter().find(|c| c.column == 1).unwrap();
        assert!((diag.value - 6.0).abs() < 1e-10);
        assert!((off.value - (-4.0)).abs() < 1e-10);
    }

    #[test]
    fn dot_product() {
        assert!((dot(&[1.0, 2.0, 3.0], &[4.0, 5.0, 6.0]) - 32.0).abs() < 1e-10);
    }
}
