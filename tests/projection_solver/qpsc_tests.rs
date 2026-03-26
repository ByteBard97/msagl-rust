use msagl_rust::projection_solver::parameters::Parameters;
use msagl_rust::projection_solver::qpsc::{dot, Qpsc};
use msagl_rust::projection_solver::variable::{NeighborAndWeight, VarIndex, Variable};

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
