use msagl_rust::projection_solver::variable::{Variable, VarIndex};
use msagl_rust::projection_solver::constraint::{Constraint, ConIndex};
use msagl_rust::projection_solver::block::{Block, BlockIndex};
use msagl_rust::projection_solver::parameters::Parameters;
use msagl_rust::projection_solver::solution::Solution;
use msagl_rust::projection_solver::constraint_vector::ConstraintVector;

#[test]
fn variable_default_fields() {
    let v = Variable::new(VarIndex(0), 5.0, 1.0, 1.0);
    assert_eq!(v.desired_pos, 5.0);
    assert_eq!(v.weight, 1.0);
    assert_eq!(v.scale, 1.0);
    assert_eq!(v.actual_pos, 5.0);
    assert_eq!(v.offset_in_block, 0.0);
    assert!(v.left_constraints.is_empty());
    assert!(v.right_constraints.is_empty());
}

#[test]
fn variable_dfdv_at_desired_pos() {
    let v = Variable::new(VarIndex(0), 5.0, 1.0, 1.0);
    assert_eq!(v.dfdv(), 0.0);
}

#[test]
fn variable_dfdv_displaced() {
    let mut v = Variable::new(VarIndex(0), 5.0, 2.0, 1.0);
    v.actual_pos = 7.0;
    assert_eq!(v.dfdv(), 8.0);
}

#[test]
fn constraint_creation() {
    let c = Constraint::new(ConIndex(0), VarIndex(0), VarIndex(1), 3.0, false);
    assert_eq!(c.gap, 3.0);
    assert!(!c.is_equality);
    assert!(!c.is_active);
    assert_eq!(c.lagrangian, 0.0);
}

#[test]
fn constraint_violation_calculation() {
    let c = Constraint::new(ConIndex(0), VarIndex(0), VarIndex(1), 3.0, false);
    // left=2, scale=1, right=4, scale=1: violation = 2 + 3 - 4 = 1 (violated)
    assert_eq!(c.violation(2.0, 1.0, 4.0, 1.0), 1.0);
    // left=0, scale=1, right=5, scale=1: violation = 0 + 3 - 5 = -2 (satisfied)
    assert_eq!(c.violation(0.0, 1.0, 5.0, 1.0), -2.0);
}

#[test]
fn default_parameters() {
    let p = Parameters::default();
    assert_eq!(p.gap_tolerance, 1e-4);
    assert_eq!(p.qpsc_convergence_epsilon, 1e-5);
    assert_eq!(p.qpsc_convergence_quotient, 1e-6);
}

#[test]
fn solution_default() {
    let s = Solution::new();
    assert_eq!(s.number_of_unsatisfiable_constraints, 0);
    assert!(!s.execution_limit_exceeded());
}

#[test]
fn block_creation() {
    let block = Block::new(BlockIndex(0), VarIndex(0), 5.0, 1.0, 1.0);
    assert_eq!(block.variables.len(), 1);
    assert_eq!(block.reference_pos, 5.0);
    assert_eq!(block.scale, 1.0);
}

#[test]
fn block_compute_variable_pos() {
    let block = Block::new(BlockIndex(0), VarIndex(0), 5.0, 1.0, 1.0);
    // pos = (scale * ref + offset) / var_scale = (1 * 5 + 0) / 1 = 5
    assert_eq!(block.compute_variable_pos(0.0, 1.0), 5.0);
    // with offset: (1 * 5 + 3) / 1 = 8
    assert_eq!(block.compute_variable_pos(3.0, 1.0), 8.0);
}

#[test]
fn constraint_vector_activate_deactivate() {
    let mut cv = ConstraintVector::new(3);
    assert_eq!(cv.active_count(), 0);
    cv.activate(ConIndex(0));
    assert_eq!(cv.active_count(), 1);
    cv.activate(ConIndex(1));
    assert_eq!(cv.active_count(), 2);
    cv.deactivate(ConIndex(0));
    assert_eq!(cv.active_count(), 1);
}

#[test]
fn constraint_vector_active_list() {
    let mut cv = ConstraintVector::new(5);
    cv.activate(ConIndex(2));
    cv.activate(ConIndex(4));
    let active = cv.active_constraints();
    assert_eq!(active.len(), 2);
    assert!(active.contains(&ConIndex(2)));
    assert!(active.contains(&ConIndex(4)));
}

#[test]
fn constraint_vector_round_trip() {
    let mut cv = ConstraintVector::new(3);
    cv.activate(ConIndex(0));
    cv.activate(ConIndex(1));
    cv.activate(ConIndex(2));
    assert_eq!(cv.active_count(), 3);
    cv.deactivate(ConIndex(1));
    assert_eq!(cv.active_count(), 2);
    cv.activate(ConIndex(1));
    assert_eq!(cv.active_count(), 3);
}
