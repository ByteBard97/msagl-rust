use msagl_rust::projection_solver::variable::{Variable, VarIndex};
use msagl_rust::projection_solver::constraint::{Constraint, ConIndex};
use msagl_rust::projection_solver::block::{Block, BlockIndex};
use msagl_rust::projection_solver::parameters::Parameters;
use msagl_rust::projection_solver::solution::Solution;
use msagl_rust::projection_solver::constraint_vector::ConstraintVector;
use msagl_rust::projection_solver::violation_cache::ViolationCache;

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

#[test]
fn violation_cache_insert_and_find() {
    let mut cache = ViolationCache::new();
    cache.insert(ConIndex(5), 10.0);
    cache.insert(ConIndex(3), 20.0);
    cache.insert(ConIndex(7), 5.0);
    let result = cache.find_if_greater(15.0);
    assert!(result.is_some());
    assert_eq!(result.unwrap().0, ConIndex(3));
}

#[test]
fn violation_cache_clear() {
    let mut cache = ViolationCache::new();
    cache.insert(ConIndex(0), 100.0);
    cache.clear();
    assert_eq!(cache.find_if_greater(0.0), None);
}

#[test]
fn violation_cache_capacity_limit() {
    let mut cache = ViolationCache::new();
    for i in 0..25 {
        cache.insert(ConIndex(i), i as f64);
    }
    // Top 20 should remain; lowest should be >= 5
    assert!(cache.find_if_greater(4.0).is_some());
}

#[test]
fn violation_cache_filter() {
    let mut cache = ViolationCache::new();
    cache.insert(ConIndex(0), 10.0);
    cache.insert(ConIndex(1), 20.0);
    cache.insert(ConIndex(2), 30.0);
    // Filter out ConIndex 1
    let remaining = cache.filter_block(|ci| ci == ConIndex(1));
    assert!(remaining);
    assert_eq!(cache.find_if_greater(0.0).unwrap().0, ConIndex(2));
}

// ===== Solver tests =====

use msagl_rust::projection_solver::solver::Solver;

#[test]
fn two_vars_satisfied_constraint() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(10.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 3.0, false);
    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!(pos1 - pos0 >= 3.0 - 1e-4);
    assert!((pos0 - 0.0).abs() < 1e-4);
    assert!((pos1 - 10.0).abs() < 1e-4);
}

#[test]
fn two_vars_violated_constraint() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(5.0, 1.0, 1.0);
    let v1 = solver.add_variable(6.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 3.0, false);
    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!(pos1 - pos0 >= 3.0 - 1e-4);
    assert!((pos0 - 4.0).abs() < 0.1);
    assert!((pos1 - 7.0).abs() < 0.1);
}

#[test]
fn three_vars_chain() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(0.0, 1.0, 1.0);
    let v2 = solver.add_variable(0.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 5.0, false);
    solver.add_constraint(v1, v2, 5.0, false);
    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    let pos2 = solver.variable(v2).actual_pos;
    assert!(pos1 - pos0 >= 5.0 - 1e-4);
    assert!(pos2 - pos1 >= 5.0 - 1e-4);
}

#[test]
fn equality_constraint() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(10.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 5.0, true);
    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!((pos1 - pos0 - 5.0).abs() < 1e-4);
}

#[test]
fn weighted_variables() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 100.0, 1.0);
    let v1 = solver.add_variable(2.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 5.0, false);
    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!(pos1 - pos0 >= 5.0 - 1e-4);
    assert!(pos0.abs() < 0.5); // Heavy var barely moves
}

// ===== QPSC tests =====

#[test]
fn solver_with_neighbors() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(10.0, 1.0, 1.0);
    let v2 = solver.add_variable(20.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 3.0, false);
    solver.add_constraint(v1, v2, 3.0, false);
    solver.add_neighbor_pair(v0, v2, 1.0);

    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    let pos2 = solver.variable(v2).actual_pos;
    // Constraints satisfied
    assert!(pos1 - pos0 >= 3.0 - 1e-3, "gap01={}", pos1 - pos0);
    assert!(pos2 - pos1 >= 3.0 - 1e-3, "gap12={}", pos2 - pos1);
    // Neighbor force pulls v0 and v2 closer than their desired 20-unit spread
    assert!(pos2 - pos0 < 20.0, "spread={}", pos2 - pos0);
}

#[test]
fn solver_force_qpsc_no_neighbors() {
    // ForceQpsc should work even without neighbors
    let mut solver = Solver::new();
    let v0 = solver.add_variable(5.0, 1.0, 1.0);
    let v1 = solver.add_variable(6.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 3.0, false);

    let mut params = Parameters::default();
    params.advanced.force_qpsc = true;
    let _solution = solver.solve(Some(params));
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    assert!(pos1 - pos0 >= 3.0 - 1e-3, "gap={}", pos1 - pos0);
}

#[test]
fn solver_neighbors_with_equality() {
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(10.0, 1.0, 1.0);
    let v2 = solver.add_variable(20.0, 1.0, 1.0);
    solver.add_constraint(v0, v1, 5.0, true); // equality
    solver.add_constraint(v1, v2, 5.0, false);
    solver.add_neighbor_pair(v0, v2, 1.0);

    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    let pos2 = solver.variable(v2).actual_pos;
    // Equality constraint: exactly 5.0 gap
    assert!((pos1 - pos0 - 5.0).abs() < 1e-3, "eq gap={}", pos1 - pos0);
    assert!(pos2 - pos1 >= 5.0 - 1e-3, "gap12={}", pos2 - pos1);
}

#[test]
fn solver_qpsc_no_constraints_neighbors_only() {
    // Neighbors but no constraints -- should still converge
    let mut solver = Solver::new();
    let v0 = solver.add_variable(0.0, 1.0, 1.0);
    let v1 = solver.add_variable(100.0, 1.0, 1.0);
    solver.add_neighbor_pair(v0, v1, 10.0);

    let _solution = solver.solve(None);
    let pos0 = solver.variable(v0).actual_pos;
    let pos1 = solver.variable(v1).actual_pos;
    // Neighbor force pulls them together; they should be closer than 100 apart
    assert!(
        (pos1 - pos0).abs() < 100.0,
        "spread={}",
        (pos1 - pos0).abs()
    );
}
