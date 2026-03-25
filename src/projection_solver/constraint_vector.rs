use super::constraint::ConIndex;

/// Partitioned constraint array. Active constraints at the front,
/// inactive at the back. Activate/deactivate is O(1) via swap.
pub struct ConstraintVector {
    positions: Vec<usize>,
    order: Vec<ConIndex>,
    first_active: usize,
    pub max_constraint_tree_depth: u32,
    pub number_of_unsatisfiable_constraints: u32,
}

impl ConstraintVector {
    pub fn new(count: usize) -> Self {
        let order: Vec<ConIndex> = (0..count).map(ConIndex).collect();
        let positions: Vec<usize> = (0..count).collect();
        Self {
            positions,
            order,
            first_active: count,
            max_constraint_tree_depth: 0,
            number_of_unsatisfiable_constraints: 0,
        }
    }

    pub fn active_count(&self) -> usize {
        self.order.len() - self.first_active
    }

    pub fn activate(&mut self, ci: ConIndex) {
        self.first_active -= 1;
        self.swap_to(ci, self.first_active);
    }

    pub fn deactivate(&mut self, ci: ConIndex) {
        self.swap_to(ci, self.first_active);
        self.first_active += 1;
    }

    fn swap_to(&mut self, ci: ConIndex, target_pos: usize) {
        let current_pos = self.positions[ci.0];
        if current_pos == target_pos {
            return;
        }
        let other_ci = self.order[target_pos];
        self.order[current_pos] = other_ci;
        self.order[target_pos] = ci;
        self.positions[ci.0] = target_pos;
        self.positions[other_ci.0] = current_pos;
    }

    pub fn active_constraints(&self) -> &[ConIndex] {
        &self.order[self.first_active..]
    }

    pub fn all_constraints(&self) -> &[ConIndex] {
        &self.order
    }
}
