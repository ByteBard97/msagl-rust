use super::constraint::ConIndex;

/// Maximum number of constraints in the cache.
/// Must be >= 2 for Insert() low-violation logic.
const MAX_CACHE_SIZE: usize = 20;

/// Violation cache for reducing full constraint scans.
/// Matches C#'s ViolationCache faithfully.
pub struct ViolationCache {
    constraints: Vec<Option<ConIndex>>,
    num_constraints: usize,
    low_violation: f64,
}

impl ViolationCache {
    pub fn new() -> Self {
        Self {
            constraints: vec![None; MAX_CACHE_SIZE],
            num_constraints: 0,
            low_violation: 0.0,
        }
    }

    pub fn clear(&mut self) {
        self.low_violation = 0.0;
        self.num_constraints = 0;
    }

    #[inline]
    pub fn low_violation(&self) -> f64 {
        self.low_violation
    }

    #[inline]
    pub fn is_full(&self) -> bool {
        self.num_constraints == MAX_CACHE_SIZE
    }

    /// Filter the cache: remove constraints matching the predicate.
    /// Returns true if the cache was non-empty before filtering.
    /// Matches C#'s FilterBlock exactly.
    pub fn filter_block(
        &mut self,
        should_remove: impl Fn(ConIndex) -> bool,
        get_violation: impl Fn(ConIndex) -> f64,
    ) -> bool {
        self.low_violation = f64::MAX;
        let was_non_empty = self.num_constraints > 0;

        // Iterate in reverse to support swap-removal.
        let mut ii = self.num_constraints as isize - 1;
        while ii >= 0 {
            let idx = ii as usize;
            let ci = self.constraints[idx].unwrap();

            if should_remove(ci) {
                // Swap in the last element before decrementing count.
                if idx < self.num_constraints - 1 {
                    self.constraints[idx] = self.constraints[self.num_constraints - 1];
                }
                self.num_constraints -= 1;
            } else {
                let violation = get_violation(ci);
                if violation < self.low_violation {
                    self.low_violation = violation;
                }
            }
            ii -= 1;
        }

        if self.num_constraints == 0 {
            self.low_violation = 0.0;
        }

        was_non_empty
    }

    /// Find the constraint with the highest violation greater than target.
    /// Returns the constraint index or None.
    pub fn find_if_greater(
        &self,
        mut target_violation: f64,
        get_violation: &impl Fn(ConIndex) -> f64,
    ) -> Option<ConIndex> {
        let mut best: Option<ConIndex> = None;
        for ii in 0..self.num_constraints {
            let ci = self.constraints[ii].unwrap();
            let violation = get_violation(ci);
            if violation > target_violation {
                target_violation = violation;
                best = Some(ci);
            }
        }
        best
    }

    /// Insert a constraint into the cache.
    /// Caller must ensure `insert_violation > self.low_violation`.
    pub fn insert(
        &mut self,
        ci: ConIndex,
        insert_violation: f64,
        get_violation: &impl Fn(ConIndex) -> f64,
    ) {
        let mut index_of_lowest = 0;
        let mut low_vio = insert_violation;
        let mut next_low_vio = insert_violation;

        for ii in 0..self.num_constraints {
            let cache_ci = self.constraints[ii].unwrap();
            let cache_vio = get_violation(cache_ci);
            if cache_vio < low_vio {
                next_low_vio = low_vio;
                index_of_lowest = ii;
                low_vio = cache_vio;
            } else if cache_vio < next_low_vio {
                next_low_vio = cache_vio;
            }
        }

        if !self.is_full() {
            // Add to the cache.
            self.constraints[self.num_constraints] = Some(ci);
            self.num_constraints += 1;
            if self.is_full() {
                self.low_violation = low_vio;
            }
        } else {
            // Replace the lowest violation in the cache.
            self.constraints[index_of_lowest] = Some(ci);
            self.low_violation = next_low_vio;
        }
    }
}

impl Default for ViolationCache {
    fn default() -> Self {
        Self::new()
    }
}
