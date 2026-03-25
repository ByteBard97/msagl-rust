use super::constraint::ConIndex;

const MAX_CACHE_SIZE: usize = 20;

struct CacheEntry {
    constraint: ConIndex,
    violation: f64,
}

pub struct ViolationCache {
    entries: Vec<CacheEntry>,
    low_violation: f64,
}

impl ViolationCache {
    pub fn new() -> Self {
        Self {
            entries: Vec::with_capacity(MAX_CACHE_SIZE),
            low_violation: f64::MAX,
        }
    }

    pub fn clear(&mut self) {
        self.entries.clear();
        self.low_violation = f64::MAX;
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn insert(&mut self, ci: ConIndex, violation: f64) {
        if self.entries.len() < MAX_CACHE_SIZE {
            self.entries.push(CacheEntry { constraint: ci, violation });
            if violation < self.low_violation {
                self.low_violation = violation;
            }
        } else if violation > self.low_violation {
            let mut min_idx = 0;
            let mut min_val = self.entries[0].violation;
            for (i, e) in self.entries.iter().enumerate().skip(1) {
                if e.violation < min_val {
                    min_val = e.violation;
                    min_idx = i;
                }
            }
            self.entries[min_idx] = CacheEntry { constraint: ci, violation };
            self.low_violation = self.entries.iter().map(|e| e.violation).fold(f64::MAX, f64::min);
        }
    }

    pub fn find_if_greater(&self, target_violation: f64) -> Option<(ConIndex, f64)> {
        let mut best: Option<(ConIndex, f64)> = None;
        for e in &self.entries {
            if e.violation > target_violation {
                if best.is_none() || e.violation > best.unwrap().1 {
                    best = Some((e.constraint, e.violation));
                }
            }
        }
        best
    }

    pub fn filter_block(&mut self, is_in_block: impl Fn(ConIndex) -> bool) -> bool {
        self.entries.retain(|e| !is_in_block(e.constraint));
        if self.entries.is_empty() {
            self.low_violation = f64::MAX;
            false
        } else {
            self.low_violation = self.entries.iter().map(|e| e.violation).fold(f64::MAX, f64::min);
            true
        }
    }
}
