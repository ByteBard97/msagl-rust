use ordered_float::OrderedFloat;
use std::collections::BTreeMap;

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;

use super::scan_direction::ScanDirection;

/// Data for a lookahead reflection event.
#[derive(Debug, Clone)]
pub struct ReflectionSite {
    pub site: Point,
    pub initial_obstacle: usize,
    pub reflecting_obstacle: usize,
}

/// BTreeMap-based collection of reflection events for lookahead processing.
/// Ported from LookaheadScan.ts — keyed by scan-parallel coordinate of each reflection site.
pub struct LookaheadScan {
    scan_direction: ScanDirection,
    sites: BTreeMap<OrderedFloat<f64>, ReflectionSite>,
}

impl LookaheadScan {
    pub fn new(scan_direction: ScanDirection) -> Self {
        Self {
            scan_direction,
            sites: BTreeMap::new(),
        }
    }

    pub fn add_site(
        &mut self,
        site: Point,
        initial_obstacle: usize,
        reflecting_obstacle: usize,
    ) {
        let key = OrderedFloat(GeomConstants::round(self.scan_direction.coord(site)));
        self.sites.insert(
            key,
            ReflectionSite {
                site,
                initial_obstacle,
                reflecting_obstacle,
            },
        );
    }

    pub fn remove_site(&mut self, site: Point) {
        let key = OrderedFloat(GeomConstants::round(self.scan_direction.coord(site)));
        self.sites.remove(&key);
    }

    /// Find the first reflection site whose scan-parallel coordinate is in `[low, high]`.
    pub fn find_first_in_range(&self, low: f64, high: f64) -> Option<&ReflectionSite> {
        let low_key = OrderedFloat(GeomConstants::round(low));
        let high_key = OrderedFloat(GeomConstants::round(high));
        self.sites
            .range(low_key..=high_key)
            .next()
            .map(|(_, v)| v)
    }

    /// Find all reflection sites whose scan-parallel coordinate is in `[low, high]`.
    pub fn find_all_in_range(&self, low: f64, high: f64) -> Vec<&ReflectionSite> {
        let low_key = OrderedFloat(GeomConstants::round(low));
        let high_key = OrderedFloat(GeomConstants::round(high));
        self.sites
            .range(low_key..=high_key)
            .map(|(_, v)| v)
            .collect()
    }

    pub fn is_empty(&self) -> bool {
        self.sites.is_empty()
    }

    pub fn len(&self) -> usize {
        self.sites.len()
    }

    pub fn clear(&mut self) {
        self.sites.clear();
    }
}
