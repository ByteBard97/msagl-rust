//! Nudging pipeline: separates parallel rectilinear edge segments.
//!
//! Takes routed paths and uses a projection solver to push overlapping
//! parallel segments apart while respecting obstacle boundaries.

mod axis_edge;
mod combinatorial_nudger;
mod free_space_finder;
mod longest_nudged_segment;
mod nudger;
mod path_edge;
mod path_refiner;
mod staircase_remover;

pub use nudger::nudge_paths;
