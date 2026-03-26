//! Nudging pipeline: separates parallel rectilinear edge segments.
//!
//! Takes routed paths and uses a projection solver to push overlapping
//! parallel segments apart while respecting obstacle boundaries.

pub mod axis_edge;
pub mod combinatorial_nudger;
pub mod free_space_finder;
pub mod linked_point;
pub mod linked_point_splitter;
pub mod longest_nudged_segment;
pub mod nudger;
pub mod path_edge;
pub mod path_merger;
pub mod path_refiner;
pub mod staircase_remover;

pub use nudger::nudge_paths;
