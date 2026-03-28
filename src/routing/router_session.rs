//! Shared routing session data.
//!
//! `RouterSession` owns all state that multiple routing components need to access:
//! the visibility graph, the obstacle tree, and the H/V scan segment trees built
//! during graph generation. Other structs (PortManager, TransientGraphUtility)
//! receive `&mut RouterSession` as a method parameter rather than storing a reference,
//! avoiding lifetime annotations entirely.
//!
//! Mirrors the shared-ownership chain that C# achieves through the `graphGenerator`
//! reference held by `PortManager` and `TransientGraphUtility`.

use crate::routing::obstacle_tree::ObstacleTree;
use crate::routing::scan_segment::ScanSegmentTree;
use crate::visibility::graph::VisibilityGraph;

/// Owns all data shared between PortManager, VisibilityGraphGenerator, and TransientGraphUtility.
pub struct RouterSession {
    pub vis_graph: VisibilityGraph,
    pub obstacle_tree: ObstacleTree,
    /// Scan segment tree from the horizontal sweep pass (used for port splice lookups).
    pub h_scan_segments: ScanSegmentTree,
    /// Scan segment tree from the vertical sweep pass (used for port splice lookups).
    pub v_scan_segments: ScanSegmentTree,
    pub padding: f64,
}
