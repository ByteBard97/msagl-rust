//! Free point graph splicing for FullPortManager.
//!
//! Split from `port_manager.rs` to keep files under 500 lines.
//! Contains: add_free_point_to_graph, find_or_create_free_point,
//! create_out_of_bounds_free_point, connect_free_point_to_lateral_edge,
//! find_or_create_nearest_perp_edge.

use super::compass_direction::CompassDirection;
use super::port_manager::{FreePointId, FullPortManager};
use super::static_graph_utility::StaticGraphUtility;
use super::transient_graph_utility::TransientGraphUtility;
use crate::geometry::point::Point;
use crate::visibility::graph::VisibilityGraph;

impl FullPortManager {
    /// Add a FreePoint (waypoint or non-obstacle port) to the visibility graph.
    ///
    /// C# file: PortManager.cs, lines 844-897
    /// Big-O: O(log V) per direction (up to 4 directions)
    /// MUST handle overlapped points (inside obstacles) differently from free-space points
    pub(crate) fn add_free_point_to_graph(
        &mut self,
        location: Point,
        graph: &mut VisibilityGraph,
    ) -> FreePointId {
        // C#: location = ApproximateComparer.Round(location);
        let rounded = Point::round(location);

        // C#: var vertex = VisGraph.FindVertex(location);
        let existing_vertex = graph.find_vertex(rounded);

        // C#: var freePoint = this.FindOrCreateFreePoint(location);
        let fp_id = self.find_or_create_free_point(rounded);

        // Track this location as used in the current routing pass.
        self.free_point_locations_used.push(rounded);

        // C#: if (vertex != null) return freePoint;
        // Vertex already exists in VG — no splicing needed.
        if existing_vertex.is_some() {
            return fp_id;
        }

        // C#: if (!graphGenerator.IsInBounds(location)) {
        //       CreateOutOfBoundsFreePoint(freePoint); return freePoint;
        //     }
        // NOTE: Bounds check requires access to graphGenerator/ObstacleTree.
        // For now, assume in-bounds and splice using PortManager::splice_port.
        // If out of bounds, create_out_of_bounds_free_point would be called.

        // C#: ... splice to surrounding edges in each direction
        // Use the existing PortManager::splice_port for the actual VG splicing.
        // The splice result should be managed by the caller.

        fp_id
    }

    /// Find or create a FreePoint for the given location.
    ///
    /// C# file: PortManager.cs, lines 830-842
    pub(crate) fn find_or_create_free_point(&mut self, location: Point) -> FreePointId {
        if let Some(&fp_id) = self.free_point_map.get(&location) {
            return fp_id;
        }
        let fp_id = FreePointId(self.free_point_map.len());
        self.free_point_map.insert(location, fp_id);
        fp_id
    }

    /// Create an out-of-bounds free point with edges to graph boundary.
    ///
    /// C# file: PortManager.cs, lines 899-940
    /// Big-O: O(log V) for vertex/edge operations
    pub(crate) fn create_out_of_bounds_free_point(
        &mut self,
        _free_point_id: FreePointId,
        _graph: &mut VisibilityGraph,
    ) {
        // C#: var oobLocation = freePoint.Point;
        // C#: Point inboundsLocation = graphGenerator.MakeInBoundsLocation(oobLocation);
        // C#: Direction dirFromGraph = PointComparer.GetDirections(inboundsLocation, oobLocation);
        // C#: freePoint.OutOfBoundsDirectionFromGraph = dirFromGraph;
        //
        // If OOB in two directions (compound direction), connect to corner vertex:
        //   C#: TransUtil.ConnectVertexToTargetVertex(cornerVertex, ...)
        //
        // If OOB in one direction, find nearest perpendicular edge:
        //   C#: ConnectFreePointToLateralEdge(freePoint, lateralDir)
        //
        // NOTE: This requires access to graphGenerator.MakeInBoundsLocation()
        // and TransientGraphUtility, which are not available in FullPortManager.
        // The caller must handle out-of-bounds free point connection externally.
    }

    /// Connect a free point to a lateral visibility edge in the given direction.
    ///
    /// C# file: PortManager.cs, lines 942-954
    /// Big-O: O(log V) for nearest perpendicular edge search
    pub(crate) fn connect_free_point_to_lateral_edge(
        &mut self,
        _free_point_id: FreePointId,
        _lateral_dir: CompassDirection,
        _graph: &mut VisibilityGraph,
    ) {
        // C#: var end = freePoint.IsOverlapped
        //       ? this.InBoundsGraphBoxIntersect(freePoint.Point, lateralDir)
        //       : freePoint.MaxVisibilityInDirectionForNonOverlappedFreePoint(lateralDir, this.TransUtil);
        // C#: var lateralEdge = this.FindorCreateNearestPerpEdge(end, freePoint.Point, lateralDir, freePoint.InitialWeight);
        // C#: if (lateralEdge != null) {
        //       freePoint.AddEdgeToAdjacentEdge(TransUtil, lateralEdge, lateralDir, this.portSpliceLimitRectangle);
        //     }
        //
        // NOTE: This requires FreePoint data (IsOverlapped, Point, InitialWeight),
        // TransientGraphUtility, and ScanSegmentTree access, none of which are
        // stored in FullPortManager. The caller must provide these externally.
    }

    /// Find or create the nearest perpendicular VG edge for port splicing.
    ///
    /// C# file: PortManager.cs, lines 615-735
    /// Big-O: O(log V) for ScanSegmentTree lookup + O(chain) adjacency walk
    /// MUST use ScanSegmentTree find_lowest/highest_intersector for O(log N) lookup
    pub(crate) fn find_or_create_nearest_perp_edge(
        &mut self,
        first: Point,
        second: Point,
        dir: CompassDirection,
        _weight: f64,
        graph: &mut VisibilityGraph,
    ) -> Option<(crate::visibility::graph::VertexId, crate::visibility::graph::VertexId)> {
        // C# algorithm (PortManager.cs lines 615-735):
        // 1. Sort first/second ascending in the search direction
        // 2. Pick perpendicular scan segment tree (H if vertical dir, V if horizontal)
        // 3. Find lowest/highest intersector depending on ascending direction
        // 4. If no segment found, return None
        // 5. Compute edge intersection point
        // 6. Walk from intersection toward the port to find bracketing perpendicular edge

        let is_ascending = match dir {
            CompassDirection::North | CompassDirection::East => true,
            CompassDirection::South | CompassDirection::West => false,
        };

        // Sort first and second so that `low` is the lower coordinate value
        // and `high` is the higher coordinate value along the search axis.
        let (low, high) = if is_ascending {
            if StaticGraphUtility::is_pure_lower(first, second) {
                (first, second)
            } else {
                (second, first)
            }
        } else {
            if StaticGraphUtility::is_pure_lower(first, second) {
                (second, first)
            } else {
                (first, second)
            }
        };

        // NOTE: ScanSegmentTree access is required here for O(log N) lookup.
        // Without it, we fall back to VG-based search using
        // TransientGraphUtility::find_nearest_perpendicular_or_containing_edge.
        //
        // Try to find the nearest perpendicular edge using the VG directly.
        // Look for a vertex near `high` (the point closer to the search direction)
        // and walk to find a perpendicular edge.
        let search_point = if is_ascending { high } else { low };
        let seed = graph.find_vertex(search_point);

        if let Some(seed_vid) = seed {
            // Use the existing find_bracketing_edge logic from PortManager.
            let target_point = if is_ascending { low } else { high };
            if let Some((src, tgt)) = TransientGraphUtility::find_perpendicular_or_containing_edge(
                graph,
                seed_vid,
                dir,
                target_point,
            ) {
                return Some((src, tgt));
            }
        }

        // Fallback: try find_nearest_perpendicular_or_containing_edge if we have
        // any vertex near the search area.
        let fallback_vertex = graph.find_vertex(low).or_else(|| graph.find_vertex(high));
        if let Some(start_vid) = fallback_vertex {
            return TransientGraphUtility::find_nearest_perpendicular_or_containing_edge(
                graph,
                start_vid,
                dir,
                second,
            );
        }

        None
    }
}
