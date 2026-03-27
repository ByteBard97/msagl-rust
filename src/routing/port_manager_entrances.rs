//! Port entrance creation for FullPortManager.
//!
//! Auto-generated skeleton from `port_manager_entrances.ts`.
//! Fill in the todo!() bodies by translating the TS reference line-by-line.
//!
//! Reference: `reference/port_manager_entrances.ts`

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::rectangle::Rectangle;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use super::compass_direction::CompassDirection;
use super::obstacle_tree::ObstacleTree;
use super::port::{ObstaclePort, ObstaclePortEntrance};
use super::port_manager::FullPortManager;
use super::transient_graph_utility::TransientGraphUtility;

impl FullPortManager {
    // ── TS: port_manager_entrances.ts lines 1-64 (64 lines) ──
    // C#: PortManager.cs lines 447-503
    //
    // Branches:
    //   if: location.y NOT on top/bottom boundary → create E/W entrances
    //   if: location.x NOT on left/right boundary → create N/S entrances
    //   if: neither matched → corner port
    // Calls: CreatePortEntrancesAtBorderIntersections, CreateEntrancesForCornerPort
    //
    // For rectangular obstacles, border intersections are simply the bounding box
    // edges at the port's x or y coordinate. No curve intersection needed.
    pub(crate) fn create_obstacle_port_entrances_from_points(
        oport: &mut ObstaclePort,
        padded_box: &Rectangle,
        graph_box: &Rectangle,
        obstacle_tree: &mut ObstacleTree,
    ) {
        // TS line 18: const location = Point.RoundPoint(oport.PortLocation)
        let location = Point::round(oport.location);
        let mut found = false;

        // TS lines 22-38: Horizontal pass — if port is NOT on top or bottom boundary,
        // create E and W entrances at the horizontal border intersections.
        if !GeomConstants::close(location.y(), padded_box.top())
            && !GeomConstants::close(location.y(), padded_box.bottom())
        {
            found = true;
            // For rectangular obstacles, horizontal border intersections are simply
            // (box.left, location.y) and (box.right, location.y).
            let w_border = Point::new(padded_box.left(), location.y());
            let e_border = Point::new(padded_box.right(), location.y());
            Self::create_port_entrances_at_border_intersections(
                oport, padded_box, location, w_border, e_border, obstacle_tree, graph_box,
            );
        }

        // TS lines 41-57: Vertical pass — if port is NOT on left or right boundary,
        // create N and S entrances at the vertical border intersections.
        if !GeomConstants::close(location.x(), padded_box.left())
            && !GeomConstants::close(location.x(), padded_box.right())
        {
            found = true;
            // For rectangular obstacles, vertical border intersections are simply
            // (location.x, box.bottom) and (location.x, box.top).
            let s_border = Point::new(location.x(), padded_box.bottom());
            let n_border = Point::new(location.x(), padded_box.top());
            Self::create_port_entrances_at_border_intersections(
                oport, padded_box, location, s_border, n_border, obstacle_tree, graph_box,
            );
        }

        // TS lines 60-63: Corner case — port is on both boundaries
        if !found {
            Self::create_entrances_for_corner_port(oport, padded_box, location, obstacle_tree, graph_box);
        }
    }

    // ── TS: port_manager_entrances.ts lines 74-90 (17 lines) ──
    // C#: PortManager.cs lines 515-525
    //
    // Branches:
    //   if: unpaddedBorderIntersect0 != location → create entrance at intersect1 in dir
    //   if: unpaddedBorderIntersect1 != location → create entrance at intersect0 in opposite dir
    // Calls: oport.CreatePortEntrance
    fn create_port_entrances_at_border_intersections(
        oport: &mut ObstaclePort,
        padded_box: &Rectangle,
        location: Point,
        unpadded_border_intersect0: Point,
        unpadded_border_intersect1: Point,
        obstacle_tree: &mut ObstacleTree,
        graph_box: &Rectangle,
    ) {
        // TS line 82: const dir = PointComparer.GetDirections(intersect0, intersect1)
        let dir = CompassDirection::from_points(unpadded_border_intersect0, unpadded_border_intersect1);

        if let Some(dir) = dir {
            // TS line 83-85: if intersect0 != location, create entrance at intersect1 in dir
            if !unpadded_border_intersect0.close_to(location) {
                Self::create_port_entrance(oport, padded_box, unpadded_border_intersect1, dir, obstacle_tree, graph_box);
            }
            // TS line 87-89: if intersect1 != location, create entrance at intersect0 in opposite dir
            if !unpadded_border_intersect1.close_to(location) {
                Self::create_port_entrance(oport, padded_box, unpadded_border_intersect0, dir.opposite(), obstacle_tree, graph_box);
            }
        }
    }

    // ── TS: port_manager_entrances.ts lines 104-128 (25 lines) ──
    // C#: PortManager.cs lines 538-556
    //
    // Creates the port entrance and optionally a perpendicular entrance
    // for non-rectangular obstacles (sloped boundary).
    // For rectangular obstacles, only the primary entrance is created.
    // Calls: oport.CreatePortEntrance
    fn create_port_entrance(
        oport: &mut ObstaclePort,
        padded_box: &Rectangle,
        unpadded_border_intersect: Point,
        out_dir: CompassDirection,
        obstacle_tree: &mut ObstacleTree,
        graph_box: &Rectangle,
    ) {
        // TS line 105: oport.CreatePortEntrance(unpaddedBorderIntersect, outDir, this.ObstacleTree)
        oport.create_port_entrance(unpadded_border_intersect, out_dir, oport.obstacle_index, obstacle_tree, graph_box);

        // TS lines 106-127: For non-rectangular obstacles, check if the border intersect
        // is on a sloped boundary and add a perpendicular entrance. For rectangular obstacles
        // (our current scope), the distance is always 0 so this never fires.
        // TODO: implement perpendicular entrance for non-rectangular obstacles
    }

    // ── TS: port_manager_entrances.ts lines 130-149 (20 lines) ──
    // C#: PortManager.cs lines 571-581
    //
    // Branches:
    //   if: location == leftBottom → create N and E entrances
    //   if: location == leftTop → create E and S entrances (or S and E depending on winding)
    //   if: location == rightTop → create S and W entrances
    //   if: location == rightBottom → create W and N entrances
    // Calls: oport.CreatePortEntrance, CompassVector.RotateRight
    fn create_entrances_for_corner_port(
        oport: &mut ObstaclePort,
        padded_box: &Rectangle,
        location: Point,
        obstacle_tree: &mut ObstacleTree,
        graph_box: &Rectangle,
    ) {
        // TS lines 134-145: determine outDir based on which corner
        let out_dir = if location.close_to(padded_box.left_bottom()) {
            CompassDirection::South
        } else if location.close_to(padded_box.left_top()) {
            CompassDirection::West
        } else if location.close_to(padded_box.right_top()) {
            CompassDirection::North
        } else if location.close_to(padded_box.right_bottom()) {
            CompassDirection::East
        } else {
            debug_assert!(false, "Expected port to be on corner of padded_box");
            return;
        };

        // TS lines 147-148: create entrances in outDir and RotateRight(outDir)
        let obstacle_index = oport.obstacle_index;
        oport.create_port_entrance(location, out_dir, obstacle_index, obstacle_tree, graph_box);
        oport.create_port_entrance(location, out_dir.right(), obstacle_index, obstacle_tree, graph_box);
    }

    // ── TS: port_manager_entrances.ts lines 151-177 (27 lines) ──
    // C#: PortManager.cs lines 360-368 (AddObstaclePortEntranceToGraph)
    //
    // Branches:
    //   if: border vertex already exists → just extend chain
    //   else: find nearest perp edge → add_to_adjacent_vertex
    //   if: edge found → create edge between entrance vertex and target
    // Calls: entrance.ExtendEdgeChain, entrance.AddToAdjacentVertex
    pub(crate) fn add_obstacle_port_entrance_to_graph(
        entrance: &ObstaclePortEntrance,
        graph: &mut VisibilityGraph,
        trans_util: &mut TransientGraphUtility,
        obstacle_tree: &mut ObstacleTree,
        limit_rect: &Rectangle,
        route_to_center: bool,
    ) {
        // TS line 156: const borderVertex = this.VisGraph.FindVertex(entrance.VisibilityBorderIntersect)
        let border_vertex = graph.find_vertex(entrance.visibility_border_intersect);

        if let Some(bv) = border_vertex {
            // TS line 158: entrance.ExtendEdgeChain(transUtil, bv, bv, limitRect, routeToCenter)
            entrance.extend_edge_chain(graph, trans_util, obstacle_tree, bv, bv, limit_rect, route_to_center);
            return;
        }

        // TS lines 167-173: Find nearest perpendicular edge in the outward direction.
        // Use TransientGraphUtility's find_nearest_perpendicular_or_containing_edge
        // as a simplified version of FindorCreateNearestPerpEdgePPDNT.
        let target_vertex = {
            let start_vertex = trans_util.find_or_add_vertex(graph, entrance.visibility_border_intersect);
            TransientGraphUtility::find_nearest_perpendicular_or_containing_edge(
                graph,
                start_vertex,
                entrance.outward_direction,
                entrance.max_visibility_segment_end,
            )
            .map(|(src, _tgt)| src)
            .or(Some(start_vertex))
        };

        // TS line 174-176: if edge found, call AddToAdjacentVertex
        if let Some(tv) = target_vertex {
            entrance.add_to_adjacent_vertex(graph, trans_util, obstacle_tree, tv, limit_rect, route_to_center);
        }
    }
}
