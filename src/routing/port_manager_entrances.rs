//! Port entrance creation for FullPortManager.
//!
//! Auto-generated skeleton from `port_manager_entrances.ts`.
//! Fill in the todo!() bodies by translating the TS reference line-by-line.
//!
//! Reference: `reference/port_manager_entrances.ts`

use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;
use super::compass_direction::{CompassDirection, Direction};
use super::obstacle_tree::ObstacleTree;
use super::port::{ObstaclePort, ObstaclePortEntrance};
use super::port_manager::FullPortManager;
use super::router_session::RouterSession;
use super::scan_direction::ScanDirection;
use super::static_graph_utility::StaticGraphUtility;
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
        // C# line 539: oport.CreatePortEntrance(unpaddedBorderIntersect, outDir, this.ObstacleTree)
        let obstacle_index = oport.obstacle_index;
        oport.create_port_entrance(unpadded_border_intersect, out_dir, obstacle_index, obstacle_tree, graph_box);

        // C# lines 540-556: For non-rectangular obstacles, check if the border intersect
        // is NOT on the rectangular extreme boundary (axisDistanceBetweenIntersections > epsilon).
        // If so, compute the tangent direction of the polyline at the border point and create
        // a second entrance in the perpendicular direction.
        //
        // For rectangular obstacles port_curve_polyline is None, so this block never fires.
        if let Some(polyline) = &oport.port_curve_polyline.clone() {
            // C# line 541: ScanDirection scanDir = ScanDirection.GetInstance(outDir)
            let scan_dir = ScanDirection::for_compass(out_dir);

            // C# lines 542-545: axisDistanceBetweenIntersections =
            //   |GetRectangleBound(curveBox, outDir) - scanDir.Coord(unpaddedBorderIntersect)|
            let rect_bound = StaticGraphUtility::get_rectangle_bound(&oport.port_curve_bbox, out_dir);
            let intersect_coord = scan_dir.coord(unpadded_border_intersect);
            let axis_dist = (rect_bound - intersect_coord).abs();

            if axis_dist > GeomConstants::INTERSECTION_EPSILON {
                // C# line 547: perpDirs = CompassVector.VectorDirection(GetDerivative(oport, unpaddedBorderIntersect))
                let deriv = get_polyline_derivative(polyline, unpadded_border_intersect);
                let perp_dirs = Direction::from_vector(deriv.x(), deriv.y());

                // C# line 548: perpDir = perpDirs & ~(outDir | OppositeDir(outDir))
                let out_flag = out_dir.to_direction();
                let opp_flag = out_dir.opposite().to_direction();
                let mut perp_dir = perp_dirs & !(out_flag | opp_flag);

                // C# lines 549-551: if outDir is contained in perpDirs, flip perpDir
                if !(out_flag & perp_dirs).is_none() {
                    perp_dir = !perp_dir & Direction::ALL;
                }

                // C# line 552: if Direction.None != (outDir & perpDirs) → already handled above
                // C# line 553: oport.CreatePortEntrance(unpaddedBorderIntersect, perpDir, this.ObstacleTree)
                if perp_dir.is_pure() {
                    let perp_compass = CompassDirection::from_direction(perp_dir);
                    oport.create_port_entrance(unpadded_border_intersect, perp_compass, obstacle_index, obstacle_tree, graph_box);
                }
            }
        }
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
        session: &mut RouterSession,
        trans_util: &mut TransientGraphUtility,
        limit_rect: &Rectangle,
        route_to_center: bool,
    ) {
        // TS line 156: const borderVertex = this.VisGraph.FindVertex(entrance.VisibilityBorderIntersect)
        let border_vertex = session.vis_graph.find_vertex(entrance.visibility_border_intersect);

        if let Some(bv) = border_vertex {
            // TS line 158: entrance.ExtendEdgeChain(transUtil, bv, bv, limitRect, routeToCenter)
            entrance.extend_edge_chain(session, trans_util, bv, bv, limit_rect, route_to_center);
            return;
        }

        // TS lines 167-173: Find nearest perpendicular edge in the outward direction.
        // Use TransientGraphUtility's find_nearest_perpendicular_or_containing_edge
        // as a simplified version of FindorCreateNearestPerpEdgePPDNT.
        let target_vertex = {
            let start_vertex = trans_util.find_or_add_vertex(session, entrance.visibility_border_intersect);
            TransientGraphUtility::find_nearest_perpendicular_or_containing_edge(
                &session.vis_graph,
                start_vertex,
                entrance.outward_direction,
                entrance.max_visibility_segment_end,
            )
            .map(|(src, _tgt)| src)
            .or(Some(start_vertex))
        };

        // TS line 174-176: if edge found, call AddToAdjacentVertex
        if let Some(tv) = target_vertex {
            entrance.add_to_adjacent_vertex(session, trans_util, tv, limit_rect, route_to_center);
        }
    }
}

/// Faithful port of C# `PortManager.GetDerivative()`.
///
/// For a polyline boundary, the "derivative" at a border point is the
/// direction vector of the polyline edge that the point lies on (clockwise
/// orientation). The C# version calls `PortCurve.Derivative(param)` on an
/// ICurve; for polylines the derivative is simply the edge direction.
///
/// Algorithm:
/// 1. Iterate over consecutive edges of the closed polyline.
/// 2. Find the edge whose segment contains `border_point` (min distance).
/// 3. Return the normalised direction vector of that edge.
///
/// The polyline is assumed to be clockwise (Shape::polygon enforces this).
fn get_polyline_derivative(polyline: &Polyline, border_point: Point) -> Point {
    let mut best_dist = f64::MAX;
    let mut best_dir = Point::new(1.0, 0.0); // fallback: East

    let pts: Vec<Point> = polyline.points().collect();
    let n = pts.len();
    if n < 2 {
        return best_dir;
    }

    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];

        // Distance from border_point to segment a→b.
        let ab = b - a;
        let ab_len2 = ab.x() * ab.x() + ab.y() * ab.y();
        if ab_len2 < 1e-20 {
            continue; // degenerate segment
        }

        let t = ((border_point.x() - a.x()) * ab.x() + (border_point.y() - a.y()) * ab.y())
            / ab_len2;
        let t_clamped = t.clamp(0.0, 1.0);
        let closest = Point::new(a.x() + t_clamped * ab.x(), a.y() + t_clamped * ab.y());
        let dx = border_point.x() - closest.x();
        let dy = border_point.y() - closest.y();
        let dist = dx * dx + dy * dy;

        if dist < best_dist {
            best_dist = dist;
            // Normalise the edge direction (clockwise polyline → derivative in clockwise order).
            let len = ab_len2.sqrt();
            best_dir = Point::new(ab.x() / len, ab.y() / len);
        }
    }

    best_dir
}
