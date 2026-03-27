//! Auto-generated skeleton from `port_manager_entrances.ts`.
//! Fill in the todo!() bodies by translating the TS reference line-by-line.
//!
//! Reference: `reference/port_manager_entrances.ts`

use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;
use crate::geometry::point_comparer::GeomConstants;
use crate::visibility::graph::{VertexId, VisibilityGraph};
use super::compass_direction::CompassDirection;
use super::obstacle_tree::ObstacleTree;
use super::transient_graph_utility::TransientGraphUtility;

impl FullPortManager {

    // ── TS: port_manager_entrances.ts lines 1-64 (64 lines) ──
    //
    // Branches:
    //   if (line 22): !PointComparer.Equal(location.y, curveBox.top) && !PointComp
    //   if (line 27): wBorderIntersect.x < curveBox.left
    //   if (line 33): eBorderIntersect.x > curveBox.right
    //   if (line 41): !PointComparer.Equal(location.x, curveBox.left) && !PointCom
    //   if (line 46): sBorderIntersect.y < graphBox.bottom
    //   if (line 52): nBorderIntersect.y > graphBox.top
    //   if (line 60): !found
    // Calls: Rectangle.mkPP, Point.RoundPoint, PointComparer.Equal, this.GetBorderIntersections, Math.min, Math.max, this.CreatePortEntrancesAtBorderIntersections, this.CreateEntrancesForCornerPort
    fn create_obstacle_port_entrances_from_points(&mut self, oport: ObstaclePort) {
        todo!()
    }

    // ── TS: port_manager_entrances.ts lines 66-72 (7 lines) ──
    //
    // Calls: Curve.getAllIntersections, Point.RoundPoint
    fn get_border_intersections(&mut self, location: Point, line_seg: (Point, Point), curve: & Polyline, t: /* TS: {xx0: Point; xx1: Point} */ todo_type) {
        todo!()
    }

    // ── TS: port_manager_entrances.ts lines 74-90 (17 lines) ──
    //
    // Branches:
    //   if (line 83): !PointComparer.EqualPP(unpaddedBorderIntersect0, location)
    //   if (line 87): !PointComparer.EqualPP(unpaddedBorderIntersect1, location)
    // Calls: PointComparer.GetDirections, PointComparer.EqualPP, this.CreatePortEntrance, CompassVector.OppositeDir
    fn create_port_entrances_at_border_intersections(&mut self, curve_box: & Rectangle, oport: ObstaclePort, location: Point, unpadded_border_intersect0: Point, unpadded_border_intersect1: Point) {
        todo!()
    }

    // ── TS: port_manager_entrances.ts lines 92-102 (11 lines) ──
    //
    // Branches:
    //   if (line 97): !InteractiveObstacleCalculator.CurveIsClockwise(oport.PortCu
    // Calls: oport.PortCurve.closestParameter, oport.PortCurve.derivative, InteractiveObstacleCalculator.CurveIsClockwise, oport.PortCurve.value, deriv.mul
    fn get_derivative(oport: ObstaclePort, border_point: Point) -> Point {
        todo!()
    }

    // ── TS: port_manager_entrances.ts lines 104-128 (25 lines) ──
    //
    // Branches:
    //   if (line 109): axisDistanceBetweenIntersections < 0
    //   if (line 113): axisDistanceBetweenIntersections > GeomConstants.intersectio
    //   if (line 120): Direction.None !== (outDir & perpDirs)
    // Calls: oport.CreatePortEntrance, ScanDirection.GetInstance, StaticGraphUtility.GetRectangleBound, scanDir.Coord, CompassVector.VectorDirection, PortManager.GetDerivative, CompassVector.OppositeDir
    fn create_port_entrance(&mut self, curve_box: & Rectangle, oport: ObstaclePort, unpadded_border_intersect: Point, out_dir: CompassDirection) {
        todo!()
    }

    // ── TS: port_manager_entrances.ts lines 130-149 (20 lines) ──
    //
    // Branches:
    //   if (line 135): PointComparer.EqualPP(location, curveBox.leftBottom)
    //   if (line 137): PointComparer.EqualPP(location, curveBox.leftTop)
    //   if (line 139): PointComparer.EqualPP(location, curveBox.rightTop)
    //   if (line 141): PointComparer.EqualPP(location, curveBox.rightBottom)
    // Calls: PointComparer.EqualPP, oport.CreatePortEntrance, CompassVector.RotateRight
    fn create_entrances_for_corner_port(&mut self, curve_box: & Rectangle, oport: ObstaclePort, location: Point) {
        todo!()
    }

    // ── TS: port_manager_entrances.ts lines 151-177 (27 lines) ──
    //
    // Branches:
    //   if (line 157): borderVertex
    //   if (line 174): edge != null
    // Calls: this.VisGraph.FindVertex, entrance.ExtendEdgeChain, this.FindorCreateNearestPerpEdgePPDNT, entrance.AddToAdjacentVertex
    fn add_obstacle_port_entrance_to_graph(&mut self, entrance: ObstaclePortEntrance) {
        todo!()
    }
}
