//
// PortManager.cs
// MSAGL class for Port management for Rectilinear Edge Routing.
//

// This stores information mapping the App-level Ports (e.g. FloatingPort, RelativeFloatingPort,
// and MultiLocationFloatingPort) to the router's BasicScanPort subclasses (ObstaclePort and FreePoint).
import {uniteSets, substractSets} from '../../utils/setOperations'

import {Point, Rectangle, ICurve} from '../../math/geometry'
import {Port} from '../../layout/core/port'
import {CompassVector} from '../../math/geometry/compassVector'
import {PointLocation, Curve} from '../../math/geometry/curve'
import {Direction} from '../../math/geometry/direction'
import {LineSegment} from '../../math/geometry/lineSegment'
import {InteractiveObstacleCalculator} from '../interactiveObstacleCalculator'
import {Shape} from '../shape'
import {VisibilityEdge} from '../visibility/VisibilityEdge'
import {VisibilityGraph} from '../visibility/VisibilityGraph'
import {VisibilityVertex} from '../visibility/VisibilityVertex'
import {Obstacle} from './obstacle'
import {ObstaclePort} from './ObstaclePort'
import {ObstacleTree} from './ObstacleTree'
import {PointComparer} from './PointComparer'
import {ScanDirection} from './ScanDirection'
import {ScanSegment} from './ScanSegment'
import {ScanSegmentTree} from './ScanSegmentTree'
import {StaticGraphUtility} from './StaticGraphUtility'
import {VisibilityGraphGenerator} from './VisibilityGraphGenerator'
import {FreePoint} from './FreePoint'
import {GeomConstants} from '../../math/geometry/geomConstants'

import {TransientGraphUtility} from './TransientGraphUtility'
import {ObstaclePortEntrance} from './ObstaclePortEntrance'
import {PointMap} from '../../utils/PointMap'
import {PointSet} from '../../utils/PointSet'
import {GeomEdge} from '../..'

export class PortManager {
  // The mapping of Msagl.Port (which may be MultiLocation) to the underlying Obstacle.Shape.
  private obstaclePortMap: Map<Port, ObstaclePort> = new Map<Port, ObstaclePort>()

  // The mapping of Msagl.Port.Location or a Waypoint to a FreePoint with visibility info.
  private freePointMap: PointMap<FreePoint> = new PointMap<FreePoint>()

  // This tracks which locations were used by the last call to RouteEdges, so we can remove unused locations.
  private freePointLocationsUsedByRouteEdges: PointSet = new PointSet()

  // Created to wrap the graph for adding transient vertices/edges to the graph.
  TransUtil: TransientGraphUtility

  // Owned by RectilinearEdgeRouter.
  private graphGenerator: VisibilityGraphGenerator

  // Storage and implementation of RectilinearEdgeRouter property of the same name.
  RouteToCenterOfObstacles = false
  // Extension of port visibility splices into the visibility graph.
  get LimitPortVisibilitySpliceToEndpointBoundingBox(): boolean {
    return this.TransUtil.LimitPortVisibilitySpliceToEndpointBoundingBox
  }
  set LimitPortVisibilitySpliceToEndpointBoundingBox(value: boolean) {
    this.TransUtil.LimitPortVisibilitySpliceToEndpointBoundingBox = value
  }

  // A control point is a source, target, or waypoint (terminology only, there's no ControlPoint
  // class).  These lists are the control points we've added for the current path.
  private obstaclePortsInGraph: Array<ObstaclePort> = new Array<ObstaclePort>()

  private freePointsInGraph: Set<FreePoint> = new Set<FreePoint>()

  // The limit for edge-chain extension.
  private portSpliceLimitRectangle: Rectangle

  // The current set of Obstacles that are groups whose boundaries are crossable.
  private activeAncestors: Array<Obstacle> = new Array<Obstacle>()

  // Typing shortcuts
  private get VisGraph(): VisibilityGraph {
    return this.graphGenerator.VisibilityGraph
  }

  private get HScanSegments(): ScanSegmentTree {
    return this.graphGenerator.HorizontalScanSegments
  }

  private get VScanSegments(): ScanSegmentTree {
    return this.graphGenerator.VerticalScanSegments
  }

  private get ObstacleTree(): ObstacleTree {
    return this.graphGenerator.ObstacleTree
  }

  private get AncestorSets(): Map<Shape, Set<Shape>> {
    return this.ObstacleTree.AncestorSets
  }

  constructor(graphGenerator: VisibilityGraphGenerator) {
    this.TransUtil = new TransientGraphUtility(graphGenerator)
    this.graphGenerator = graphGenerator
  }
