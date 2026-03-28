  private CreateObstaclePortEntrancesFromPoints(oport: ObstaclePort) {
    const graphBox = this.graphGenerator.ObstacleTree.GraphBox
    const curveBox = Rectangle.mkPP(
      Point.RoundPoint(oport.PortCurve.boundingBox.leftBottom),
      Point.RoundPoint(oport.PortCurve.boundingBox.rightTop),
    )
    // This Port does not have a PortEntry, so we'll have visibility edges to its location
    // in the Horizontal and Vertical directions (possibly all 4 directions, if not on boundary).
    //
    // First calculate the intersection with the obstacle in all directions.  Do nothing in the
    // horizontal direction for port locations that are on the unpadded vertical extremes, because
    // this will have a path that moves alongside a rectilinear obstacle side in less than the
    // padding radius and will thus create the PaddedBorderIntersection on the side rather than top
    // (and vice-versa for the vertical direction).  We'll have an edge in the vertical direction
    // to the padded extreme boundary ScanSegment, and the Nudger will modify paths as appropriate
    // to remove unnecessary bends.
    // Use the unrounded port location to intersect with its curve.
    const location: Point = Point.RoundPoint(oport.PortLocation)
    let found = false
    const t: {xx0: Point; xx1: Point} = {xx0: null, xx1: null}

    if (!PointComparer.Equal(location.y, curveBox.top) && !PointComparer.Equal(location.y, curveBox.bottom)) {
      found = true
      const hSeg = new LineSegment(graphBox.left, location.y, graphBox.right, location.y)
      this.GetBorderIntersections(location, hSeg, oport.PortCurve, t)
      let wBorderIntersect = new Point(Math.min(t.xx0.x, t.xx1.x), location.y)
      if (wBorderIntersect.x < curveBox.left) {
        // Handle rounding error
        wBorderIntersect = new Point(curveBox.left, wBorderIntersect.y)
      }

      let eBorderIntersect = new Point(Math.max(t.xx0.x, t.xx1.x), location.y)
      if (eBorderIntersect.x > curveBox.right) {
        eBorderIntersect = new Point(curveBox.right, eBorderIntersect.y)
      }

      this.CreatePortEntrancesAtBorderIntersections(curveBox, oport, location, wBorderIntersect, eBorderIntersect)
    }

    // endif horizontal pass is not at vertical extreme
    if (!PointComparer.Equal(location.x, curveBox.left) && !PointComparer.Equal(location.x, curveBox.right)) {
      found = true
      const vSeg = new LineSegment(location.x, graphBox.bottom, location.x, graphBox.top)
      this.GetBorderIntersections(location, vSeg, oport.PortCurve, t)
      let sBorderIntersect = new Point(location.x, Math.min(t.xx0.y, t.xx1.y))
      if (sBorderIntersect.y < graphBox.bottom) {
        // Handle rounding error
        sBorderIntersect = new Point(sBorderIntersect.x, graphBox.bottom)
      }

      let nBorderIntersect = new Point(location.x, Math.max(t.xx0.y, t.xx1.y))
      if (nBorderIntersect.y > graphBox.top) {
        nBorderIntersect = new Point(nBorderIntersect.x, graphBox.top)
      }

      this.CreatePortEntrancesAtBorderIntersections(curveBox, oport, location, sBorderIntersect, nBorderIntersect)
    }

    // endif vertical pass is not at horizontal extreme
    if (!found) {
      // This must be on a corner, else one of the above would have matched.
      this.CreateEntrancesForCornerPort(curveBox, oport, location)
    }
  }

  private GetBorderIntersections(location: Point, lineSeg: LineSegment, curve: ICurve, t: {xx0: Point; xx1: Point}) {
    // Important:  the LineSegment must be the first arg to GetAllIntersections so RawIntersection works.
    const xxs = Curve.getAllIntersections(lineSeg, curve, true)
    /*Assert.assert(2 === xxs.length, 'Expected two intersections')*/
    t.xx0 = Point.RoundPoint(xxs[0].x)
    t.xx1 = Point.RoundPoint(xxs[1].x)
  }

  private CreatePortEntrancesAtBorderIntersections(
    curveBox: Rectangle,
    oport: ObstaclePort,
    location: Point,
    unpaddedBorderIntersect0: Point,
    unpaddedBorderIntersect1: Point,
  ) {
    // Allow entry from both sides, except from the opposite side of a point on the border.
    const dir: Direction = PointComparer.GetDirections(unpaddedBorderIntersect0, unpaddedBorderIntersect1)
    if (!PointComparer.EqualPP(unpaddedBorderIntersect0, location)) {
      this.CreatePortEntrance(curveBox, oport, unpaddedBorderIntersect1, dir)
    }

    if (!PointComparer.EqualPP(unpaddedBorderIntersect1, location)) {
      this.CreatePortEntrance(curveBox, oport, unpaddedBorderIntersect0, CompassVector.OppositeDir(dir))
    }
  }

  private static GetDerivative(oport: ObstaclePort, borderPoint: Point): Point {
    // This is only used for ObstaclePorts, which have ensured Port.Curve is not null.
    const param: number = oport.PortCurve.closestParameter(borderPoint)
    let deriv = oport.PortCurve.derivative(param)
    const parMid = (oport.PortCurve.parStart + oport.PortCurve.parEnd) / 2
    if (!InteractiveObstacleCalculator.CurveIsClockwise(oport.PortCurve, oport.PortCurve.value(parMid))) {
      deriv = deriv.mul(-1)
    }

    return deriv
  }

  private CreatePortEntrance(curveBox: Rectangle, oport: ObstaclePort, unpaddedBorderIntersect: Point, outDir: Direction) {
    oport.CreatePortEntrance(unpaddedBorderIntersect, outDir, this.ObstacleTree)
    const scanDir: ScanDirection = ScanDirection.GetInstance(outDir)
    let axisDistanceBetweenIntersections: number =
      StaticGraphUtility.GetRectangleBound(curveBox, outDir) - scanDir.Coord(unpaddedBorderIntersect)
    if (axisDistanceBetweenIntersections < 0) {
      axisDistanceBetweenIntersections = -axisDistanceBetweenIntersections
    }

    if (axisDistanceBetweenIntersections > GeomConstants.intersectionEpsilon) {
      // This is not on an extreme boundary of the unpadded curve (it's on a sloping (nonrectangular) boundary),
      // so we need to generate another entrance in one of the perpendicular directions (depending on which
      // way the side slopes).  Derivative is always clockwise.
      const perpDirs: Direction = CompassVector.VectorDirection(PortManager.GetDerivative(oport, unpaddedBorderIntersect))
      let perpDir: Direction
      outDir | CompassVector.OppositeDir(outDir)
      if (Direction.None !== (outDir & perpDirs)) {
        // If the derivative is in the same direction as outDir then perpDir is toward the obstacle
        // interior and must be reversed.
        perpDir = CompassVector.OppositeDir(perpDir)
      }

      oport.CreatePortEntrance(unpaddedBorderIntersect, perpDir, this.ObstacleTree)
    }
  }

  private CreateEntrancesForCornerPort(curveBox: Rectangle, oport: ObstaclePort, location: Point) {
    // This must be a corner or it would have been within one of the bounds and handled elsewhere.
    // Therefore create an entrance in both directions, with the first direction selected so that
    // the second can be obtained via RotateRight.
    let outDir: Direction = Direction.North
    if (PointComparer.EqualPP(location, curveBox.leftBottom)) {
      outDir = Direction.South
    } else if (PointComparer.EqualPP(location, curveBox.leftTop)) {
      outDir = Direction.West
    } else if (PointComparer.EqualPP(location, curveBox.rightTop)) {
      outDir = Direction.North
    } else if (PointComparer.EqualPP(location, curveBox.rightBottom)) {
      outDir = Direction.East
    } else {
      /*Assert.assert(false, 'Expected Port to be on corner of curveBox')*/
    }

    oport.CreatePortEntrance(location, outDir, this.ObstacleTree)
    oport.CreatePortEntrance(location, CompassVector.RotateRight(outDir), this.ObstacleTree)
  }

  private AddObstaclePortEntranceToGraph(entrance: ObstaclePortEntrance) {
    // Note: As discussed in ObstaclePortEntrance.AddToGraph, oport.VisibilityBorderIntersect may be
    // on a border shared with another obstacle, in which case we'll extend into that obstacle.  This
    // should be fine if we're consistent about "touching means overlapped", so that a path that comes
    // through the other obstacle on the shared border is OK.
    const borderVertex: VisibilityVertex = this.VisGraph.FindVertex(entrance.VisibilityBorderIntersect)
    if (borderVertex) {
      entrance.ExtendEdgeChain(this.TransUtil, borderVertex, borderVertex, this.portSpliceLimitRectangle, this.RouteToCenterOfObstacles)
      return
    }

    // There may be no scansegment to splice to before we hit an adjacent obstacle, so if the edge
    // is null there is nothing to do.
    const t: {targetVertex: VisibilityVertex} = {targetVertex: null}
    const weight: number = entrance.IsOverlapped ? ScanSegment.OverlappedWeight : ScanSegment.NormalWeight

    const edge: VisibilityEdge = this.FindorCreateNearestPerpEdgePPDNT(
      entrance.MaxVisibilitySegment.end,
      entrance.VisibilityBorderIntersect,
      entrance.OutwardDirection,
      weight,
      t,
    )
    if (edge != null) {
      entrance.AddToAdjacentVertex(this.TransUtil, t.targetVertex, this.portSpliceLimitRectangle, this.RouteToCenterOfObstacles)
    }
  }

