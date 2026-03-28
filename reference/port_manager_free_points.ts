  private GetPortSpliceLimitRectangle(edgeGeom: GeomEdge) {
    if (!this.LimitPortVisibilitySpliceToEndpointBoundingBox) {
      this.portSpliceLimitRectangle = this.graphGenerator.ObstacleTree.GraphBox
      return
    }

    // Return the endpoint-containing rectangle marking the limits of edge-chain extension for a single path.
    this.portSpliceLimitRectangle = this.GetPortRectangle(edgeGeom.sourcePort)
    this.portSpliceLimitRectangle.addRecSelf(this.GetPortRectangle(edgeGeom.targetPort))
  }

  GetPortRectangle(port: Port): Rectangle {
    const oport: ObstaclePort = this.obstaclePortMap.get(port)
    if (oport) {
      return oport.Obstacle.VisibilityBoundingBox.clone()
    }

    // FreePoint.
    return Rectangle.mkOnPoints([Point.RoundPoint(port.Location)])
  }

  AddToLimitRectangle(location: Point) {
    if (this.graphGenerator.IsInBoundsP(location)) {
      this.portSpliceLimitRectangle.add(location)
    }
  }

  private FindOrCreateFreePoint(location: Point): FreePoint {
    let freePoint: FreePoint = this.freePointMap.get(location)
    if (!freePoint) {
      freePoint = new FreePoint(this.TransUtil, location)
      this.freePointMap.set(location, freePoint)
    } else {
      freePoint.GetVertex(this.TransUtil, location)
    }

    this.freePointsInGraph.add(freePoint)
    this.freePointLocationsUsedByRouteEdges.add(location)
    return freePoint
  }

  // This is private because it depends on LimitRectangle
  private AddFreePointToGraph(location: Point): FreePoint {
    // This is a FreePoint, either a Waypoint or a Port not in an Obstacle.Ports list.
    // We can't modify the Port.Location as the caller owns that, so Point.RoundPoint it
    // at the point at which we acquire it.
    location = Point.RoundPoint(location)
    // If the point already exists before FreePoint creation, there's nothing to do.
    const vertex = this.VisGraph.FindVertex(location)
    const freePoint = this.FindOrCreateFreePoint(location)
    if (vertex != null) {
      return freePoint
    }

    if (!this.graphGenerator.IsInBoundsP(location)) {
      this.CreateOutOfBoundsFreePoint(freePoint)
      return freePoint
    }

    // Vertex is inbounds and does not yet exist.  Possibilities are:
    //  - point is on one ScanSegment (perhaps a dead-end)
    //  - point is not on any edge (it's in free space so it's in the middle of some rectangle
    //    (possibly not closed) formed by ScanSegment intersections)
    let edge: VisibilityEdge = null
    freePoint.IsOverlapped = this.ObstacleTree.PointIsInsideAnObstacle(freePoint.Point, this.HScanSegments.ScanDirection)
    let scanSegment
    this.VScanSegments.FindSegmentContainingPoint(location, true)
    if (scanSegment != null) {
      // The location is on one ScanSegment.  Find the intersector and split an edge along the segment
      // (or extend the VisibilityEdges of the segment in the desired direction).
      const t: {targetVertex: VisibilityVertex} = {targetVertex: null}
      edge = this.FindOrCreateNearestPerpEdgeFromNearestPerpSegment(location, scanSegment, location, freePoint.InitialWeight, t)
    }

    let edgeDir: Direction = Direction.South
    if (edge != null) {
      // The freePoint is on one (but not two) segments, and has already been spliced into
      // that segment's edge chain.  Add edges laterally to the parallel edges.
      edgeDir = StaticGraphUtility.EdgeDirectionVE(edge)
      this.ConnectFreePointToLateralEdge(freePoint, CompassVector.RotateLeft(edgeDir))
      this.ConnectFreePointToLateralEdge(freePoint, CompassVector.RotateRight(edgeDir))
    } else {
      // The freePoint is not on ScanSegment so we must splice to 4 surrounding edges (or it may be on a
      // TransientVE). Look in each of the 4 directions, trying first to avoid crossing any obstacle
      // boundaries.  However if we cannot find an edge that does not cross an obstacle boundary, the
      // freepoint is inside a non-overlapped obstacle, so take a second pass to connect to the nearest
      // edge regardless of obstacle boundaries.
      for (let ii = 0; ii < 4; ii++) {
        this.ConnectFreePointToLateralEdge(freePoint, edgeDir)
        edgeDir = CompassVector.RotateLeft(edgeDir)
      }
    }

    return freePoint
  }

  private CreateOutOfBoundsFreePoint(freePoint: FreePoint) {
    // For an out of bounds (OOB) point, we'll link one edge from it to the inbounds edge if it's
    // out of bounds in only one direction; if in two, we'll add a bend. Currently we don't need
    // to do any more because multiple waypoints are processed as multiple subpaths.
    const oobLocation = freePoint.Point
    const inboundsLocation: Point = this.graphGenerator.MakeInBoundsLocation(oobLocation)
    const dirFromGraph: Direction = PointComparer.GetDirections(inboundsLocation, oobLocation)
    freePoint.OutOfBoundsDirectionFromGraph = dirFromGraph
    if (!PointComparer.IsPureDirectionD(dirFromGraph)) {
      // It's OOB in two directions so will need a bend, but we know inboundsLocation
      // is a graph corner so it has a vertex already and we don't need to look up sides.
      //StaticGraphUtility.Assert((this.VisGraph.FindVertex(inboundsLocation) != null), "graph corner vertex not found", this.ObstacleTree, this.VisGraph);
      freePoint.AddOobEdgesFromGraphCorner(this.TransUtil, inboundsLocation)
      return
    }

    // We know inboundsLocation is on the nearest graphBox border ScanSegment, so this won't return a
    // null edge, and we'll just do normal join-to-one-edge handling, extending in the direction to the graph.
    let inboundsVertex = this.VisGraph.FindVertex(inboundsLocation)
    const dirToGraph = CompassVector.OppositeDir(dirFromGraph)
    if (inboundsVertex != null) {
      freePoint.AddToAdjacentVertex(this.TransUtil, inboundsVertex, dirToGraph, this.portSpliceLimitRectangle)
    } else {
      const edge = this.FindorCreateNearestPerpEdgePPDN(oobLocation, inboundsLocation, dirFromGraph, ScanSegment.NormalWeight)
      if (edge != null) {
        inboundsVertex = freePoint.AddEdgeToAdjacentEdge(this.TransUtil, edge, dirToGraph, this.portSpliceLimitRectangle)
      }
    }

    // This may be an oob waypoint, in which case we want to add additional edges so we can
    // go outside graph, cross the waypoint, and come back in.  Shortest-paths will do the
    // work of determining the optimal path, to avoid backtracking.
    const inboundsLeftVertex = StaticGraphUtility.FindAdjacentVertex(inboundsVertex, CompassVector.RotateLeft(dirToGraph))
    if (inboundsLeftVertex != null) {
      this.TransUtil.ConnectVertexToTargetVertex(freePoint.Vertex, inboundsLeftVertex, dirToGraph, ScanSegment.NormalWeight)
    }

    const inboundsRightVertex = StaticGraphUtility.FindAdjacentVertex(inboundsVertex, CompassVector.RotateRight(dirToGraph))
    if (inboundsRightVertex != null) {
      this.TransUtil.ConnectVertexToTargetVertex(freePoint.Vertex, inboundsRightVertex, dirToGraph, ScanSegment.NormalWeight)
    }
  }

  private ConnectFreePointToLateralEdge(freePoint: FreePoint, lateralDir: Direction) {
    // Turn on pivot vertex to either side to find the next edge to connect to.  If the freepoint is
    // overlapped (inside an obstacle), just find the closest ScanSegment outside the obstacle and
    // start extending from there; otherwise, we can have the FreePoint calculate its max visibility.
    const end = freePoint.IsOverlapped
      ? this.InBoundsGraphBoxIntersect(freePoint.Point, lateralDir)
      : freePoint.MaxVisibilityInDirectionForNonOverlappedFreePoint(lateralDir, this.TransUtil)
    const lateralEdge = this.FindorCreateNearestPerpEdgePPDN(end, freePoint.Point, lateralDir, freePoint.InitialWeight)
    // There may be no VisibilityEdge between the current point and any adjoining obstacle in that direction.
    if (lateralEdge != null) {
      freePoint.AddEdgeToAdjacentEdge(this.TransUtil, lateralEdge, lateralDir, this.portSpliceLimitRectangle)
    }
  }
}
