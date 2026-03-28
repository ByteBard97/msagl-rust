  private ConnectToGraphAtPointsCollinearWithVertices(
    oobWaypoint: FreePoint,
    dirFromGraph: Direction,
    endpointVertices: Array<VisibilityVertex>,
  ) {
    if (Direction.None === dirFromGraph) {
      // Not out of bounds on this axis.
      return
    }

    const dirToGraph = CompassVector.OppositeDir(dirFromGraph)
    for (const vertex of endpointVertices) {
      const graphBorderLocation = this.InBoundsGraphBoxIntersect(vertex.point, dirFromGraph)
      const graphBorderVertex = this.VisGraph.FindVertex(graphBorderLocation)
      if (graphBorderVertex != null) {
        this.TransUtil.ConnectVertexToTargetVertex(oobWaypoint.Vertex, graphBorderVertex, dirToGraph, ScanSegment.NormalWeight)
      }
    }
  }

  SetAllAncestorsActive(edgeGeom: GeomEdge, shapeToObstacleMap: Map<Shape, Obstacle>): boolean {
    if (0 === this.AncestorSets.size) {
      return false
    }

    this.ObstacleTree.AdjustSpatialAncestors()
    this.ClearActiveAncestors()
    const t: {oport: ObstaclePort} = {oport: null}
    const s: {oport: ObstaclePort} = {oport: null}
    const ssAncs = this.FindAncestorsAndObstaclePort(edgeGeom.sourcePort, s)
    const ttAncs = this.FindAncestorsAndObstaclePort(edgeGeom.targetPort, t)
    if (this.AncestorSets.size > 0 && ssAncs != null && ttAncs != null) {
      // Make all ancestors boundaries transparent; in this case we've already tried with only
      // non-common and found no path, so perhaps an obstacle is outside its parent group's bounds.
      this.ActivateAncestors(ssAncs, ttAncs, shapeToObstacleMap)
      return true
    }

    return false
  }

  SetAllGroupsActive() {
    // We couldn't get a path when we activated all hierarchical and spatial group ancestors of the shapes,
    // so assume we may be landlocked and activate all groups, period.
    this.ClearActiveAncestors()
    for (const group of this.ObstacleTree.GetAllGroups()) {
      group.IsTransparentAncestor = true
      this.activeAncestors.push(group)
    }
  }

  FindAncestorsAndObstaclePort(port: Port, t: {oport: ObstaclePort}): Set<Shape> {
    t.oport = this.FindObstaclePort(port)
    if (0 === this.AncestorSets.size) {
      return null
    }

    if (t.oport != null) {
      return this.AncestorSets.get(t.oport.Obstacle.InputShape)
    }

    // This is a free Port (not associated with an obstacle) or a Waypoint; return all spatial parents.
    return new Set<Shape>(
      Array.from(this.ObstacleTree.Root.AllHitItems(Rectangle.mkPP(port.Location, port.Location), (shape) => shape.IsGroup)).map(
        (obs) => obs.InputShape,
      ),
    )
  }

  private ActivateAncestors(ssAncsToUse: Set<Shape>, ttAncsToUse: Set<Shape>, shapeToObstacleMap: Map<Shape, Obstacle>) {
    for (const shape of uniteSets(ssAncsToUse, ttAncsToUse)) {
      const group = shapeToObstacleMap.get(shape)
      /*Assert.assert(group.IsGroup, 'Ancestor shape is not a group')*/
      group.IsTransparentAncestor = true
      this.activeAncestors.push(group)
    }
  }

  ClearActiveAncestors() {
    for (const group of this.activeAncestors) {
      group.IsTransparentAncestor = false
    }

    this.activeAncestors = []
  }

  RemoveControlPointsFromGraph() {
    this.ClearActiveAncestors()
    this.RemoveObstaclePortsFromGraph()
    this.RemoveFreePointsFromGraph()
    this.TransUtil.RemoveFromGraph()
    this.portSpliceLimitRectangle = Rectangle.mkEmpty()
  }

  private RemoveObstaclePortsFromGraph() {
    for (const oport of this.obstaclePortsInGraph) {
      oport.RemoveFromGraph()
    }

    this.obstaclePortsInGraph = []
  }

  private RemoveFreePointsFromGraph() {
    for (const freePoint of this.freePointsInGraph) {
      freePoint.RemoveFromGraph()
    }

    this.freePointsInGraph.clear()
  }

  private RemoveStaleFreePoints() {
    // FreePoints are not necessarily persistent - they may for example be waypoints which are removed.
    // So after every routing pass, remove any that were not added to the graph. Because the FreePoint has
    // be removed from the graph, its Vertex (and thus Point) are no longer set in the FreePoint, so we
    // must use the key from the dictionary.
    if (this.freePointMap.size > this.freePointLocationsUsedByRouteEdges.size) {
      const staleFreePairs = Array.from(this.freePointMap).filter((p) => !this.freePointLocationsUsedByRouteEdges.has(p[0]))
      for (const staleFreePair of staleFreePairs) {
        this.freePointMap.deleteP(staleFreePair[0])
      }
    }
  }

  ClearVisibility() {
    // Most of the retained freepoint stuff is about precalculated visibility.
    this.freePointMap.clear()
    for (const oport of this.obstaclePortMap.values()) {
      oport.ClearVisibility()
    }
  }

  BeginRouteEdges() {
    this.RemoveControlPointsFromGraph()
    // ensure there are no leftovers
    this.freePointLocationsUsedByRouteEdges.clear()
  }

  EndRouteEdges() {
    this.RemoveStaleFreePoints()
  }

  FindObstaclePort(port: Port): ObstaclePort {
    let oport: ObstaclePort = this.obstaclePortMap.get(port)
    if (oport) {
      // First see if the obstacle's port list has changed without UpdateObstacles() being called.
      // Unfortunately we don't have a way to update the obstacle's ports until we enter
      // this block; there is no direct Port->Shape/Obstacle mapping.  So UpdateObstacle must still
      // be called, but at least this check here will remove obsolete ObstaclePorts.
      const t: {removedPorts: Set<Port>; addedPorts: Set<Port>} = {
        removedPorts: null,
        addedPorts: null,
      }
      if (oport.Obstacle.GetPortChanges(t)) {
        for (const newPort of t.addedPorts) {
          this.CreateObstaclePort(oport.Obstacle, newPort)
        }

        for (const oldPort of t.removedPorts) {
          this.RemoveObstaclePort(oldPort)
        }

        // If it's not still there, it was moved outside the obstacle so we'll just add it as a FreePoint.
        oport = this.obstaclePortMap.get(port)
      }
    }

    return oport
  }

