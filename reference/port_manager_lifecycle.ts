  Clear() {
    this.TransUtil.RemoveFromGraph()
    // Probably nothing in here when this is called
    this.obstaclePortMap.clear()
  }

  CreateObstaclePorts(obstacle: Obstacle) {
    // Create ObstaclePorts for all Ports of this obstacle.  This just creates the
    // ObstaclePort object; we don't add its edges/vertices to the graph until we
    // do the actual routing.
    for (const port of obstacle.Ports) {
      this.CreateObstaclePort(obstacle, port)
    }
  }

  private CreateObstaclePort(obstacle: Obstacle, port: Port): ObstaclePort {
    // This will replace any previous specification for the port (last one wins).
    /*Assert.assert(
      !this.obstaclePortMap.has(port),
      'Port is used by more than one obstacle',
    )*/
    if (port.Curve == null) {
      return null
    }

    const roundedLocation = Point.RoundPoint(port.Location)
    if (PointLocation.Outside === Curve.PointRelativeToCurveLocation(roundedLocation, obstacle.InputShape.BoundaryCurve)) {
      // Obstacle.Port is outside Obstacle.Shape; handle it as a FreePoint.
      return null
    }

    if (
      obstacle.InputShape.BoundaryCurve !== port.Curve &&
      PointLocation.Outside === Curve.PointRelativeToCurveLocation(roundedLocation, port.Curve)
    ) {
      // Obstacle.Port is outside port.Curve; handle it as a FreePoint.
      return null
    }

    const oport = new ObstaclePort(port, obstacle)
    this.obstaclePortMap.set(port, oport)
    return oport
  }

  FindVertices(port: Port): Array<VisibilityVertex> {
    const vertices = new Array<VisibilityVertex>()
    const oport: ObstaclePort = this.obstaclePortMap.get(port)
    if (oport) {
      if (this.RouteToCenterOfObstacles) {
        vertices.push(oport.CenterVertex)
      } else {
        // Add all vertices on the obstacle borders.  Avoid LINQ for performance.
        for (const entrance of oport.PortEntrances) {
          const vertex: VisibilityVertex = this.VisGraph.FindVertex(entrance.UnpaddedBorderIntersect)
          if (vertex != null) {
            vertices.push(vertex)
          }
        }
      }
    } else {
      vertices.push(this.VisGraph.FindVertex(Point.RoundPoint(port.Location)))
    }
    return vertices
  }

  RemoveObstaclePorts(obstacle: Obstacle) {
    for (const port of obstacle.Ports) {
      // Since we remove the port from the visibility graph after each routing, all we
      // have to do here is remove it from the dictionary.
      this.RemoveObstaclePort(port)
    }
  }

  RemoveObstaclePort(port: Port) {
    this.obstaclePortMap.delete(port)
  }

  // Add path control points - source, target, and any waypoints.
  AddControlPointsToGraph(edge: GeomEdge, shapeToObstacleMap: Map<Shape, Obstacle>) {
    this.GetPortSpliceLimitRectangle(edge)
    this.activeAncestors = []
    const s: {oport: ObstaclePort} = {oport: null}
    const t: {oport: ObstaclePort} = {oport: null}
    const ssAncs = this.FindAncestorsAndObstaclePort(edge.sourcePort, s)
    const ttAncs = this.FindAncestorsAndObstaclePort(edge.targetPort, t)
    if (this.AncestorSets.size > 0 && s.oport != null && t.oport != null) {
      // Make non-common ancestors' boundaries transparent (we don't want to route outside common ancestors).
      const ttAncsOnly = substractSets(ttAncs, ssAncs)
      const ssAncsOnly = substractSets(ssAncs, ttAncs)
      this.ActivateAncestors(ssAncsOnly, ttAncsOnly, shapeToObstacleMap)
    }

    // Now that we've set any active ancestors, splice in the port visibility.
    this.AddPortToGraph(edge.sourcePort, s.oport)
    this.AddPortToGraph(edge.targetPort, t.oport)
  }

  ConnectOobWaypointToEndpointVisibilityAtGraphBoundary(oobWaypoint: FreePoint, port: Port) {
    if (oobWaypoint == null || !oobWaypoint.IsOutOfBounds) {
      return
    }

    // Connect to the graphbox side at points collinear with the vertices.  The waypoint may be
    // OOB in two directions so call once for each axis.
    const endpointVertices = this.FindVertices(port)
    let dirFromGraph = oobWaypoint.OutOfBoundsDirectionFromGraph & (Direction.North | Direction.South)
    this.ConnectToGraphAtPointsCollinearWithVertices(oobWaypoint, dirFromGraph, endpointVertices)
    dirFromGraph = oobWaypoint.OutOfBoundsDirectionFromGraph & (Direction.East | Direction.West)
    this.ConnectToGraphAtPointsCollinearWithVertices(oobWaypoint, dirFromGraph, endpointVertices)
  }

