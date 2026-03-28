  private AddPortToGraph(port: Port, oport: ObstaclePort) {
    if (oport != null) {
      this.AddObstaclePortToGraph(oport)
      return
    }

    // This is a FreePoint, either a Waypoint or a Port not in an Obstacle.Ports list.
    this.AddFreePointToGraph(port.Location)
  }

  private AddObstaclePortToGraph(oport: ObstaclePort) {
    // If the port's position has changed without UpdateObstacles() being called, recreate it.
    if (oport.LocationHasChanged) {
      this.RemoveObstaclePort(oport.Port)
      oport = this.CreateObstaclePort(oport.Obstacle, oport.Port)
      if (oport == null) {
        // Port has been moved outside obstacle; return and let caller add it as a FreePoint.
        return
      }
    }

    oport.AddToGraph(this.TransUtil, this.RouteToCenterOfObstacles)
    this.obstaclePortsInGraph.push(oport)
    this.CreateObstaclePortEntrancesIfNeeded(oport)
    // We've determined the entrypoints on the obstacle boundary for each PortEntry,
    // so now add them to the VisGraph.
    for (const entrance of oport.PortEntrances) {
      this.AddObstaclePortEntranceToGraph(entrance)
    }

    return
  }

  private CreateObstaclePortEntrancesIfNeeded(oport: ObstaclePort) {
    if (oport.PortEntrances.length > 0) {
      return
    }

    // Create the PortEntrances with initial information:  border intersect and outer edge direction.
    this.CreateObstaclePortEntrancesFromPoints(oport)
  }

  public GetPortVisibilityIntersection(edgeGeometry: GeomEdge): Point[] {
    const sourceOport = this.FindObstaclePort(edgeGeometry.sourcePort)
    const targetOport = this.FindObstaclePort(edgeGeometry.targetPort)
    if (sourceOport == null || targetOport == null) {
      return null
    }

    if (sourceOport.Obstacle.IsInConvexHull || targetOport.Obstacle.IsInConvexHull) {
      return null
    }

    this.CreateObstaclePortEntrancesIfNeeded(sourceOport)
    this.CreateObstaclePortEntrancesIfNeeded(targetOport)
    if (!sourceOport.VisibilityRectangle.intersects(targetOport.VisibilityRectangle)) {
      return null
    }

    for (const sourceEntrance of sourceOport.PortEntrances) {
      if (!sourceEntrance.WantVisibilityIntersection) {
        continue
      }

      for (const targetEntrance of targetOport.PortEntrances) {
        if (!targetEntrance.WantVisibilityIntersection) {
          continue
        }

        const points =
          sourceEntrance.IsVertical === targetEntrance.IsVertical
            ? PortManager.GetPathPointsFromOverlappingCollinearVisibility(sourceEntrance, targetEntrance)
            : PortManager.GetPathPointsFromIntersectingVisibility(sourceEntrance, targetEntrance)

        if (points != null) {
          return points
        }
      }
    }

    return null
  }

  private static GetPathPointsFromOverlappingCollinearVisibility(
    sourceEntrance: ObstaclePortEntrance,
    targetEntrance: ObstaclePortEntrance,
  ): Point[] {
    // If the segments are the same they'll be in reverse.  Note: check for IntervalsOverlap also, if we support FreePoints here.
    if (
      !StaticGraphUtility.IntervalsAreSame(
        sourceEntrance.MaxVisibilitySegment.start,
        sourceEntrance.MaxVisibilitySegment.end,
        targetEntrance.MaxVisibilitySegment.end,
        targetEntrance.MaxVisibilitySegment.start,
      )
    ) {
      return null
    }

    if (sourceEntrance.HasGroupCrossings || targetEntrance.HasGroupCrossings) {
      return null
    }

    if (Point.closeDistEps(sourceEntrance.UnpaddedBorderIntersect, targetEntrance.UnpaddedBorderIntersect)) {
      // Probably one obstacle contained within another; we handle that elsewhere.
      return null
    }

    return [sourceEntrance.UnpaddedBorderIntersect, targetEntrance.UnpaddedBorderIntersect]
  }

  private static GetPathPointsFromIntersectingVisibility(
    sourceEntrance: ObstaclePortEntrance,
    targetEntrance: ObstaclePortEntrance,
  ): Point[] {
    const intersect: Point = StaticGraphUtility.SegmentsIntersectLL(
      sourceEntrance.MaxVisibilitySegment,
      targetEntrance.MaxVisibilitySegment,
    )
    if (!intersect) {
      return null
    }

    if (sourceEntrance.HasGroupCrossingBeforePoint(intersect) || targetEntrance.HasGroupCrossingBeforePoint(intersect)) {
      return null
    }

    return [sourceEntrance.UnpaddedBorderIntersect, intersect, targetEntrance.UnpaddedBorderIntersect]
  }

