  private InBoundsGraphBoxIntersect(point: Point, dir: Direction): Point {
    return StaticGraphUtility.RectangleBorderIntersect(this.graphGenerator.ObstacleTree.GraphBox, point, dir)
  }

  FindorCreateNearestPerpEdgePPDN(first: Point, second: Point, dir: Direction, weight: number): VisibilityEdge {
    const t: {targetVertex: VisibilityVertex} = {targetVertex: null}
    return this.FindorCreateNearestPerpEdgePPDNT(first, second, dir, weight, t)
  }

  private FindorCreateNearestPerpEdgePPDNT(
    first: Point,
    second: Point,
    dir: Direction,
    weight: number,
    t: {targetVertex: VisibilityVertex},
  ): VisibilityEdge {
    // Find the closest perpendicular ScanSegment that intersects a segment with endpoints
    // first and second, then find the closest parallel ScanSegment that intersects that
    // perpendicular ScanSegment.  This gives us a VisibilityVertex location from which we
    // can walk to the closest perpendicular VisibilityEdge that intersects first->second.
    const couple = StaticGraphUtility.SortAscending(first, second)
    const low: Point = couple[0]
    const high: Point = couple[1]
    const perpendicularScanSegments: ScanSegmentTree = StaticGraphUtility.IsVerticalD(dir) ? this.HScanSegments : this.VScanSegments
    // Look up the nearest intersection.  For obstacles, we cannot just look for the bounding box
    // corners because nonrectilinear obstacles may have other obstacles overlapping the bounding
    // box (at either the corners or between the port border intersection and the bounding box
    // side), and of course obstacles may overlap too.
    const nearestPerpSeg: ScanSegment = StaticGraphUtility.IsAscending(dir)
      ? perpendicularScanSegments.FindLowestIntersector(low, high)
      : perpendicularScanSegments.FindHighestIntersector(low, high)
    if (nearestPerpSeg == null) {
      // No ScanSegment between this and visibility limits.
      t.targetVertex = null
      return null
    }

    const edgeIntersect: Point = StaticGraphUtility.SegmentIntersectionSP(nearestPerpSeg, low)
    // We now know the nearest perpendicular segment that intersects start->end.  Next we'll find a close
    // parallel scansegment that intersects the perp segment, then walk to find the nearest perp edge.
    return this.FindOrCreateNearestPerpEdgeFromNearestPerpSegment(
      StaticGraphUtility.IsAscending(dir) ? low : high,
      nearestPerpSeg,
      edgeIntersect,
      weight,
      t,
    )
  }

  private FindOrCreateNearestPerpEdgeFromNearestPerpSegment(
    pointLocation: Point,
    scanSeg: ScanSegment,
    edgeIntersect: Point,
    weight: number,
    t: {targetVertex: VisibilityVertex},
  ): VisibilityEdge {
    // Given: a ScanSegment scanSeg perpendicular to pointLocation->edgeIntersect and containing edgeIntersect.
    // To find: a VisibilityEdge perpendicular to pointLocation->edgeIntersect which may be on scanSeg, or may
    //          be closer to pointLocation than the passed edgeIntersect is.
    // Since there may be TransientEdges between pointLocation and edgeIntersect, we start by finding
    // a scanSeg-intersecting (i.e. parallel to pointLocation->edgeIntersect) ScanSegment, then starting from
    // the intersection of those segments, walk the VisibilityGraph until we find the closest VisibilityEdge
    // perpendicular to pointLocation->edgeIntersect.  If there is a vertex on that edge collinear to
    // pointLocation->edgeIntersect, return the edge for which it is Source, else split the edge.
    // If there is already a vertex at edgeIntersect, we do not need to look for the intersecting ScanSegment.
    const tt: {
      segsegVertex: VisibilityVertex
      targetVertex: VisibilityVertex
    } = {
      segsegVertex: this.VisGraph.FindVertex(edgeIntersect),
      targetVertex: null,
    }
    if (tt.segsegVertex == null) {
      const edge = this.FindOrCreateSegmentIntersectionVertexAndAssociatedEdge(pointLocation, edgeIntersect, scanSeg, weight, tt)
      if (edge != null) {
        return edge
      }
    } else if (PointComparer.EqualPP(pointLocation, edgeIntersect)) {
      // The initial pointLocation was on scanSeg at an existing vertex so return an edge
      // from that vertex along scanSeg. Look in both directions in case of dead ends.
      t.targetVertex = tt.segsegVertex
      return this.TransUtil.FindNextEdge(t.targetVertex, CompassVector.OppositeDir(scanSeg.ScanDirection.Dir))
    }

    // pointLocation is not on the initial scanSeg, so see if there is a transient edge between
    // pointLocation and edgeIntersect.  edgeIntersect === segsegVertex.Point if pointLocation is
    // collinear with intSegBefore (pointLocation is before or after intSegBefore's VisibilityVertices).
    const dirTowardLocation: Direction = PointComparer.GetDirections(edgeIntersect, pointLocation)
    let perpDir: Direction = PointComparer.GetDirections(tt.segsegVertex.point, pointLocation)
    if (dirTowardLocation === perpDir) {
      // intSegBefore is collinear with pointLocation so walk to the vertex closest to pointLocation.
      const ot: {
        bracketTarget: VisibilityVertex
        bracketSource: VisibilityVertex
      } = {bracketTarget: null, bracketSource: null}
      TransientGraphUtility.FindBracketingVertices(tt.segsegVertex, pointLocation, dirTowardLocation, ot)
      // Return an edge. Look in both directions in case of dead ends.
      return (
        this.TransUtil.FindNextEdge(ot.bracketSource, CompassVector.RotateLeft(dirTowardLocation)) ??
        this.TransUtil.FindNextEdge(ot.bracketSource, CompassVector.RotateRight(dirTowardLocation))
      )
    }

    // Now make perpDir have only the perpendicular component.
    perpDir &= ~dirTowardLocation // if this is Directions. None, pointLocation === edgeIntersect
    // StaticGraphUtility.Assert((Direction.None !== perpDir), "pointLocation === initial segsegVertex.Point should already have exited", this.ObstacleTree, this.VisGraph);
    // Other TransientVE edge chains may have been added between the control point and the
    // ScanSegment (which is always non-transient), and they may have split ScanSegment VEs.
    // Fortunately we know we'll always have all transient edge chains extended to or past any
    // control point (due to LimitRectangle), so we can just move up lowestIntSeg toward
    // pointLocation, updating segsegVertex and edgeIntersect.  There are 3 possibilities:
    //  - location is not on an edge - the usual case, we just create an edge perpendicular
    //    to an edge on scanSeg, splitting that scanSeg edge in the process.
    //  - location is on a VE that is parallel to scanSeg.  This is essentially the same thing
    //    but we don't need the first perpendicular edge to scanSeg.
    //  - location is on a VE that is perpendicular to scanSeg.  In that case the vertex on ScanSeg
    //    already exists; TransUtil.FindOrAddEdge just returns the edge starting at that intersection.
    // FreePoint tests of this are in RectilinearTests.FreePortLocationRelativeToTransientVisibilityEdges*.
    const perpendicularEdge: VisibilityEdge = this.TransUtil.FindNearestPerpendicularOrContainingEdge(
      tt.segsegVertex,
      perpDir,
      pointLocation,
    )
    if (perpendicularEdge == null) {
      // Dead end; we're above the highest point at which there is an intersection of scanSeg.
      // Create a new vertex and edge higher than the ScanSegment's HighestVisibilityVertex
      // if that doesn't cross an obstacle (if we are between two ScanSegment dead-ends, we may).
      // We hit this in RectilinearFileTests.Nudger_Many_Paths_In_Channel and .Nudger_Overlap*.
      // StaticGraphUtility.Assert((edgeIntersect > scanSeg.HighestVisibilityVertex.point), "edgeIntersect is not > scanSeg.HighestVisibilityVertex", this.ObstacleTree, this.VisGraph);
      t.targetVertex = this.TransUtil.AddVertex(edgeIntersect)
      return this.TransUtil.FindOrAddEdge(t.targetVertex, scanSeg.HighestVisibilityVertex, scanSeg.Weight)
    }

    // We have an intersecting perp edge, which may be on the original scanSeg or closer to pointLocation.
    // Get one of its vertices and re-find the intersection on it (it doesn't matter which vertex of the
    // edge we use, but for consistency use the "lower in perpDir" one).
    tt.segsegVertex = StaticGraphUtility.GetEdgeEnd(perpendicularEdge, CompassVector.OppositeDir(perpDir))
    edgeIntersect = StaticGraphUtility.SegmentIntersectionPPP(pointLocation, edgeIntersect, tt.segsegVertex.point)
    // By this point we've verified there's no intervening Transient edge, so if we have an identical
    // point, we're done.
    if (PointComparer.EqualPP(tt.segsegVertex.point, edgeIntersect)) {
      t.targetVertex = tt.segsegVertex
      return this.TransUtil.FindNextEdge(tt.segsegVertex, perpDir)
    }

    // The targetVertex doesn't exist; this will split the edge and add it.
    t.targetVertex = this.TransUtil.FindOrAddVertex(edgeIntersect)
    return this.TransUtil.FindOrAddEdge(tt.segsegVertex, t.targetVertex, weight)
  }

  private FindOrCreateSegmentIntersectionVertexAndAssociatedEdge(
    pointLocation: Point,
    edgeIntersect: Point,
    scanSeg: ScanSegment,
    weight: number,
    t: {segsegVertex: VisibilityVertex; targetVertex: VisibilityVertex},
  ): VisibilityEdge {
    const intersectingSegments: ScanSegmentTree = scanSeg.IsVertical ? this.HScanSegments : this.VScanSegments
    const intSegBefore: ScanSegment = intersectingSegments.FindHighestIntersector(scanSeg.Start, edgeIntersect)
    if (intSegBefore == null) {
      // Dead end; we're below the lowest point at which there is an intersection of scanSeg.
      // Create a new vertex and edge lower than the ScanSegment's LowestVisibilityVertex.
      // Test: RectilinearFileTests.Overlap_Rotate_SplicePort_FreeObstaclePorts.
      t.segsegVertex = null
      t.targetVertex = this.TransUtil.AddVertex(edgeIntersect)
      return this.TransUtil.FindOrAddEdge(t.targetVertex, scanSeg.LowestVisibilityVertex, scanSeg.Weight)
    }

    // Get the VisibilityVertex at the intersection of the two segments we just found;
    // edgeIntersect is between that vertex and another on the segment, and we'll split
    // the edge between those two vertices (or find one nearer to walk to).
    const segsegIntersect: Point = StaticGraphUtility.SegmentsIntersection(scanSeg, intSegBefore)
    t.segsegVertex = this.VisGraph.FindVertex(segsegIntersect)
    if (!t.segsegVertex) {
      // This happens only for UseSparseVisibilityGraph; in that case we must create the
      // intersection vertex in the direction of both segments so we can start walking.
      t.segsegVertex = this.TransUtil.AddVertex(segsegIntersect)
      const newEdge = this.AddEdgeToClosestSegmentEnd(scanSeg, t.segsegVertex, scanSeg.Weight)
      this.AddEdgeToClosestSegmentEnd(intSegBefore, t.segsegVertex, intSegBefore.Weight)
      if (PointComparer.EqualPP(t.segsegVertex.point, edgeIntersect)) {
        t.targetVertex = t.segsegVertex
        return newEdge
      }
    }

    if (PointComparer.EqualPP(pointLocation, edgeIntersect)) {
      // The initial pointLocation was on scanSeg and we had to create a new vertex for it,
      // so we'll find or create (by splitting) the edge on scanSeg that contains pointLocation.
      t.targetVertex = this.TransUtil.FindOrAddVertex(edgeIntersect)
      return this.TransUtil.FindOrAddEdge(t.segsegVertex, t.targetVertex, weight)
    }

    t.targetVertex = null
    return null
  }

  private AddEdgeToClosestSegmentEnd(scanSeg: ScanSegment, segsegVertex: VisibilityVertex, weight: number): VisibilityEdge {
    // FindOrAddEdge will walk until it finds the minimal bracketing vertices.
    if (PointComparer.IsPureLower(scanSeg.HighestVisibilityVertex.point, segsegVertex.point)) {
      return this.TransUtil.FindOrAddEdge(scanSeg.HighestVisibilityVertex, segsegVertex, weight)
    }

    if (PointComparer.IsPureLower(segsegVertex.point, scanSeg.LowestVisibilityVertex.point)) {
      return this.TransUtil.FindOrAddEdge(segsegVertex, scanSeg.LowestVisibilityVertex, weight)
    }

    return this.TransUtil.FindOrAddEdgeVV(scanSeg.LowestVisibilityVertex, segsegVertex)
  }

