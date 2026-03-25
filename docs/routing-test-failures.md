# Routing Test Failure Investigation

Investigated on 2026-03-25. Both tests exercise the same underlying failure mode
(PRD defect #2 — the VG generator does not place vertices at port Y/X coordinates,
so port splicing fails and the router degrades to a straight-line fallback).

---

## obstacle_in_between

**Setup:**
- `o0` rectangle centered at (0, 50), 60×60 → bounds left=-30, right=30, bottom=20, top=80
- `o1` rectangle centered at (300, 50), 60×60 → bounds left=270, right=330, bottom=20, top=80
- `blocker` rectangle centered at (150, 50), 60×60 → bounds left=120, right=180, bottom=20, top=80
- Padding = 1.0 (padded blocker: x∈[119,181], y∈[19,81])
- One edge routed between the centers of o0 and o1: source=(0,50), target=(300,50)

**Expected:** Path bends above or below the blocker with at least one bend, e.g.:
```
(0,50) → (0,19) → (300,19) → (300,50)   # below the blocker
```
or an equivalent detour above y=81.

**Actual:**
```
thread panicked: Edge 0 segment 0 passes through obstacle 2:
  midpoint (150, 50) is inside padded bbox [119, 19, 181, 81]
```
The router emits the two-point fallback path `[(0,50), (300,50)]` — a straight horizontal
line that punches straight through the blocker.

**Root cause — two compounding failures:**

1. **VG generator does not emit segments at port Y coordinates.**
   The horizontal sweep pass creates horizontal scan segments only at obstacle event rows
   (the bottom and top of each padded bounding box). For this scenario those rows are
   y=19, y=21, y=79, y=81 (and the sentinel rows). y=50 — the port row — is never an
   event row, so no horizontal VG segment exists at y=50. Because there are no VG
   vertices at y=50, `PortManager::find_nearest_aligned` returns `None` in every
   direction for the port at (0,50).

2. **Port splicing silently degrades to a zero-neighbor splice.**
   When `find_nearest_aligned` returns `None` for all four directions, the splice adds the
   port vertex to the graph but attaches no edges. `PathSearch::find_path` then finds
   the source vertex in isolation (no out-edges, no in-edges from other vertices on the
   same row), returns `None`, and `rectilinear_edge_router.rs:131` falls back to
   `vec![source, target]`.

**Fix needed:**

The correct fix, as documented in PRD defect #2 (Visibility Graph Generator), is to
implement the faithful sweep-line algorithm from `VisibilityGraphGenerator.ts`. The TS
generator fires *additional* horizontal (and vertical) segments at every obstacle-corner
X/Y coordinate throughout the whole graph box — not just at each obstacle's own row.
This ensures that for any port location the splicing step can always find a neighbouring
VG vertex to attach to.

Specifically, the key missing piece is that the sweep line, on encountering an open or
close vertex event, must look left and right (for a horizontal sweep) to the nearest
scanline neighbours and emit segments that span the entire gap between those neighbours
at that sweep position. The current implementation does this correctly inside each
obstacle's own column, but it does not propagate segments horizontally across the full
graph width at the event Y coordinates of obstacles that are not in the same column as
the ports.

A simpler short-term workaround (not faithful to the TS, but would make these tests
pass) would be for `PortManager::splice_port` to emit the port's own horizontal and
vertical rays through the entire graph bounding box, clipped by the nearest obstacle
boundary on each side. This is essentially what `TransientGraphUtility.ts` does via
`ObstaclePortEntrance` objects. However, the PRD requires the faithful port, so the
right path is the VG rewrite.

---

## vertically_stacked_obstacles

**Setup:**
- `o0` rectangle centered at (100, 0), 80×40 → bounds left=60, right=140, bottom=-20, top=20
- `middle` rectangle centered at (100, 100), 80×40 → bounds left=60, right=140, bottom=80, top=120
- `o2` rectangle centered at (100, 200), 80×40 → bounds left=60, right=140, bottom=180, top=220
- Padding = 1.0 (padded middle: x∈[59,141], y∈[79,121])
- One edge routed between the centers of o0 and o2: source=(100,0), target=(100,200)

**Expected:** Path goes around the middle obstacle horizontally, e.g.:
```
(100,0) → (59,0) → (59,200) → (100,200)   # left side of middle
```
or an equivalent detour to the right (x=141).

**Actual:**
```
thread panicked: Edge 0 segment 0 passes through obstacle 1:
  midpoint (100, 100) is inside padded bbox [59, 79, 141, 121]
```
The router emits the two-point fallback path `[(100,0), (100,200)]` — a straight
vertical line passing through the middle obstacle.

**Root cause — identical failure mode to `obstacle_in_between`, but on the vertical axis.**

1. **VG generator does not emit segments at port X coordinates.**
   The vertical sweep pass creates vertical scan segments only at obstacle event columns
   (left and right edges of each padded bounding box). For this scenario those columns
   are x=59, x=61, x=139, x=141 and the sentinels. x=100 — the port column — is an
   event column only for the sentinels, not for the obstacles, so no vertical VG segment
   spanning y=[-20,220] exists at x=100 (each obstacle's own column segment only
   reaches the obstacle's own boundary, not the ports' Y positions).

   More precisely: the vertical segment at x=100 that the sweep would naturally generate
   spans from the top of o0's padded box (y=21) to the bottom of middle's padded box
   (y=79), and from middle's top (y=121) to o2's bottom (y=179). Neither of those
   segments contains y=0 (inside o0) or y=200 (inside o2), so `find_nearest_aligned`
   finds no vertex to the North from (100,0) or to the South from (100,200).

2. **Same silent splice degradation as above** — zero-neighbor splice → `find_path`
   returns `None` → fallback straight line.

**Fix needed:**

Same root-cause fix as `obstacle_in_between`: implement the faithful VG generator from
`VisibilityGraphGenerator.ts` (PRD defect #2). The correct generator emits vertical
segments from the graph bounding box's bottom sentinel to each obstacle's bottom edge,
and from each obstacle's top edge to the next obstacle's bottom (or the top sentinel),
at every obstacle-boundary X column. This creates a complete rectilinear grid around
all obstacles, with the only gaps being the obstacles' own interiors.

Additionally, the `PortManager` rewrite (PRD defect #4 — `PortManager.ts` + `TransientGraphUtility.ts`)
must correctly handle the case where a port lies *inside* its own obstacle's bounding
box (the port is at the obstacle center, which is inside the padded bbox). The faithful
TS `PortManager` uses `ObstaclePortEntrance` to fire a ray from the port outward through
the obstacle boundary and splice it into the nearest VG segment beyond that boundary.
The current simplified port manager does not do this — it only looks for existing VG
vertices on the same axis, which do not exist at the port's coordinates.

---

## Summary Table

| Test | Failure message | Primary broken component | Secondary broken component |
|------|-----------------|--------------------------|----------------------------|
| `obstacle_in_between` | Segment midpoint (150,50) inside blocker | VG generator: no horizontal segments at port Y=50 | Port manager: no edge splitting / ray casting from port |
| `vertically_stacked_obstacles` | Segment midpoint (100,100) inside middle | VG generator: no vertical segment at x=100 spanning port Y coords | Port manager: no ObstaclePortEntrance ray through obstacle boundary |

Both tests will pass once PRD defects #1 (VG Generator rewrite) and #2 (Port Manager +
TransientGraphUtility rewrite) are implemented in the order specified by the PRD priority
list (VG first, then Port Manager).
