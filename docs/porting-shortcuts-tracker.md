# Porting Shortcuts Tracker
**Last updated:** 2026-03-27

Issues where porting agents took architectural shortcuts or algorithmic shortcuts that deviate from the C# reference in ways that need to be fixed. Approved Rust adaptations (BTreeMap for RBTree, rstar for RectangleNode, etc.) are NOT listed here.

---

## CRITICAL — Structural / Architectural

### S1. PortManager is a stateless unit struct
**File:** `src/routing/port_splice.rs`, `src/routing/port_manager.rs`

C# `PortManager` is a class that owns its dependencies:
```csharp
class PortManager {
    TransientGraphUtility TransUtil;
    VisibilityGraphGenerator graphGenerator; // ObstacleTree, VG, H/V scan trees via this
}
```

Rust `PortManager` is `pub struct PortManager;` — no fields. All methods are static and receive `graph: &mut VisibilityGraph`, `obstacle_tree: &mut ObstacleTree`, etc. as parameters.

**What's lost:**
- No access to HScanSegments / VScanSegments (needed for O(log n) edge lookup during splice)
- No persistent TransientGraphUtility lifecycle — a new TGU is created per call and thrown away
- Port entrance state, group boundary crossings, and overlap tracking can't be cached

**Note:** `FullPortManager` struct is defined in `port_manager.rs` (lines 32–66) with the correct fields and never instantiated. The router calls the stateless `PortManager::splice_port` directly.

**Fix:** Either restore `FullPortManager` as the real PortManager and wire it into `RectilinearEdgeRouter`, or thread all dependencies correctly. Option A (proper struct) is correct; option B is a workaround that causes cascading parameter threading problems.

---

### S2. VisibilityGraphGenerator is a free function, not a struct
**File:** `src/routing/visibility_graph_generator.rs`

C# `FullVisibilityGraphGenerator` is a class that owns and stores:
- `ScanSegmentTree HorizontalScanSegments`
- `ScanSegmentTree VerticalScanSegments`
- `VisibilityGraph VisibilityGraph`
- `ObstacleTree ObsTree`

Rust `generate_visibility_graph()` is a free function that creates local scan segment trees and returns a `(VisibilityGraph, ObstacleTree)` tuple. The H/V scan segment trees are created, used, then dropped.

**What's lost:**
- PortManager can't access HScanSegments/VScanSegments (it doesn't have a reference)
- Segment trees can't be inspected for sparse VG optimization or interactive routing
- Architecture prevents the correct PortManager struct (S1) from working properly

**Fix:** Convert to a struct holding all four fields. The free function approach forces either S1 (stateless PortManager) or massive parameter threading.

---

### S3. TransientGraphUtility takes all dependencies as parameters
**File:** `src/routing/transient_graph_utility.rs`

C# `TransientGraphUtility` takes a `VisibilityGraphGenerator` in its constructor and accesses everything else through it. The Rust version takes `graph: &mut VisibilityGraph`, `obstacle_tree: &mut ObstacleTree`, etc. as parameters on every method call.

This is a downstream symptom of S1 and S2 — once the generator became a free function and PortManager lost its fields, everything had to be parameter-threaded. **S1 and S2 fix this as a side effect.**

---

## CRITICAL — Algorithmic Bugs

### A1. ~~`opposite_dir()` returns the same direction~~ — NOT A BUG
**File:** `src/routing/nudging/nudger.rs` line ~441

C# `AxisEdge.cs` has `Debug.Assert(Direction == Direction.East || Direction == Direction.North)` — AxisEdges are canonicalized to only North or East in both C# and Rust. The `|| edge_dir == opposite` branch in `create_longest_segments` can never be true in either implementation. `opposite_dir` returning the same value is correct. The comment is accurate. Remove from bug list.

---

### A2. Sweep-line event handler bugs (6 items)
**File:** `src/routing/vg_event_processing.rs`

These are documented in detail in `gemini-work-evaluation.md`. Summary:

| ID | Bug | Impact |
|----|-----|--------|
| C1 | O(n) linear scan to find active sides in bend/close handlers | O(n²) performance, wrong side if duplicates |
| C2 | `process_open_vertex` missing Block A (reflection loading) and Block B (flat-bottom removal) | Missing reflection scan segments |
| C3 | `process_close_vertex` missing `obstacle.close()`, side-swap, neighbor reflection loading, overlap drain | Second sweep pass uses stale sides |
| C4 | `process_low_bend` wrong else branch: adds fake High-type side to scanline | Scanline corruption |
| C5 | `ObstaclePortEntrance::extend_edge_chain` is a stub | **Direct cause of both failing tests** |
| C6 | Both low and high side neighbor searches use `low_in_sl` as ref_point | High side neighbors found at wrong coordinate |
| W1 | Bend handlers never call `set_active_low/high_side()` | Active side state stale after every bend |
| W2 | `add_perpendicular_reflection_segment` only handles staircase case | Most reflection segments suppressed |
| W5 | `prev_event_site` tracked as running loop variable, not stored on event at enqueue | Reflection events use wrong previous site |

**Fix order:** C4+W1 → C1 → C5 → C6 → C2/C3 (with explicit `obstacle.close()`) → W5 → W2 (hardest, separate task)

---

## WARNING — Performance / Incomplete Features

### W1. FreeSpaceFinder uses O(n·m) neighbor discovery instead of sweep
**File:** `src/routing/nudging/free_space_finder.rs`

A comment at the top acknowledges this explicitly: the sweep-line neighbor discovery is disabled in favor of `find_right_neighbors_only()` which is O(n·m). The C# uses the full O(n log n) sweep. Acceptable as a known tradeoff for now, but nudging quality degrades on complex layouts.

---

### W2. `shift_point` in nudger.rs may have inverted axes
**File:** `src/routing/nudging/nudger.rs` line ~379

`Direction::North` moves the X coordinate; `Direction::East` moves Y with negation. This appears inverted (North should move Y, East should move X). Needs cross-check against TS source before confirming.

---

## NOTE — Minor / Deferred

### N1. `obstacle.rs` exceeds 700-line hard cap
**File:** `src/routing/obstacle.rs` — 731 lines

CLAUDE.md Rule 6: 700 absolute max. Needs to be split. Natural split: `obstacle_sides.rs` for the scanline-related methods (`create_initial_sides`, `get_open_vertex`, active side accessors).

### N2. `scan_line.rs` CRITICAL-6 fix still pending
**File:** `src/routing/scan_line.rs`

`side_coordinate()` uses `start().x()` / `start().y()` instead of calling `scanline_intersect()`. This is a latent bug — correct for rectangular obstacles (all sides axis-aligned) but wrong for non-rectangular. Not blocking current tests.

### N3. `shift_point` North/East axes (see W2 above — needs verification)

---

## Status

| ID | Description | Priority | Status |
|----|-------------|----------|--------|
| S1 | PortManager stateless | HIGH | Open |
| S2 | VG generator free function | HIGH | Open |
| S3 | TGU parameter threading | HIGH | Blocked on S1/S2 |
| A1 | ~~opposite_dir() bug~~ | — | Not a bug — C# also canonicalizes to N/E only |
| A2-C5 | extend_edge_chain stub (2 test failures) | HIGH | Open |
| A2-C4 | process_low_bend wrong else | HIGH | Open |
| A2-W1 | bend handlers missing active side update | HIGH | Open |
| A2-C1 | O(n) scan in bend/close handlers | MEDIUM | Open |
| A2-C6 | Wrong ref_point for high side | MEDIUM | Open |
| A2-C2 | Missing open vertex blocks | MEDIUM | Open |
| A2-C3 | Missing close vertex blocks + obstacle.close() | MEDIUM | Open |
| A2-W5 | prev_event_site on event enum | MEDIUM | Open |
| A2-W2 | Reflection non-staircase case | LOW | Open |
| W1 | FreeSpaceFinder simplified | LOW | Open |
| W2 | shift_point axes | LOW | Needs verification |
| N1 | obstacle.rs over 700 lines | LOW | Open |
| N2 | scan_line CRITICAL-6 latent | LOW | Open (non-rectangular only) |
