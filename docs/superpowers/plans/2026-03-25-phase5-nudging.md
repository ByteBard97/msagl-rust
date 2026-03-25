# Phase 5: Nudging Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the nudging pipeline that separates parallel edge segments using QPSC constraint solving.

**Architecture:** Six-stage pipeline: PathRefiner → CombinatorialNudger → FreeSpaceFinder → LongestNudgedSegment grouping → QPSC Solver → StaircaseRemover. All data uses index-based arenas owned by the Nudger. Uses UniformOneDimensionalSolver from Phase 2.

**Tech Stack:** Rust, existing geometry (Phase 1), projection solver (Phase 2), visibility graph (Phase 3).

**Porting sources:** TS: `routing/rectilinear/nudging/` (16 files, ~3,074 lines)

---

## File Structure

```
msagl-rust/src/routing/nudging/
├── mod.rs
├── nudger.rs              # Top-level orchestrator
├── path.rs                # Path + PathEdge types
├── axis_edge.rs           # AxisEdge (atomic rectilinear segment)
├── longest_nudged_segment.rs  # Maximal parallel segment clusters
├── path_refiner.rs        # Path normalization + intersection insertion
├── combinatorial_nudger.rs # Build DAG, determine path ordering
├── free_space_finder.rs   # Sweep-line bounds computation
└── staircase_remover.rs   # Post-processing simplification
```

---

### Task 1: Nudging Data Types (Path, PathEdge, AxisEdge, LongestNudgedSegment)

Create the core data structures. All use index-based arenas.

### Task 2: PathRefiner

Normalize paths: remove duplicate points, insert intersection points between crossing segments.

### Task 3: CombinatorialNudger

Build path ordering DAG using topological sort. Determine which path goes where on each axis edge.

### Task 4: FreeSpaceFinder

Sweep-line to compute LeftBound/RightBound for each axis edge.

### Task 5: Nudger Orchestrator

Wire the pipeline: PathRefiner → CombinatorialNudger → FreeSpaceFinder → create LongestNudgedSegments → create solver constraints → solve → apply positions.

### Task 6: StaircaseRemover

Pattern detection and simplification of zig-zag segments.

### Task 7: Integration Tests + Clippy + Push

---

## Summary

| Task | What | Est. Lines |
|------|------|-----------|
| 1 | Data types | ~300 |
| 2 | PathRefiner | ~150 |
| 3 | CombinatorialNudger | ~200 |
| 4 | FreeSpaceFinder | ~300 |
| 5 | Nudger orchestrator | ~400 |
| 6 | StaircaseRemover | ~150 |
| 7 | Tests + integration | ~200 |
| **Total** | | **~1,700** |
