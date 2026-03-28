# msagl-rust — Agent Instructions

## What This Is

A Rust port of Microsoft's MSAGL RectilinearEdgeRouter. This is an open-source crate for orthogonal edge routing with guaranteed edge separation.

**Repo:** https://github.com/ByteBard97/msagl-rust

## Current State

The crate has a working skeleton (~6,300 lines, 261 tests) but **multiple components were simplified instead of faithfully ported**. A detailed defect list and remediation PRD lives at `docs/PRD-faithful-port-remediation.md`. Read it before doing anything.

## The Job

Execute the remediation PRD. Replace all simplified components with faithful ports from the TypeScript and C# reference implementations.

## Reference Sources

- **TypeScript (primary structure):** `../SignalCanvasFrontend/reference/msagl-js/modules/core/src/routing/`
- **C# (authoritative algorithms):** `../MSAGL-Reference/GraphLayout/MSAGL/Routing/Rectilinear/`
- **C# routing tests:** `../MSAGL-Reference/GraphLayout/Test/MSAGLTests/Rectilinear/RectilinearTests.cs` (5,544 lines, 242 test methods)
- **C# solver fixtures:** `../MSAGL-Reference/GraphLayout/Test/MSAGLTests/Resources/Constraints/ProjectionSolver/Data/` (86 files)

All paths are relative to `/Users/ceres/Desktop/SignalCanvas/`.

## Compliance Gate

**Run before every session and before every commit:**

```bash
bash tools/compliance-check.sh
```

This must exit 0. If it fails, fix everything it reports before doing any other work.
A pre-commit hook enforces this automatically. A Claude Code hook blocks banned patterns on every file write.

<banned_patterns>
These patterns are BLOCKED by the write hook and will fail the compliance check.
Finding them in a PR means the work is not done.

IN src/:
  - todo!()
  - unimplemented!()
  - // TODO
  - // FIXME
  - // SIMPLIFIED

IN src/routing/:
  - "bbox approximat" (any form of bounding box approximation for polygon obstacles)
  - add_nonrect_placeholder (banned helper — tests must use b.add_polygon())
  - "placeholder for non-rect"

IN tests/:
  - #[ignore] (zero tolerance — fix the implementation, not the test)
  - add_nonrect_placeholder
</banned_patterns>

<non_rectangular_obstacles>
Non-rectangular (polygon) obstacles MUST be ported using actual ICurve/polygon routing.
The bounding box approximation that was in this codebase is WRONG and has been removed.

CORRECT:   b.add_polygon(&[Point::new(x1,y1), Point::new(x2,y2), ...])
FORBIDDEN: b.add_rectangle_bl(bbox_x, bbox_y, bbox_w, bbox_h)  // as a stand-in for polygon

If a test using polygon obstacles fails because the VG generator does not yet support
polygon sweep events: that failure is CORRECT and HONEST. Do NOT hide it with #[ignore].
Do NOT substitute a bounding box. The VG generator must be fixed.
</non_rectangular_obstacles>

## Rules — READ THESE CAREFULLY

### 1. FAITHFUL PORT MEANS FAITHFUL

Port the algorithms **line-for-line** from the TypeScript source. Do not simplify, rewrite, or invent alternative approaches. If the TS implementation uses a sweep-line with reflection events, the Rust implementation uses a sweep-line with reflection events. Period.

If you think something can be simplified, **stop and ask**. Do not decide unilaterally.

### 2. Approved Deviations (Language Adaptations Only)

These are the ONLY ways the Rust code may differ from TS/C#:

| TS/C# Pattern | Rust Adaptation |
|---------------|-----------------|
| Object references with GC | Index-based arenas (`Vec<T>` + `usize` indices) |
| Class hierarchies (`extends`) | Enums with variants |
| `RBTree<T>` with cursor navigation | `BTreeMap` with `range()` queries |
| `Dictionary<Point, V>` | `HashMap<Point, V>` with `OrderedFloat` |
| `abstract class` + subclasses | Enum dispatch or trait + impl |
| `null` references | `Option<T>` |
| Custom spatial index | `rstar` crate |
| Custom linked list | `SlotMap` or `Vec` with index links |

If you encounter a pattern not in this table that seems like it needs a Rust adaptation, **ask before proceeding**.

### 3. Use Off-the-Shelf Rust Crates

Where a well-tested Rust crate already solves a problem, use it:
- `rstar` for R-tree spatial indexing
- `ordered-float` for hashable/orderable f64
- `slotmap` for arena allocation
- `kurbo` for arc/curve geometry
- Standard library `BTreeMap`, `BinaryHeap`, `HashMap` where they fit

Do NOT port custom data structures that have standard Rust equivalents.

### 4. Port the Tests

The C# repo has 242 procedural routing test methods. These are the acceptance criteria. Port them as Rust `#[test]` functions. If you need golden baselines, build a C# dump harness first.

### 5. Never Use `rm`

Use `trash` instead. No exceptions.

### 6. Files Under 500 Lines

Split if necessary. 700 absolute max.

### 7. Commit Frequently, Push Often

Commit after each component. Push at least after each major piece.

### 8. When In Doubt, Read the TS Source

The answer to "how should this work?" is always in the TypeScript or C# source. Read it. Don't guess.

### 9. SKELETON-FIRST WORKFLOW FOR ALGORITHM PORTING

**This is mandatory for any file rated SIMPLIFIED or DIVERGENT in the algorithmic parity audit.**

LLM agents have a documented tendency to substitute brute-force approximations for complex algorithms, even when explicitly told not to. Research shows this is a default behavior affecting ~50% of code translations when measured by efficiency. Saying "do not take shortcuts" does NOT prevent it — negative prompts actually make the problem worse.

The mitigation that works is structural: **lock in the algorithm before the agent writes implementation code.**

**Required workflow:**

1. **Human or supervising agent writes a skeleton file** with:
   - All method signatures matching the C# reference (with C# line numbers in comments)
   - All data structure fields with types already chosen (BTreeMap, BinaryHeap, etc.)
   - `todo!()` bodies for each method
   - Comments on every tree/map operation saying "MUST use BTreeMap range query — NOT linear scan"

2. **Implementing agent fills in only the `todo!()` bodies** by reading the C# reference
   - MUST NOT change method signatures, field types, or data structures
   - MUST NOT add new fields or methods beyond the skeleton
   - MUST run `cargo bench` and verify performance is equal to or better than before

3. **Verification:** A fresh-context agent reviews the diff against C# reference, specifically checking that no tree operation was replaced with a linear scan

**Why this works:** The skeleton locks in O(n log n) data structures. The agent cannot substitute O(n²) brute-force because the BTreeMap fields are already defined — there's no Vec to iterate over.

### 10. USE POSITIVE DIRECTIVES, NOT NEGATIVE ONES

Research shows LLMs perform WORSE with negative instructions ("do NOT simplify", "do NOT take shortcuts"). Instead, use affirmative directives that name the exact algorithm and data structures:

**Bad:** "Do NOT use brute force. Do NOT use nested for loops."
**Good:** "MUST implement sweep-line using BTreeMap<SideKey, ObstacleSide> for active obstacle trees. MUST use BTreeMap range() queries for FindFirst/FindLast operations. MUST process events via BinaryHeap in priority order."

### 11. Performance Verification is Mandatory

Every algorithm port MUST include a `cargo bench` comparison before and after. If the new implementation is slower than the old one on the large benchmark (210 obstacles, 500 edges), it has a bug. Fix the bug — do not ship slower code.

## Acceptance Criteria

1. All 86 projection solver fixture tests pass (currently 80/86 — fix the 6 failures)
2. 242 procedural routing tests from RectilinearTests.cs ported and passing
3. Every TS file in `routing/rectilinear/` has a Rust equivalent (except deferred: SparseVG, InteractiveEditor, Groups, non-floating ports)
4. `cargo test` passes, `cargo clippy -- -D warnings` is clean
5. Crate compiles to ~12,000-14,000 lines matching the original spec estimate

## Priority Order

Work through the PRD defects in this order:

1. **VisibilityVertex + VertexEntry[4]** — everything depends on this
2. **Visibility Graph Generator** — full sweep-line rewrite
3. **Path Search (SsstRectilinearPath)** — needs VertexEntry
4. **Port Manager + TransientGraphUtility** — needs proper VG
5. **Nudging expansions** (PathMerger, LinkedPointSplitter, full FreeSpaceFinder)
6. **6 solver fixture fixes**
7. **Port the 242 routing tests**
