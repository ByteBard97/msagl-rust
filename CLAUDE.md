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
