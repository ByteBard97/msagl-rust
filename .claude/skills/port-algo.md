---
name: port-algo
description: Faithful algorithm port from C#/TS to Rust. Enforces 4-phase workflow with compliance gate. Use for any file rated SIMPLIFIED or DIVERGENT, or any routing algorithm port.
---

# Port Algorithm — Faithful C#/TS → Rust

You are porting an algorithm from the C# or TypeScript reference to Rust.
This is a **faithful port** — not a rewrite, not a simplification.
Line-for-line fidelity is required.

## Phase 1: Complexity Audit (READ FIRST, WRITE NOTHING)

Before writing any code:

1. Read the C# source file completely
2. Identify every data structure used: `SortedSet`, `Dictionary`, `Queue`, custom trees
3. For each data structure, record its Big-O contract:
   - `SortedSet<T>` → O(log n) insert/delete/find → Rust: `BTreeSet`
   - `SortedDictionary<K,V>` → O(log n) → Rust: `BTreeMap`
   - `Dictionary<K,V>` → O(1) average → Rust: `HashMap`
   - `PriorityQueue` / `SortedSet` as heap → Rust: `BinaryHeap`
   - Custom linked list with cursor → Rust: `BTreeMap` with range queries
4. Write a one-paragraph complexity summary before touching any Rust

<complexity_rules>
- Every O(log n) operation in C# MUST remain O(log n) in Rust
- Linear scan (for/while over Vec) inside a loop that runs N times = O(n²) — FORBIDDEN unless C# also does O(n²)
- BTreeMap range() queries are the correct replacement for RBTree cursor navigation
- No Vec where C# uses SortedSet/SortedDictionary
</complexity_rules>

## Phase 2: Skeleton (STRUCTURE BEFORE BODIES)

Write a skeleton file with:
- All method signatures matching C# (with C# line numbers in comments)
- All struct fields with types already chosen (BTreeMap, BinaryHeap, etc.)
- `todo!()` bodies for each method — TEMPORARY ONLY, must be filled in Phase 3
- Comments on every tree/map: "MUST use BTreeMap range query — NOT linear scan"

The skeleton locks the data structures. Phase 3 fills the bodies only.

## Phase 3: Implementation (FILL BODIES ONLY)

Fill in each `todo!()` body by reading the C# reference line by line.

<banned_patterns>
- `todo!()` — must be gone after this phase
- `unimplemented!()` — never acceptable
- `// TODO` / `// FIXME` — never acceptable in src/
- `#[ignore]` — never acceptable in tests/
- `add_nonrect_placeholder` — banned, use `b.add_polygon()` with real points
- Any comment containing "bbox approximat", "placeholder for non-rect", "SIMPLIFIED"
- Vec iteration where C# uses SortedSet/SortedDictionary operations
</banned_patterns>

If an algorithm requires non-rectangular obstacle support (ICurve, polygon routing):
- Port it faithfully using `b.add_polygon()` — do NOT substitute bounding boxes
- Tests that use polygon obstacles WILL FAIL until the VG generator polygon sweep is implemented
- Failing tests are CORRECT and HONEST — do not hide them

## Phase 4: Verification (DO NOT SKIP)

Run in order:

```bash
cargo check 2>&1 | grep error
cargo test -- --include-ignored 2>&1 | grep -E "FAILED|ignored"
bash tools/compliance-check.sh
```

The compliance check must pass before this task is complete.
If it finds violations, fix them — do not document them as "deferred."

## What "Failing Tests Are Correct" Means

If a test calls `b.add_polygon()` and the routing returns wrong results because the
VG generator uses bbox approximation: **that is the correct failure to expose**.
The test is honest. The implementation is broken. The test must NOT be `#[ignore]`-d.
The test must NOT use `add_nonrect_placeholder`.
The implementation must be fixed.

## Self-Check Before Any Write

Before writing any file, ask yourself:
1. Does this contain `todo!()`? → Fill it in now.
2. Does this contain `// TODO`? → Fix the issue now.
3. Does this contain `#[ignore]`? → Remove it and fix the implementation.
4. Does this substitute a bbox for a polygon? → Use the real polygon.
5. Does this pass `bash tools/compliance-check.sh`? → If not, fix it first.
