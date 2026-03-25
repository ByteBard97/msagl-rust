# Phase 1: Foundation (Geometry + Project Scaffolding) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the geometry foundation crate that all subsequent MSAGL phases depend on — Point, Rectangle, Polyline, Curve, and epsilon comparison utilities.

**Architecture:** Bottom-up construction of value types. Point is the atom; Rectangle and Polyline compose Points; Curve composes line segments and arcs. All float comparisons use a centralized epsilon module. Polyline uses SlotMap for its doubly-linked list to satisfy Rust's ownership model.

**Tech Stack:** Rust (2021 edition), ordered-float 5.x, slotmap 1.x, kurbo 0.13

**Spec:** `msagl-rust/docs/superpowers/specs/2026-03-25-msagl-rust-port-design.md`

**Porting sources:**
- TypeScript: `SignalCanvasFrontend/reference/msagl-js/modules/core/src/math/geometry/`
- C#: `MSAGL-Reference/GraphLayout/MSAGL/Core/Geometry/`

---

## File Structure

```
msagl-rust/
├── Cargo.toml
├── LICENSE-MIT
├── src/
│   ├── lib.rs                     # Crate root, public re-exports
│   ├── geometry/
│   │   ├── mod.rs                 # Module declarations
│   │   ├── point.rs               # Point struct (OrderedFloat-backed)
│   │   ├── point_comparer.rs      # Epsilon constants + comparison functions
│   │   ├── rectangle.rs           # Axis-aligned bounding box
│   │   ├── polyline.rs            # SlotMap-backed doubly-linked point list
│   │   └── curve.rs               # Compound curve (kurbo Line + Arc)
│   └── arenas.rs                  # SlotMap key types (PolylinePointKey)
└── tests/
    └── geometry/
        ├── mod.rs
        ├── point_tests.rs
        ├── point_comparer_tests.rs
        ├── rectangle_tests.rs
        ├── polyline_tests.rs
        └── curve_tests.rs
```

---

### Task 1: Project Scaffolding

**Files:**
- Create: `msagl-rust/Cargo.toml`
- Create: `msagl-rust/LICENSE-MIT`
- Create: `msagl-rust/src/lib.rs`

- [ ] **Step 1: Create Cargo.toml**

```toml
[package]
name = "msagl-rust"
version = "0.1.0"
edition = "2021"
description = "Rectilinear edge router for orthogonal graph layouts, ported from Microsoft's MSAGL"
license = "MIT"
repository = "https://github.com/anthropics/msagl-rust"

[dependencies]
ordered-float = "5"
slotmap = "1"
kurbo = "0.13"

[dev-dependencies]
approx = "0.5"
```

- [ ] **Step 2: Create LICENSE-MIT**

Copy the MIT license text. Include:
```
MIT License

Copyright (c) Microsoft Corporation.
Portions Copyright (c) 2026 SignalCanvas contributors.

Permission is hereby granted, free of charge, ...
```

(Use the standard MIT license body.)

- [ ] **Step 3: Create src/lib.rs with module declarations**

```rust
pub mod geometry;
mod arenas;

pub use geometry::point::Point;
pub use geometry::rectangle::Rectangle;
pub use geometry::polyline::{Polyline, PolylinePoint};
pub use geometry::curve::Curve;
pub use geometry::point_comparer::GeomConstants;
```

- [ ] **Step 4: Create empty module files**

Create these files with just their module declarations:

`src/geometry/mod.rs`:
```rust
pub mod point;
pub mod point_comparer;
pub mod rectangle;
pub mod polyline;
pub mod curve;
```

`src/arenas.rs`:
```rust
use slotmap::new_key_type;

new_key_type! {
    /// Key for a point in a Polyline's doubly-linked list.
    pub struct PolylinePointKey;
}
```

- [ ] **Step 5: Create empty test module**

`tests/geometry/mod.rs`:
```rust
mod point_tests;
mod point_comparer_tests;
mod rectangle_tests;
mod polyline_tests;
mod curve_tests;
```

Create each test file as empty:
```rust
// tests/geometry/point_tests.rs
use msagl_rust::Point;
```
(Repeat for each test file, importing the relevant type.)

- [ ] **Step 6: Verify project compiles**

Run: `cd msagl-rust && cargo check`
Expected: Compiles with warnings about unused imports (empty modules). No errors.

- [ ] **Step 7: Commit**

```bash
git add msagl-rust/Cargo.toml msagl-rust/LICENSE-MIT msagl-rust/src/ msagl-rust/tests/
git commit -m "feat(msagl-rust): scaffold project with module structure and dependencies"
```

---

### Task 2: GeomConstants (Epsilon Comparisons)

**Files:**
- Create: `msagl-rust/src/geometry/point_comparer.rs`
- Test: `msagl-rust/tests/geometry/point_comparer_tests.rs`

**Porting from:**
- TS: `geomConstants.ts` (11 lines)
- C#: `ApproximateComparer.cs`

- [ ] **Step 1: Write failing tests for epsilon constants and comparison functions**

```rust
// tests/geometry/point_comparer_tests.rs
use msagl_rust::GeomConstants;

#[test]
fn epsilon_values_match_reference() {
    assert_eq!(GeomConstants::DISTANCE_EPSILON, 1e-6);
    assert_eq!(GeomConstants::SQUARE_OF_DISTANCE_EPSILON, 1e-12);
    assert_eq!(GeomConstants::INTERSECTION_EPSILON, 0.0001);
    assert_eq!(GeomConstants::TOLERANCE, 1e-8);
}

#[test]
fn close_values_within_epsilon() {
    assert!(GeomConstants::close(1.0, 1.0 + 5e-7));
    assert!(GeomConstants::close(1.0, 1.0 - 5e-7));
}

#[test]
fn not_close_values_beyond_epsilon() {
    assert!(!GeomConstants::close(1.0, 1.0 + 2e-6));
    assert!(!GeomConstants::close(1.0, 1.0 - 2e-6));
}

#[test]
fn compare_less_equal_greater() {
    use std::cmp::Ordering;
    // Values within epsilon are Equal
    assert_eq!(GeomConstants::compare(1.0, 1.0 + 5e-7), Ordering::Equal);
    // Values beyond epsilon
    assert_eq!(GeomConstants::compare(1.0, 2.0), Ordering::Less);
    assert_eq!(GeomConstants::compare(2.0, 1.0), Ordering::Greater);
}

#[test]
fn sign_within_epsilon_is_zero() {
    assert_eq!(GeomConstants::sign(5e-7), 0);
    assert_eq!(GeomConstants::sign(-5e-7), 0);
    assert_eq!(GeomConstants::sign(2e-6), 1);
    assert_eq!(GeomConstants::sign(-2e-6), -1);
}

#[test]
fn round_to_six_decimals() {
    assert_eq!(GeomConstants::round(1.23456789), 1.234568);
    assert_eq!(GeomConstants::round(-0.0000001), 0.0);
    assert_eq!(GeomConstants::round(3.0), 3.0);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd msagl-rust && cargo test --test geometry -- point_comparer`
Expected: Compilation error — `GeomConstants` has no methods yet.

- [ ] **Step 3: Implement GeomConstants**

```rust
// src/geometry/point_comparer.rs
use std::cmp::Ordering;

/// Epsilon constants and approximate comparison functions matching MSAGL's
/// `ApproximateComparer` / `GeomConstants`.
pub struct GeomConstants;

const MULT: f64 = 1_000_000.0; // 10^6

impl GeomConstants {
    pub const DISTANCE_EPSILON_PRECISION: usize = 6;
    pub const DISTANCE_EPSILON: f64 = 1e-6;
    pub const SQUARE_OF_DISTANCE_EPSILON: f64 = 1e-12;
    pub const INTERSECTION_EPSILON: f64 = 0.0001;
    pub const TOLERANCE: f64 = 1e-8;

    /// Returns true if `|a - b| <= DISTANCE_EPSILON`.
    #[inline]
    pub fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= Self::DISTANCE_EPSILON
    }

    /// Three-way comparison with epsilon tolerance.
    #[inline]
    pub fn compare(a: f64, b: f64) -> Ordering {
        let diff = a - b;
        if diff < -Self::DISTANCE_EPSILON {
            Ordering::Less
        } else if diff > Self::DISTANCE_EPSILON {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }

    /// Returns -1, 0, or 1 based on epsilon tolerance.
    #[inline]
    pub fn sign(value: f64) -> i32 {
        if value > Self::DISTANCE_EPSILON {
            1
        } else if value < -Self::DISTANCE_EPSILON {
            -1
        } else {
            0
        }
    }

    /// Round to 6 decimal places, matching C#'s `ApproximateComparer.Round()`.
    #[inline]
    pub fn round(v: f64) -> f64 {
        (v * MULT).round() / MULT
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd msagl-rust && cargo test --test geometry -- point_comparer`
Expected: All 5 tests pass.

- [ ] **Step 5: Commit**

```bash
git add msagl-rust/src/geometry/point_comparer.rs msagl-rust/tests/geometry/point_comparer_tests.rs
git commit -m "feat(msagl-rust): add GeomConstants epsilon comparison utilities"
```

---

### Task 3: Point Type

**Files:**
- Create: `msagl-rust/src/geometry/point.rs`
- Test: `msagl-rust/tests/geometry/point_tests.rs`

**Porting from:**
- TS: `point.ts` (~270 lines)
- C#: `Point.cs`

- [ ] **Step 1: Write failing tests for Point construction and basic properties**

```rust
// tests/geometry/point_tests.rs
use msagl_rust::Point;
use approx::assert_abs_diff_eq;

#[test]
fn construction_rounds_to_six_decimals() {
    let p = Point::new(1.23456789, -0.0000001);
    assert_eq!(p.x(), 1.234568);
    assert_eq!(p.y(), 0.0);
}

#[test]
fn zero_point() {
    let p = Point::ORIGIN;
    assert_eq!(p.x(), 0.0);
    assert_eq!(p.y(), 0.0);
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd msagl-rust && cargo test --test geometry -- point_tests`
Expected: Compilation error — Point struct doesn't exist.

- [ ] **Step 3: Implement Point struct with construction**

```rust
// src/geometry/point.rs
use ordered_float::OrderedFloat;
use std::fmt;
use std::hash::{Hash, Hasher};

use crate::geometry::point_comparer::GeomConstants;

/// A 2D point with `OrderedFloat` coordinates for hashing and total ordering.
/// Coordinates are rounded to 6 decimal places on construction.
#[derive(Clone, Copy)]
pub struct Point {
    x: OrderedFloat<f64>,
    y: OrderedFloat<f64>,
}

impl Point {
    pub const ORIGIN: Point = Point {
        x: OrderedFloat(0.0),
        y: OrderedFloat(0.0),
    };

    /// Create a new point. Coordinates are rounded to 6 decimal places.
    #[inline]
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            x: OrderedFloat(GeomConstants::round(x)),
            y: OrderedFloat(GeomConstants::round(y)),
        }
    }

    #[inline]
    pub fn x(&self) -> f64 {
        self.x.into_inner()
    }

    #[inline]
    pub fn y(&self) -> f64 {
        self.y.into_inner()
    }
}

impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Eq for Point {}

impl Hash for Point {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.x.hash(state);
        self.y.hash(state);
    }
}

impl Ord for Point {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.x.cmp(&other.x).then(self.y.cmp(&other.y))
    }
}

impl PartialOrd for Point {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl fmt::Debug for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x(), self.y())
    }
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x(), self.y())
    }
}
```

- [ ] **Step 4: Run construction tests**

Run: `cd msagl-rust && cargo test --test geometry -- point_tests`
Expected: 2 tests pass.

- [ ] **Step 5: Write failing tests for arithmetic operations**

Add to `point_tests.rs`:
```rust
#[test]
fn add_points() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(3.0, 4.0);
    let c = a + b;
    assert_eq!(c.x(), 4.0);
    assert_eq!(c.y(), 6.0);
}

#[test]
fn sub_points() {
    let a = Point::new(5.0, 7.0);
    let b = Point::new(2.0, 3.0);
    let c = a - b;
    assert_eq!(c.x(), 3.0);
    assert_eq!(c.y(), 4.0);
}

#[test]
fn mul_scalar() {
    let p = Point::new(3.0, 4.0);
    let scaled = p * 2.0;
    assert_eq!(scaled.x(), 6.0);
    assert_eq!(scaled.y(), 8.0);
}

#[test]
fn div_scalar() {
    let p = Point::new(6.0, 8.0);
    let scaled = p / 2.0;
    assert_eq!(scaled.x(), 3.0);
    assert_eq!(scaled.y(), 4.0);
}

#[test]
fn negate() {
    let p = Point::new(3.0, -4.0);
    let n = -p;
    assert_eq!(n.x(), -3.0);
    assert_eq!(n.y(), 4.0);
}
```

- [ ] **Step 6: Run to verify failure**

Run: `cd msagl-rust && cargo test --test geometry -- point_tests`
Expected: Compilation error — no `Add`, `Sub`, etc. implementations.

- [ ] **Step 7: Implement arithmetic operators**

Add to `point.rs`:
```rust
use std::ops::{Add, Sub, Mul, Div, Neg};

impl Add for Point {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x() + rhs.x(), self.y() + rhs.y())
    }
}

impl Sub for Point {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x() - rhs.x(), self.y() - rhs.y())
    }
}

impl Mul<f64> for Point {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x() * rhs, self.y() * rhs)
    }
}

impl Mul<Point> for f64 {
    type Output = Point;
    #[inline]
    fn mul(self, rhs: Point) -> Point {
        Point::new(self * rhs.x(), self * rhs.y())
    }
}

impl Div<f64> for Point {
    type Output = Self;
    #[inline]
    fn div(self, rhs: f64) -> Self {
        Self::new(self.x() / rhs, self.y() / rhs)
    }
}

impl Neg for Point {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self::new(-self.x(), -self.y())
    }
}
```

- [ ] **Step 8: Run arithmetic tests**

Run: `cd msagl-rust && cargo test --test geometry -- point_tests`
Expected: All 7 tests pass.

- [ ] **Step 9: Write failing tests for geometric methods**

Add to `point_tests.rs`:
```rust
use msagl_rust::GeomConstants;

#[test]
fn length_3_4_5_triangle() {
    let p = Point::new(3.0, 4.0);
    assert_abs_diff_eq!(p.length(), 5.0, epsilon = 1e-10);
}

#[test]
fn length_squared() {
    let p = Point::new(3.0, 4.0);
    assert_abs_diff_eq!(p.length_squared(), 25.0, epsilon = 1e-10);
}

#[test]
fn l1_manhattan() {
    let p = Point::new(3.0, -4.0);
    assert_abs_diff_eq!(p.l1(), 7.0, epsilon = 1e-10);
}

#[test]
fn dot_product() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(3.0, 4.0);
    assert_abs_diff_eq!(a.dot(b), 11.0, epsilon = 1e-10);
}

#[test]
fn cross_product() {
    let a = Point::new(1.0, 0.0);
    let b = Point::new(0.0, 1.0);
    assert_abs_diff_eq!(Point::cross(a, b), 1.0, epsilon = 1e-10);
}

#[test]
fn normalize_unit_vector() {
    let p = Point::new(3.0, 4.0);
    let n = p.normalize();
    assert_abs_diff_eq!(n.length(), 1.0, epsilon = 1e-6);
    assert_abs_diff_eq!(n.x(), 0.6, epsilon = 1e-6);
    assert_abs_diff_eq!(n.y(), 0.8, epsilon = 1e-6);
}

#[test]
fn rotate_90_ccw() {
    let p = Point::new(1.0, 0.0);
    let r = p.rotate90_ccw();
    assert_abs_diff_eq!(r.x(), 0.0, epsilon = 1e-6);
    assert_abs_diff_eq!(r.y(), 1.0, epsilon = 1e-6);
}

#[test]
fn rotate_90_cw() {
    let p = Point::new(0.0, 1.0);
    let r = p.rotate90_cw();
    assert_abs_diff_eq!(r.x(), 1.0, epsilon = 1e-6);
    assert_abs_diff_eq!(r.y(), 0.0, epsilon = 1e-6);
}

#[test]
fn close_dist_eps_within() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(1.0 + 5e-7, 2.0 - 3e-7);
    assert!(a.close_to(b));
}

#[test]
fn close_dist_eps_beyond() {
    let a = Point::new(1.0, 2.0);
    let b = Point::new(1.0 + 1e-3, 2.0);
    assert!(!a.close_to(b));
}

#[test]
fn middle_of_two_points() {
    let a = Point::new(0.0, 0.0);
    let b = Point::new(4.0, 6.0);
    let m = Point::middle(a, b);
    assert_eq!(m.x(), 2.0);
    assert_eq!(m.y(), 3.0);
}

#[test]
fn line_line_intersection() {
    // Horizontal line y=1 from (0,1) to (4,1)
    // Vertical line x=2 from (2,0) to (2,4)
    let result = Point::line_line_intersection(
        Point::new(0.0, 1.0), Point::new(4.0, 1.0),
        Point::new(2.0, 0.0), Point::new(2.0, 4.0),
    );
    assert!(result.is_some());
    let p = result.unwrap();
    assert_abs_diff_eq!(p.x(), 2.0, epsilon = 1e-6);
    assert_abs_diff_eq!(p.y(), 1.0, epsilon = 1e-6);
}

#[test]
fn parallel_lines_no_intersection() {
    let result = Point::line_line_intersection(
        Point::new(0.0, 0.0), Point::new(1.0, 0.0),
        Point::new(0.0, 1.0), Point::new(1.0, 1.0),
    );
    assert!(result.is_none());
}

#[test]
fn hashing_identical_points() {
    use std::collections::HashSet;
    let mut set = HashSet::new();
    set.insert(Point::new(1.0, 2.0));
    set.insert(Point::new(1.0, 2.0));
    assert_eq!(set.len(), 1);
}

#[test]
fn ordering_lexicographic() {
    let a = Point::new(1.0, 5.0);
    let b = Point::new(2.0, 1.0);
    let c = Point::new(1.0, 3.0);
    assert!(a < b);  // x=1 < x=2
    assert!(c < a);  // same x, y=3 < y=5
}
```

- [ ] **Step 10: Run to verify failure**

Run: `cd msagl-rust && cargo test --test geometry -- point_tests`
Expected: Compilation errors — methods not defined.

- [ ] **Step 11: Implement geometric methods**

Add to `point.rs`:
```rust
impl Point {
    // ... (existing new, x, y, ORIGIN) ...

    /// Euclidean length (L2 norm).
    #[inline]
    pub fn length(&self) -> f64 {
        (self.x() * self.x() + self.y() * self.y()).sqrt()
    }

    /// Squared Euclidean length.
    #[inline]
    pub fn length_squared(&self) -> f64 {
        self.x() * self.x() + self.y() * self.y()
    }

    /// Manhattan distance (L1 norm).
    #[inline]
    pub fn l1(&self) -> f64 {
        self.x().abs() + self.y().abs()
    }

    /// Dot product.
    #[inline]
    pub fn dot(&self, other: Point) -> f64 {
        self.x() * other.x() + self.y() * other.y()
    }

    /// 2D cross product (determinant).
    #[inline]
    pub fn cross(a: Point, b: Point) -> f64 {
        a.x() * b.y() - a.y() * b.x()
    }

    /// Unit vector in the same direction.
    pub fn normalize(&self) -> Point {
        let len = self.length();
        debug_assert!(len > GeomConstants::DISTANCE_EPSILON, "cannot normalize zero-length vector");
        Point::new(self.x() / len, self.y() / len)
    }

    /// Rotate 90 degrees counter-clockwise.
    #[inline]
    pub fn rotate90_ccw(&self) -> Point {
        Point::new(-self.y(), self.x())
    }

    /// Rotate 90 degrees clockwise.
    #[inline]
    pub fn rotate90_cw(&self) -> Point {
        Point::new(self.y(), -self.x())
    }

    /// Rotate by an angle in radians (counter-clockwise).
    pub fn rotate(&self, angle: f64) -> Point {
        let c = angle.cos();
        let s = angle.sin();
        Point::new(c * self.x() - s * self.y(), s * self.x() + c * self.y())
    }

    /// True if distance to `other` is within `DISTANCE_EPSILON`.
    #[inline]
    pub fn close_to(&self, other: Point) -> bool {
        (*self - other).length() <= GeomConstants::DISTANCE_EPSILON
    }

    /// True if distance to `other` is within given tolerance.
    #[inline]
    pub fn close_to_with_eps(&self, other: Point, eps: f64) -> bool {
        (*self - other).length() <= eps
    }

    /// Midpoint of two points.
    #[inline]
    pub fn middle(a: Point, b: Point) -> Point {
        (a + b) / 2.0
    }

    /// Line-line intersection. Returns None if lines are parallel.
    /// Lines defined by points (a, b) and (c, d).
    pub fn line_line_intersection(a: Point, b: Point, c: Point, d: Point) -> Option<Point> {
        let ba = b - a;
        let dc = d - c;
        let ca = c - a;
        let denom = ba.x() * dc.y() - ba.y() * dc.x();
        if denom.abs() < GeomConstants::TOLERANCE {
            return None;
        }
        let t = (ca.x() * dc.y() - ca.y() * dc.x()) / denom;
        Some(a + ba * t)
    }

    /// Signed doubled triangle area. Positive if (a, b, c) is counter-clockwise.
    #[inline]
    pub fn signed_doubled_triangle_area(a: Point, b: Point, c: Point) -> f64 {
        Point::cross(b - a, c - a)
    }
}
```

- [ ] **Step 12: Run all point tests**

Run: `cd msagl-rust && cargo test --test geometry -- point_tests`
Expected: All 18 tests pass.

- [ ] **Step 13: Commit**

```bash
git add msagl-rust/src/geometry/point.rs msagl-rust/tests/geometry/point_tests.rs
git commit -m "feat(msagl-rust): add Point type with arithmetic, geometry, and hashing"
```

---

### Task 4: Rectangle

**Files:**
- Create: `msagl-rust/src/geometry/rectangle.rs`
- Test: `msagl-rust/tests/geometry/rectangle_tests.rs`

**Porting from:**
- TS: `rectangle.ts` (~350 lines)
- C#: `Rectangle.cs`

- [ ] **Step 1: Write failing tests**

```rust
// tests/geometry/rectangle_tests.rs
use msagl_rust::{Point, Rectangle};
use approx::assert_abs_diff_eq;

#[test]
fn new_from_left_bottom_right_top() {
    let r = Rectangle::new(1.0, 2.0, 5.0, 6.0);
    assert_eq!(r.left(), 1.0);
    assert_eq!(r.bottom(), 2.0);
    assert_eq!(r.right(), 5.0);
    assert_eq!(r.top(), 6.0);
}

#[test]
fn width_and_height() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 5.0);
    assert_abs_diff_eq!(r.width(), 10.0, epsilon = 1e-10);
    assert_abs_diff_eq!(r.height(), 5.0, epsilon = 1e-10);
}

#[test]
fn center() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 6.0);
    let c = r.center();
    assert_abs_diff_eq!(c.x(), 5.0, epsilon = 1e-10);
    assert_abs_diff_eq!(c.y(), 3.0, epsilon = 1e-10);
}

#[test]
fn corner_points() {
    let r = Rectangle::new(1.0, 2.0, 5.0, 6.0);
    assert_eq!(r.left_bottom(), Point::new(1.0, 2.0));
    assert_eq!(r.right_top(), Point::new(5.0, 6.0));
    assert_eq!(r.left_top(), Point::new(1.0, 6.0));
    assert_eq!(r.right_bottom(), Point::new(5.0, 2.0));
}

#[test]
fn from_two_points() {
    let r = Rectangle::from_points(Point::new(5.0, 6.0), Point::new(1.0, 2.0));
    assert_eq!(r.left(), 1.0);
    assert_eq!(r.bottom(), 2.0);
    assert_eq!(r.right(), 5.0);
    assert_eq!(r.top(), 6.0);
}

#[test]
fn contains_point_inside() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(r.contains(Point::new(5.0, 5.0)));
}

#[test]
fn contains_point_on_boundary() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(r.contains(Point::new(0.0, 5.0)));
    assert!(r.contains(Point::new(10.0, 5.0)));
}

#[test]
fn does_not_contain_point_outside() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    assert!(!r.contains(Point::new(11.0, 5.0)));
    assert!(!r.contains(Point::new(-1.0, 5.0)));
}

#[test]
fn contains_with_padding() {
    let r = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    // Point at 10.5 is outside, but within padding=1.0
    assert!(r.contains_with_padding(Point::new(10.5, 5.0), 1.0));
    assert!(!r.contains_with_padding(Point::new(12.0, 5.0), 1.0));
}

#[test]
fn intersects_overlapping() {
    let a = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    let b = Rectangle::new(5.0, 5.0, 15.0, 15.0);
    assert!(a.intersects(&b));
    assert!(b.intersects(&a));
}

#[test]
fn intersects_disjoint() {
    let a = Rectangle::new(0.0, 0.0, 5.0, 5.0);
    let b = Rectangle::new(10.0, 10.0, 15.0, 15.0);
    assert!(!a.intersects(&b));
}

#[test]
fn contains_rect() {
    let outer = Rectangle::new(0.0, 0.0, 20.0, 20.0);
    let inner = Rectangle::new(5.0, 5.0, 10.0, 10.0);
    assert!(outer.contains_rect(&inner));
    assert!(!inner.contains_rect(&outer));
}

#[test]
fn intersection_region() {
    let a = Rectangle::new(0.0, 0.0, 10.0, 10.0);
    let b = Rectangle::new(5.0, 3.0, 15.0, 8.0);
    let i = a.intersection(&b);
    assert_eq!(i.left(), 5.0);
    assert_eq!(i.bottom(), 3.0);
    assert_eq!(i.right(), 10.0);
    assert_eq!(i.top(), 8.0);
}

#[test]
fn add_point_expands_bbox() {
    let mut r = Rectangle::from_point(Point::new(0.0, 0.0));
    r.add_point(Point::new(5.0, 3.0));
    assert_eq!(r.left(), 0.0);
    assert_eq!(r.right(), 5.0);
    assert_eq!(r.bottom(), 0.0);
    assert_eq!(r.top(), 3.0);
}

#[test]
fn add_rect_expands_bbox() {
    let mut a = Rectangle::new(0.0, 0.0, 5.0, 5.0);
    let b = Rectangle::new(3.0, 3.0, 10.0, 10.0);
    a.add_rect(&b);
    assert_eq!(a.left(), 0.0);
    assert_eq!(a.right(), 10.0);
    assert_eq!(a.top(), 10.0);
}

#[test]
fn pad() {
    let mut r = Rectangle::new(2.0, 3.0, 8.0, 7.0);
    r.pad(1.0);
    assert_eq!(r.left(), 1.0);
    assert_eq!(r.bottom(), 2.0);
    assert_eq!(r.right(), 9.0);
    assert_eq!(r.top(), 8.0);
}

#[test]
fn empty_rectangle() {
    let r = Rectangle::empty();
    assert!(r.is_empty());
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd msagl-rust && cargo test --test geometry -- rectangle_tests`
Expected: Compilation error — `Rectangle` not defined.

- [ ] **Step 3: Implement Rectangle**

```rust
// src/geometry/rectangle.rs
use crate::geometry::point::Point;
use crate::geometry::point_comparer::GeomConstants;

/// Axis-aligned bounding box with left/bottom/right/top bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rectangle {
    left: f64,
    bottom: f64,
    right: f64,
    top: f64,
}

impl Rectangle {
    /// Create from explicit bounds: left, bottom, right, top.
    pub fn new(left: f64, bottom: f64, right: f64, top: f64) -> Self {
        Self { left, bottom, right, top }
    }

    /// Create from two corner points (any diagonal).
    pub fn from_points(a: Point, b: Point) -> Self {
        Self {
            left: a.x().min(b.x()),
            bottom: a.y().min(b.y()),
            right: a.x().max(b.x()),
            top: a.y().max(b.y()),
        }
    }

    /// Degenerate rectangle at a single point.
    pub fn from_point(p: Point) -> Self {
        Self {
            left: p.x(),
            bottom: p.y(),
            right: p.x(),
            top: p.y(),
        }
    }

    /// Empty (invalid) rectangle. `is_empty()` returns true.
    pub fn empty() -> Self {
        Self { left: 0.0, right: -1.0, bottom: 0.0, top: -1.0 }
    }

    pub fn is_empty(&self) -> bool {
        self.right < self.left
    }

    pub fn left(&self) -> f64 { self.left }
    pub fn bottom(&self) -> f64 { self.bottom }
    pub fn right(&self) -> f64 { self.right }
    pub fn top(&self) -> f64 { self.top }
    pub fn width(&self) -> f64 { self.right - self.left }
    pub fn height(&self) -> f64 { self.top - self.bottom }

    pub fn center(&self) -> Point {
        Point::new((self.left + self.right) / 2.0, (self.bottom + self.top) / 2.0)
    }

    pub fn left_bottom(&self) -> Point { Point::new(self.left, self.bottom) }
    pub fn right_top(&self) -> Point { Point::new(self.right, self.top) }
    pub fn left_top(&self) -> Point { Point::new(self.left, self.top) }
    pub fn right_bottom(&self) -> Point { Point::new(self.right, self.bottom) }

    /// Point is inside or on the boundary (with epsilon tolerance).
    pub fn contains(&self, p: Point) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        p.x() >= self.left - eps
            && p.x() <= self.right + eps
            && p.y() >= self.bottom - eps
            && p.y() <= self.top + eps
    }

    /// Point is inside with extra padding.
    pub fn contains_with_padding(&self, p: Point, padding: f64) -> bool {
        p.x() >= self.left - padding
            && p.x() <= self.right + padding
            && p.y() >= self.bottom - padding
            && p.y() <= self.top + padding
    }

    /// Another rectangle is entirely contained within this one.
    pub fn contains_rect(&self, other: &Rectangle) -> bool {
        self.contains(other.left_bottom()) && self.contains(other.right_top())
    }

    /// Two rectangles overlap (with epsilon tolerance).
    pub fn intersects(&self, other: &Rectangle) -> bool {
        self.intersects_on_x(other) && self.intersects_on_y(other)
    }

    fn intersects_on_x(&self, other: &Rectangle) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        !(other.left > self.right + eps || other.right < self.left - eps)
    }

    fn intersects_on_y(&self, other: &Rectangle) -> bool {
        let eps = GeomConstants::DISTANCE_EPSILON;
        !(other.bottom > self.top + eps || other.top < self.bottom - eps)
    }

    /// Intersection region. Returns empty rectangle if disjoint.
    pub fn intersection(&self, other: &Rectangle) -> Rectangle {
        if !self.intersects(other) {
            return Rectangle::empty();
        }
        Rectangle::new(
            self.left.max(other.left),
            self.bottom.max(other.bottom),
            self.right.min(other.right),
            self.top.min(other.top),
        )
    }

    /// Expand to include a point.
    pub fn add_point(&mut self, p: Point) {
        if self.is_empty() {
            *self = Rectangle::from_point(p);
        } else {
            self.left = self.left.min(p.x());
            self.right = self.right.max(p.x());
            self.bottom = self.bottom.min(p.y());
            self.top = self.top.max(p.y());
        }
    }

    /// Expand to include another rectangle.
    pub fn add_rect(&mut self, other: &Rectangle) {
        if other.is_empty() {
            return;
        }
        if self.is_empty() {
            *self = *other;
        } else {
            self.left = self.left.min(other.left);
            self.right = self.right.max(other.right);
            self.bottom = self.bottom.min(other.bottom);
            self.top = self.top.max(other.top);
        }
    }

    /// Expand all sides by `padding`.
    pub fn pad(&mut self, padding: f64) {
        self.left -= padding;
        self.bottom -= padding;
        self.right += padding;
        self.top += padding;
    }
}
```

- [ ] **Step 4: Run rectangle tests**

Run: `cd msagl-rust && cargo test --test geometry -- rectangle_tests`
Expected: All 17 tests pass.

- [ ] **Step 5: Commit**

```bash
git add msagl-rust/src/geometry/rectangle.rs msagl-rust/tests/geometry/rectangle_tests.rs
git commit -m "feat(msagl-rust): add Rectangle type with containment, intersection, and expansion"
```

---

### Task 5: Polyline (SlotMap-backed doubly-linked list)

**Files:**
- Create: `msagl-rust/src/geometry/polyline.rs`
- Modify: `msagl-rust/src/arenas.rs` (already has PolylinePointKey)
- Test: `msagl-rust/tests/geometry/polyline_tests.rs`

**Porting from:**
- TS: `polyline.ts` (~500 lines) + `polylinePoint.ts` (54 lines)
- C#: `Polyline.cs` + `PolylinePoint.cs`

**Key design decision:** The TS/C# Polyline uses GC'd object references for next/prev pointers. In Rust, we use a `SlotMap<PolylinePointKey, PolylinePointData>` owned by the Polyline. External code accesses points via keys, not references.

- [ ] **Step 1: Write failing tests for basic polyline operations**

```rust
// tests/geometry/polyline_tests.rs
use msagl_rust::{Point, Polyline};
use approx::assert_abs_diff_eq;

#[test]
fn empty_polyline() {
    let poly = Polyline::new();
    assert_eq!(poly.count(), 0);
    assert!(poly.start_key().is_none());
    assert!(poly.end_key().is_none());
}

#[test]
fn add_single_point() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(1.0, 2.0));
    assert_eq!(poly.count(), 1);
    assert!(poly.start_key().is_some());
    assert_eq!(poly.start_key(), poly.end_key());
}

#[test]
fn add_multiple_points() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(1.0, 0.0));
    poly.add_point(Point::new(1.0, 1.0));
    assert_eq!(poly.count(), 3);
}

#[test]
fn start_and_end_points() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(5.0, 5.0));
    assert_eq!(poly.start(), Point::new(0.0, 0.0));
    assert_eq!(poly.end(), Point::new(5.0, 5.0));
}

#[test]
fn iterate_points() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 0.0));
    poly.add_point(Point::new(1.0, 0.0));
    poly.add_point(Point::new(1.0, 1.0));

    let points: Vec<Point> = poly.points().collect();
    assert_eq!(points.len(), 3);
    assert_eq!(points[0], Point::new(0.0, 0.0));
    assert_eq!(points[1], Point::new(1.0, 0.0));
    assert_eq!(points[2], Point::new(1.0, 1.0));
}

#[test]
fn iterate_polyline_points_with_keys() {
    let mut poly = Polyline::new();
    let k0 = poly.add_point(Point::new(0.0, 0.0));
    let k1 = poly.add_point(Point::new(1.0, 0.0));
    let k2 = poly.add_point(Point::new(1.0, 1.0));

    // Navigate forward
    assert_eq!(poly.next_key(k0), Some(k1));
    assert_eq!(poly.next_key(k1), Some(k2));
    assert_eq!(poly.next_key(k2), None);

    // Navigate backward
    assert_eq!(poly.prev_key(k2), Some(k1));
    assert_eq!(poly.prev_key(k1), Some(k0));
    assert_eq!(poly.prev_key(k0), None);
}

#[test]
fn point_at_key() {
    let mut poly = Polyline::new();
    let k = poly.add_point(Point::new(3.0, 4.0));
    assert_eq!(poly.point_at(k), Point::new(3.0, 4.0));
}

#[test]
fn bounding_box() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(0.0, 2.0));
    poly.add_point(Point::new(5.0, 0.0));
    poly.add_point(Point::new(3.0, 7.0));

    let bb = poly.bounding_box();
    assert_eq!(bb.left(), 0.0);
    assert_eq!(bb.bottom(), 0.0);
    assert_eq!(bb.right(), 5.0);
    assert_eq!(bb.top(), 7.0);
}

#[test]
fn from_points_constructor() {
    let poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(1.0, 1.0),
    ]);
    assert_eq!(poly.count(), 3);
    assert_eq!(poly.start(), Point::new(0.0, 0.0));
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd msagl-rust && cargo test --test geometry -- polyline_tests`
Expected: Compilation error — Polyline not defined.

- [ ] **Step 3: Implement Polyline with SlotMap storage**

```rust
// src/geometry/polyline.rs
use slotmap::SlotMap;

use crate::arenas::PolylinePointKey;
use crate::geometry::point::Point;
use crate::geometry::rectangle::Rectangle;

/// A single node in the polyline's doubly-linked list.
#[derive(Clone, Debug)]
struct PolylinePointData {
    point: Point,
    next: Option<PolylinePointKey>,
    prev: Option<PolylinePointKey>,
}

/// Public view of a polyline point (key + point value).
#[derive(Clone, Copy, Debug)]
pub struct PolylinePoint {
    pub key: PolylinePointKey,
    pub point: Point,
}

/// A doubly-linked list of points backed by a SlotMap.
/// Supports both open and closed polylines.
#[derive(Clone, Debug)]
pub struct Polyline {
    nodes: SlotMap<PolylinePointKey, PolylinePointData>,
    start: Option<PolylinePointKey>,
    end: Option<PolylinePointKey>,
    closed: bool,
}

impl Polyline {
    pub fn new() -> Self {
        Self {
            nodes: SlotMap::with_key(),
            start: None,
            end: None,
            closed: false,
        }
    }

    /// Create a polyline from a slice of points.
    pub fn from_points(pts: &[Point]) -> Self {
        let mut poly = Self::new();
        for &p in pts {
            poly.add_point(p);
        }
        poly
    }

    /// Append a point to the end. Returns the key for the new point.
    pub fn add_point(&mut self, p: Point) -> PolylinePointKey {
        let key = self.nodes.insert(PolylinePointData {
            point: p,
            next: None,
            prev: self.end,
        });
        if let Some(end_key) = self.end {
            self.nodes[end_key].next = Some(key);
        } else {
            self.start = Some(key);
        }
        self.end = Some(key);
        key
    }

    /// Prepend a point to the start. Returns the key for the new point.
    pub fn prepend_point(&mut self, p: Point) -> PolylinePointKey {
        let key = self.nodes.insert(PolylinePointData {
            point: p,
            next: self.start,
            prev: None,
        });
        if let Some(start_key) = self.start {
            self.nodes[start_key].prev = Some(key);
        } else {
            self.end = Some(key);
        }
        self.start = Some(key);
        key
    }

    pub fn count(&self) -> usize {
        self.nodes.len()
    }

    pub fn is_closed(&self) -> bool {
        self.closed
    }

    pub fn set_closed(&mut self, closed: bool) {
        self.closed = closed;
    }

    pub fn start_key(&self) -> Option<PolylinePointKey> {
        self.start
    }

    pub fn end_key(&self) -> Option<PolylinePointKey> {
        self.end
    }

    /// The first point's coordinates. Panics if empty.
    pub fn start(&self) -> Point {
        self.nodes[self.start.expect("empty polyline")].point
    }

    /// The last point's coordinates. Panics if empty.
    pub fn end(&self) -> Point {
        self.nodes[self.end.expect("empty polyline")].point
    }

    /// Get the point at a given key.
    pub fn point_at(&self, key: PolylinePointKey) -> Point {
        self.nodes[key].point
    }

    /// Set the point at a given key.
    pub fn set_point_at(&mut self, key: PolylinePointKey, p: Point) {
        self.nodes[key].point = p;
    }

    /// Next key in the chain. For closed polylines, wraps from end to start.
    pub fn next_key(&self, key: PolylinePointKey) -> Option<PolylinePointKey> {
        let next = self.nodes[key].next;
        if next.is_some() {
            next
        } else if self.closed {
            self.start
        } else {
            None
        }
    }

    /// Previous key in the chain. For closed polylines, wraps from start to end.
    pub fn prev_key(&self, key: PolylinePointKey) -> Option<PolylinePointKey> {
        let prev = self.nodes[key].prev;
        if prev.is_some() {
            prev
        } else if self.closed {
            self.end
        } else {
            None
        }
    }

    /// Iterate over all points in order.
    pub fn points(&self) -> PointIter<'_> {
        PointIter {
            poly: self,
            current: self.start,
            done: false,
        }
    }

    /// Iterate over (key, point) pairs in order.
    pub fn polyline_points(&self) -> PolylinePointIter<'_> {
        PolylinePointIter {
            poly: self,
            current: self.start,
            done: false,
        }
    }

    /// Compute the axis-aligned bounding box.
    pub fn bounding_box(&self) -> Rectangle {
        let mut bb = Rectangle::empty();
        for p in self.points() {
            bb.add_point(p);
        }
        bb
    }

    /// Total length of the polyline segments.
    pub fn length(&self) -> f64 {
        let mut total = 0.0;
        let mut current = self.start;
        while let Some(key) = current {
            let next = self.nodes[key].next;
            if let Some(next_key) = next {
                let a = self.nodes[key].point;
                let b = self.nodes[next_key].point;
                total += (b - a).length();
            }
            current = next;
        }
        if self.closed {
            if let (Some(s), Some(e)) = (self.start, self.end) {
                total += (self.nodes[s].point - self.nodes[e].point).length();
            }
        }
        total
    }

    /// Translate all points by a delta vector.
    pub fn translate(&mut self, delta: Point) {
        let keys: Vec<_> = self.nodes.keys().collect();
        for key in keys {
            let p = self.nodes[key].point;
            self.nodes[key].point = p + delta;
        }
    }

    /// Remove the start point. Panics if empty.
    pub fn remove_start_point(&mut self) {
        let start_key = self.start.expect("empty polyline");
        let next = self.nodes[start_key].next;
        self.nodes.remove(start_key);
        self.start = next;
        if let Some(next_key) = next {
            self.nodes[next_key].prev = None;
        } else {
            self.end = None;
        }
    }

    /// Remove the end point. Panics if empty.
    pub fn remove_end_point(&mut self) {
        let end_key = self.end.expect("empty polyline");
        let prev = self.nodes[end_key].prev;
        self.nodes.remove(end_key);
        self.end = prev;
        if let Some(prev_key) = prev {
            self.nodes[prev_key].next = None;
        } else {
            self.start = None;
        }
    }
}

impl Default for Polyline {
    fn default() -> Self {
        Self::new()
    }
}

/// Iterator over Point values in a Polyline.
pub struct PointIter<'a> {
    poly: &'a Polyline,
    current: Option<PolylinePointKey>,
    done: bool,
}

impl<'a> Iterator for PointIter<'a> {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        if self.done {
            return None;
        }
        let key = self.current?;
        let point = self.poly.nodes[key].point;
        let next = self.poly.nodes[key].next;
        if next.is_none() && self.poly.closed {
            // For closed polylines, we stop after visiting all nodes once
            self.done = true;
        }
        self.current = next;
        Some(point)
    }
}

/// Iterator over PolylinePoint (key + point) pairs.
pub struct PolylinePointIter<'a> {
    poly: &'a Polyline,
    current: Option<PolylinePointKey>,
    done: bool,
}

impl<'a> Iterator for PolylinePointIter<'a> {
    type Item = PolylinePoint;

    fn next(&mut self) -> Option<PolylinePoint> {
        if self.done {
            return None;
        }
        let key = self.current?;
        let data = &self.poly.nodes[key];
        let pp = PolylinePoint {
            key,
            point: data.point,
        };
        let next = data.next;
        if next.is_none() && self.poly.closed {
            self.done = true;
        }
        self.current = next;
        Some(pp)
    }
}
```

- [ ] **Step 4: Run polyline tests**

Run: `cd msagl-rust && cargo test --test geometry -- polyline_tests`
Expected: All 9 tests pass.

- [ ] **Step 5: Write failing tests for advanced polyline operations**

Add to `polyline_tests.rs`:
```rust
#[test]
fn prepend_point() {
    let mut poly = Polyline::new();
    poly.add_point(Point::new(1.0, 0.0));
    poly.prepend_point(Point::new(0.0, 0.0));
    assert_eq!(poly.start(), Point::new(0.0, 0.0));
    assert_eq!(poly.end(), Point::new(1.0, 0.0));
    assert_eq!(poly.count(), 2);
}

#[test]
fn set_point_at() {
    let mut poly = Polyline::new();
    let k = poly.add_point(Point::new(0.0, 0.0));
    poly.set_point_at(k, Point::new(5.0, 5.0));
    assert_eq!(poly.point_at(k), Point::new(5.0, 5.0));
}

#[test]
fn polyline_length() {
    let poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(3.0, 0.0),
        Point::new(3.0, 4.0),
    ]);
    assert_abs_diff_eq!(poly.length(), 7.0, epsilon = 1e-10);
}

#[test]
fn translate_all_points() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 1.0),
    ]);
    poly.translate(Point::new(10.0, 20.0));
    assert_eq!(poly.start(), Point::new(10.0, 20.0));
    assert_eq!(poly.end(), Point::new(11.0, 21.0));
}

#[test]
fn closed_polyline_wraps() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(0.5, 1.0),
    ]);
    poly.set_closed(true);

    let start = poly.start_key().unwrap();
    let end = poly.end_key().unwrap();
    // Next from end wraps to start
    assert_eq!(poly.next_key(end), Some(start));
    // Prev from start wraps to end
    assert_eq!(poly.prev_key(start), Some(end));
}

#[test]
fn closed_polyline_length_includes_closing_edge() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(1.0, 1.0),
        Point::new(0.0, 1.0),
    ]);
    poly.set_closed(true);
    // 4 edges of unit square = 4.0
    assert_abs_diff_eq!(poly.length(), 4.0, epsilon = 1e-10);
}

#[test]
fn remove_start_point() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(2.0, 0.0),
    ]);
    poly.remove_start_point();
    assert_eq!(poly.count(), 2);
    assert_eq!(poly.start(), Point::new(1.0, 0.0));
}

#[test]
fn remove_end_point() {
    let mut poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(1.0, 0.0),
        Point::new(2.0, 0.0),
    ]);
    poly.remove_end_point();
    assert_eq!(poly.count(), 2);
    assert_eq!(poly.end(), Point::new(1.0, 0.0));
}

#[test]
fn iterate_polyline_points() {
    let mut poly = Polyline::new();
    let k0 = poly.add_point(Point::new(0.0, 0.0));
    let k1 = poly.add_point(Point::new(1.0, 0.0));

    let pps: Vec<_> = poly.polyline_points().collect();
    assert_eq!(pps.len(), 2);
    assert_eq!(pps[0].key, k0);
    assert_eq!(pps[0].point, Point::new(0.0, 0.0));
    assert_eq!(pps[1].key, k1);
}
```

- [ ] **Step 6: Run to verify they pass** (implementation already covers these)

Run: `cd msagl-rust && cargo test --test geometry -- polyline_tests`
Expected: All 18 tests pass.

- [ ] **Step 7: Commit**

```bash
git add msagl-rust/src/geometry/polyline.rs msagl-rust/src/arenas.rs msagl-rust/tests/geometry/polyline_tests.rs
git commit -m "feat(msagl-rust): add Polyline with SlotMap-backed doubly-linked list"
```

---

### Task 6: Curve (Compound Curve)

**Files:**
- Create: `msagl-rust/src/geometry/curve.rs`
- Test: `msagl-rust/tests/geometry/curve_tests.rs`

**Porting from:**
- TS: `curve.ts`, `lineSegment.ts`, `icurve.ts`
- C#: `Curve.cs`, `LineSegment.cs`

**Note:** The rectilinear router outputs routes as Polylines and only converts to Curve for final output. The Curve type needs to hold a sequence of segments (line segments and arcs) and provide bounding box, start/end, and iteration. We use `kurbo` types internally.

- [ ] **Step 1: Write failing tests**

```rust
// tests/geometry/curve_tests.rs
use msagl_rust::{Point, Curve};
use approx::assert_abs_diff_eq;

#[test]
fn empty_curve() {
    let c = Curve::new();
    assert_eq!(c.segment_count(), 0);
}

#[test]
fn single_line_segment() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    assert_eq!(c.segment_count(), 1);
    assert_eq!(c.start(), Point::new(0.0, 0.0));
    assert_eq!(c.end(), Point::new(5.0, 0.0));
}

#[test]
fn multiple_line_segments() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    c.add_line(Point::new(5.0, 0.0), Point::new(5.0, 3.0));
    assert_eq!(c.segment_count(), 2);
    assert_eq!(c.start(), Point::new(0.0, 0.0));
    assert_eq!(c.end(), Point::new(5.0, 3.0));
}

#[test]
fn bounding_box_of_line_segments() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    c.add_line(Point::new(5.0, 0.0), Point::new(5.0, 3.0));

    let bb = c.bounding_box();
    assert_eq!(bb.left(), 0.0);
    assert_eq!(bb.bottom(), 0.0);
    assert_eq!(bb.right(), 5.0);
    assert_eq!(bb.top(), 3.0);
}

#[test]
fn from_polyline() {
    use msagl_rust::Polyline;
    let poly = Polyline::from_points(&[
        Point::new(0.0, 0.0),
        Point::new(3.0, 0.0),
        Point::new(3.0, 4.0),
    ]);
    let c = Curve::from_polyline(&poly);
    assert_eq!(c.segment_count(), 2);
    assert_eq!(c.start(), Point::new(0.0, 0.0));
    assert_eq!(c.end(), Point::new(3.0, 4.0));
}

#[test]
fn add_arc_segment() {
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    // Add a 90-degree arc from (5,0) turning up with radius 2
    c.add_arc(Point::new(5.0, 0.0), Point::new(7.0, 2.0), Point::new(7.0, 0.0), true);
    assert_eq!(c.segment_count(), 2);
}

#[test]
fn iterate_segments() {
    use msagl_rust::CurveSegment;
    let mut c = Curve::new();
    c.add_line(Point::new(0.0, 0.0), Point::new(5.0, 0.0));
    c.add_line(Point::new(5.0, 0.0), Point::new(5.0, 3.0));

    let segments: Vec<_> = c.segments().collect();
    assert_eq!(segments.len(), 2);
    match &segments[0] {
        CurveSegment::Line { from, to } => {
            assert_eq!(*from, Point::new(0.0, 0.0));
            assert_eq!(*to, Point::new(5.0, 0.0));
        }
        _ => panic!("expected line segment"),
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd msagl-rust && cargo test --test geometry -- curve_tests`
Expected: Compilation error — Curve not defined.

- [ ] **Step 3: Implement Curve**

```rust
// src/geometry/curve.rs
use crate::geometry::point::Point;
use crate::geometry::polyline::Polyline;
use crate::geometry::rectangle::Rectangle;

/// A segment within a compound curve.
#[derive(Clone, Debug)]
pub enum CurveSegment {
    Line {
        from: Point,
        to: Point,
    },
    Arc {
        from: Point,
        to: Point,
        /// Center of the arc's circle.
        center: Point,
        /// True = counter-clockwise.
        ccw: bool,
    },
}

impl CurveSegment {
    pub fn start(&self) -> Point {
        match self {
            CurveSegment::Line { from, .. } => *from,
            CurveSegment::Arc { from, .. } => *from,
        }
    }

    pub fn end(&self) -> Point {
        match self {
            CurveSegment::Line { to, .. } => *to,
            CurveSegment::Arc { to, .. } => *to,
        }
    }

    /// Conservative bounding box for this segment.
    pub fn bounding_box(&self) -> Rectangle {
        match self {
            CurveSegment::Line { from, to } => {
                Rectangle::from_points(*from, *to)
            }
            CurveSegment::Arc { from, to, center, .. } => {
                // Conservative: bbox of the three defining points.
                // A tighter bbox would require checking axis crossings,
                // but this is sufficient for the router.
                let mut bb = Rectangle::from_points(*from, *to);
                bb.add_point(*center);
                bb
            }
        }
    }
}

/// A compound curve consisting of connected line segments and arcs.
#[derive(Clone, Debug)]
pub struct Curve {
    segments: Vec<CurveSegment>,
}

impl Curve {
    pub fn new() -> Self {
        Self { segments: Vec::new() }
    }

    /// Add a line segment.
    pub fn add_line(&mut self, from: Point, to: Point) {
        self.segments.push(CurveSegment::Line { from, to });
    }

    /// Add an arc segment.
    pub fn add_arc(&mut self, from: Point, to: Point, center: Point, ccw: bool) {
        self.segments.push(CurveSegment::Arc { from, to, center, ccw });
    }

    /// Number of segments.
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    /// Start point of the first segment. Panics if empty.
    pub fn start(&self) -> Point {
        self.segments.first().expect("empty curve").start()
    }

    /// End point of the last segment. Panics if empty.
    pub fn end(&self) -> Point {
        self.segments.last().expect("empty curve").end()
    }

    /// Iterate over segments.
    pub fn segments(&self) -> impl Iterator<Item = &CurveSegment> {
        self.segments.iter()
    }

    /// Bounding box enclosing all segments.
    pub fn bounding_box(&self) -> Rectangle {
        let mut bb = Rectangle::empty();
        for seg in &self.segments {
            bb.add_rect(&seg.bounding_box());
        }
        bb
    }

    /// Convert a Polyline into a Curve of line segments.
    pub fn from_polyline(poly: &Polyline) -> Self {
        let mut curve = Curve::new();
        let points: Vec<Point> = poly.points().collect();
        for pair in points.windows(2) {
            curve.add_line(pair[0], pair[1]);
        }
        if poly.is_closed() && points.len() >= 2 {
            curve.add_line(*points.last().unwrap(), points[0]);
        }
        curve
    }
}

impl Default for Curve {
    fn default() -> Self {
        Self::new()
    }
}
```

- [ ] **Step 4: Update lib.rs to expose curve module**

Ensure `src/lib.rs` has:
```rust
pub use geometry::curve::{Curve, CurveSegment};
```

And add `pub mod curve;` to the `src/geometry/mod.rs` re-export line for `curve`.

- [ ] **Step 5: Run curve tests**

Run: `cd msagl-rust && cargo test --test geometry -- curve_tests`
Expected: All 7 tests pass.

- [ ] **Step 6: Commit**

```bash
git add msagl-rust/src/geometry/curve.rs msagl-rust/tests/geometry/curve_tests.rs msagl-rust/src/lib.rs msagl-rust/src/geometry/mod.rs
git commit -m "feat(msagl-rust): add Curve compound type with line and arc segments"
```

---

### Task 7: Final Integration — Compile + Full Test Suite

**Files:**
- Modify: `msagl-rust/src/lib.rs` (ensure all re-exports are clean)

- [ ] **Step 1: Run full test suite**

Run: `cd msagl-rust && cargo test`
Expected: All tests pass (~65 total across all modules).

- [ ] **Step 2: Run clippy for lint warnings**

Run: `cd msagl-rust && cargo clippy -- -D warnings`
Expected: No warnings.

- [ ] **Step 3: Fix any clippy issues**

Apply clippy suggestions if any. Common ones: unused imports, redundant clones, missing `#[must_use]`.

- [ ] **Step 4: Run clippy again to confirm clean**

Run: `cd msagl-rust && cargo clippy -- -D warnings`
Expected: Clean pass.

- [ ] **Step 5: Commit any clippy fixes**

```bash
git add -A msagl-rust/
git commit -m "chore(msagl-rust): fix clippy warnings in Phase 1 foundation"
```

- [ ] **Step 6: Verify final state**

Run: `cd msagl-rust && cargo test && cargo clippy -- -D warnings`
Expected: All tests pass, no warnings. Phase 1 complete.

---

## Summary

| Task | What | Files | Tests |
|------|------|-------|-------|
| 1 | Project scaffolding | Cargo.toml, lib.rs, mod.rs, arenas.rs | 0 |
| 2 | GeomConstants | point_comparer.rs | 5 |
| 3 | Point | point.rs | 18 |
| 4 | Rectangle | rectangle.rs | 17 |
| 5 | Polyline | polyline.rs, arenas.rs | 18 |
| 6 | Curve | curve.rs | 7 |
| 7 | Integration | lint + verify | 0 |
| **Total** | | **8 source files** | **~65 tests** |

## What Comes Next

**Phase 2: Projection Solver** — The QPSC constrained optimization solver. Independent of the routing code, validated against 86 C# fixture files. Plan will be written separately.
