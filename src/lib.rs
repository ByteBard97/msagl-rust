//! Rectilinear edge routing engine ported from Microsoft's MSAGL.
//!
//! This crate provides orthogonal (axis-aligned) edge routing around rectangular
//! obstacles with guaranteed edge separation. The main entry point is
//! [`RectilinearEdgeRouter`].
//!
//! # Quick start
//!
//! ```
//! use msagl_rust::{RectilinearEdgeRouter, Shape, FloatingPort, EdgeGeometry, Point};
//!
//! let shapes = vec![
//!     Shape::rectangle(0.0, 0.0, 50.0, 30.0),
//!     Shape::rectangle(200.0, 0.0, 50.0, 30.0),
//! ];
//! let mut router = RectilinearEdgeRouter::new(&shapes).padding(4.0);
//! router.add_edge(EdgeGeometry::new(
//!     FloatingPort::new(0, Point::new(54.0, 15.0)),
//!     FloatingPort::new(1, Point::new(196.0, 15.0)),
//! ));
//! let result = router.run();
//! assert_eq!(result.edges.len(), 1);
//! ```

pub mod geometry;
pub mod projection_solver;
pub mod visibility;
pub mod routing;
pub mod arenas;

pub use geometry::point::Point;
pub use geometry::rectangle::Rectangle;
pub use geometry::polyline::{Polyline, PolylinePoint};
pub use geometry::curve::{Curve, CurveSegment};
pub use geometry::point_comparer::GeomConstants;

pub use routing::rectilinear_edge_router::{RectilinearEdgeRouter, RoutingResult, RoutedEdge};
pub use routing::shape::Shape;
pub use routing::port::FloatingPort;
pub use routing::edge_geometry::EdgeGeometry;
