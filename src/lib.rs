pub mod arenas;
pub mod geometry;
pub mod projection_solver;
pub mod routing;
pub mod visibility;

pub use geometry::curve::{Curve, CurveSegment, LineSegment};
pub use geometry::point::Point;
pub use geometry::point_comparer::GeomConstants;
pub use geometry::polyline::{Polyline, PolylinePoint};
pub use geometry::rectangle::Rectangle;

pub use routing::edge_geometry::EdgeGeometry;
pub use routing::port::FloatingPort;
pub use routing::rectilinear_edge_router::{RectilinearEdgeRouter, RoutedEdge, RoutingResult};
pub use routing::shape::Shape;
