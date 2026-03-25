pub mod geometry;
pub mod projection_solver;
pub mod visibility;
pub mod routing;
mod arenas;

pub use geometry::point::Point;
pub use geometry::rectangle::Rectangle;
pub use geometry::polyline::{Polyline, PolylinePoint};
pub use geometry::curve::{Curve, CurveSegment};
pub use geometry::point_comparer::GeomConstants;

pub use routing::rectilinear_edge_router::{RectilinearEdgeRouter, RoutingResult, RoutedEdge};
pub use routing::shape::Shape;
pub use routing::port::FloatingPort;
pub use routing::edge_geometry::EdgeGeometry;
