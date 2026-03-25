pub mod geometry;
mod arenas;

pub use geometry::point::Point;
pub use geometry::rectangle::Rectangle;
pub use geometry::polyline::{Polyline, PolylinePoint};
pub use geometry::curve::Curve;
pub use geometry::point_comparer::GeomConstants;
