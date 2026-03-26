//! Simplest possible example: 2 rectangles, 1 edge.
//!
//! Run with: `cargo run --example basic`

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

fn main() {
    // Two rectangular obstacles, spaced apart horizontally
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 50.0),
        Shape::rectangle(300.0, 0.0, 100.0, 50.0),
    ];

    // Create the router with default settings
    let mut router = RectilinearEdgeRouter::new(&shapes);

    // Add a single edge connecting right side of shape 0 to left side of shape 1
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(104.0, 25.0)),
        FloatingPort::new(1, Point::new(296.0, 25.0)),
    ));

    // Route all edges
    let result = router.run();

    // Print the waypoints
    println!("Routed {} edge(s)", result.edges.len());
    for (i, edge) in result.edges.iter().enumerate() {
        println!("\nEdge {}:", i);
        for pt in &edge.points {
            println!("  ({:.1}, {:.1})", pt.x, pt.y);
        }
    }
}
