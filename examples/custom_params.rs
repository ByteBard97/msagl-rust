//! Demonstrates custom routing parameters: padding, edge separation, and bend penalty.
//!
//! Run with: `cargo run --example custom_params`

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

fn main() {
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 120.0, 60.0),
        Shape::rectangle(400.0, 0.0, 120.0, 60.0),
        Shape::rectangle(200.0, 100.0, 80.0, 40.0), // obstacle in between
    ];

    // Configure routing parameters:
    //   padding:        space between obstacle boundaries and routed edges
    //   edge_separation: minimum distance between parallel edge segments
    //   bend_penalty:   percentage penalty for bends (higher = straighter paths)
    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(10.0)
        .edge_separation(12.0)
        .bend_penalty_as_percentage(8.0);

    // Two parallel edges that must route around the middle obstacle
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(124.0, 20.0)),
        FloatingPort::new(1, Point::new(396.0, 20.0)),
    ));
    router.add_edge(EdgeGeometry::new(
        FloatingPort::new(0, Point::new(124.0, 40.0)),
        FloatingPort::new(1, Point::new(396.0, 40.0)),
    ));

    let result = router.run();

    println!("Routing with custom parameters:");
    println!("  padding = 10.0");
    println!("  edge_separation = 12.0");
    println!("  bend_penalty = 8.0%");
    println!();

    for (i, edge) in result.edges.iter().enumerate() {
        println!("Edge {}:", i);
        for pt in &edge.points {
            println!("  ({:.1}, {:.1})", pt.x, pt.y);
        }
        println!();
    }
}
