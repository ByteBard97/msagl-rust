//! Multiple obstacles and edges demonstrating parallel routing.
//!
//! Run with: `cargo run --example multi_edge`

use msagl_rust::{EdgeGeometry, FloatingPort, Point, RectilinearEdgeRouter, Shape};

fn main() {
    // Four rectangles in a grid layout
    let shapes = vec![
        Shape::rectangle(0.0, 0.0, 100.0, 50.0),     // top-left
        Shape::rectangle(300.0, 0.0, 100.0, 50.0),   // top-right
        Shape::rectangle(0.0, 200.0, 100.0, 50.0),   // bottom-left
        Shape::rectangle(300.0, 200.0, 100.0, 50.0), // bottom-right
    ];

    let mut router = RectilinearEdgeRouter::new(&shapes)
        .padding(4.0)
        .edge_separation(8.0);

    // Six edges connecting all pairs horizontally and vertically
    let edges = [
        // Horizontal edges
        (0, Point::new(104.0, 25.0), 1, Point::new(296.0, 25.0)),
        (2, Point::new(104.0, 225.0), 3, Point::new(296.0, 225.0)),
        // Vertical edges
        (0, Point::new(50.0, 54.0), 2, Point::new(50.0, 196.0)),
        (1, Point::new(350.0, 54.0), 3, Point::new(350.0, 196.0)),
        // Diagonal edges (will be routed orthogonally)
        (0, Point::new(104.0, 25.0), 3, Point::new(296.0, 225.0)),
        (1, Point::new(296.0, 25.0), 2, Point::new(104.0, 225.0)),
    ];

    for (src_obs, src_pt, tgt_obs, tgt_pt) in &edges {
        router.add_edge(EdgeGeometry::new(
            FloatingPort::new(*src_obs, *src_pt),
            FloatingPort::new(*tgt_obs, *tgt_pt),
        ));
    }

    let result = router.run();

    println!("Routed {} edge(s)", result.edges.len());
    for (i, edge) in result.edges.iter().enumerate() {
        let bend_count = edge.points.len().saturating_sub(2);
        println!(
            "  Edge {}: {} waypoints, {} bend(s)",
            i,
            edge.points.len(),
            bend_count
        );
    }

    // Print detailed paths
    for (i, edge) in result.edges.iter().enumerate() {
        println!("\nEdge {} path:", i);
        for pt in &edge.points {
            println!("  ({:.1}, {:.1})", pt.x(), pt.y());
        }
    }
}
