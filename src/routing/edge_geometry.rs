use super::port::FloatingPort;

/// Geometry of an edge to be routed, defined by source and target ports.
#[derive(Clone, Debug)]
pub struct EdgeGeometry {
    pub source: FloatingPort,
    pub target: FloatingPort,
}

impl EdgeGeometry {
    /// Create an edge from `source` to `target` ports.
    pub fn new(source: FloatingPort, target: FloatingPort) -> Self {
        Self { source, target }
    }
}
