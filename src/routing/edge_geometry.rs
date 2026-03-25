use super::port::FloatingPort;

/// Geometry of an edge to be routed, defined by source and target ports.
#[derive(Clone, Debug)]
pub struct EdgeGeometry {
    pub source: FloatingPort,
    pub target: FloatingPort,
}

impl EdgeGeometry {
    pub fn new(source: FloatingPort, target: FloatingPort) -> Self {
        Self { source, target }
    }
}
