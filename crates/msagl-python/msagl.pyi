"""Type stubs for the msagl Python bindings."""

from typing import List, Tuple

class Port:
    """A port on an obstacle. Created via Obstacle.port(x, y).

    Coordinates are absolute canvas position, not relative to the obstacle.
    """

    obstacle_index: int
    x: float
    y: float

class Obstacle:
    """A rectangular obstacle on the canvas."""

    x: float
    y: float
    width: float
    height: float

    def __init__(self, x: float, y: float, width: float, height: float) -> None: ...
    def port(self, x: float, y: float) -> Port:
        """Create a port at absolute canvas coordinates (x, y) on this obstacle."""
        ...

class Path:
    """A routed path consisting of a sequence of (x, y) waypoints."""

    points: List[Tuple[float, float]]

class RoutingResult:
    """The result of running the router, containing all routed paths."""

    paths: List[Path]

class Router:
    """Rectilinear edge router.

    Routes edges between ports on rectangular obstacles, producing
    orthogonal (axis-aligned) paths with guaranteed minimum edge separation.
    """

    def __init__(
        self,
        obstacles: List[Obstacle],
        *,
        padding: float = 4.0,
        edge_separation: float = 8.0,
        bend_penalty: float = 4.0,
    ) -> None: ...
    def add_edge(self, source: Port, target: Port) -> None: ...
    def route(self) -> RoutingResult: ...
