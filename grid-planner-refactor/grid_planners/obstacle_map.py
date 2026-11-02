"""Grid geometry and obstacle map shared by the Dijkstra and A* planners.

The grid follows the original PythonRobotics code exactly: the same min/max
rounding, the same number of cells and the same cell positions. What changes
is how the obstacle map is computed and how it is stored for the search.
"""

import math
from collections.abc import Sequence

import numpy as np
from scipy.spatial import cKDTree

# 8-connected motion model, same moves, costs and order as the original
# get_motion_model(): (dx, dy, cost).
MOTION_MODEL: list[tuple[int, int, float]] = [
    (1, 0, 1.0),
    (0, 1, 1.0),
    (-1, 0, 1.0),
    (0, -1, 1.0),
    (-1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)),
    (1, 1, math.sqrt(2)),
]

# Cells whose nearest obstacle is this close to the robot radius are
# rechecked with math.hypot. See build_obstacle_map for the reason.
TIE_TOLERANCE = 1e-9


def build_obstacle_map(
    ox: Sequence[float],
    oy: Sequence[float],
    xs: np.ndarray,
    ys: np.ndarray,
    robot_radius: float,
) -> np.ndarray:
    """Return a bool array of shape (len(xs), len(ys)).

    Cell [ix, iy] is True when some obstacle point lies within robot_radius
    of the cell position (xs[ix], ys[iy]), which is the original's rule.
    """
    points = np.column_stack([np.asarray(ox, dtype=float), np.asarray(oy, dtype=float)])
    tree = cKDTree(points)

    # All cell positions as one (N, 2) array. indexing="ij" keeps x as the
    # first axis, so the result lines up with obstacle_map[ix][iy].
    grid_x, grid_y = np.meshgrid(xs, ys, indexing="ij")
    centres = np.column_stack([grid_x.ravel(), grid_y.ravel()])

    # The original asks "is any obstacle within the radius?" by checking every
    # obstacle for every cell. That has the same answer as "is the nearest
    # obstacle within the radius?", and a KD-tree finds the nearest obstacle
    # without looking at all of them. distance_upper_bound lets the tree give
    # up early on cells with nothing that close; those get distance inf.
    dist, _ = tree.query(centres, distance_upper_bound=robot_radius + TIE_TOLERANCE)
    occupied = dist <= robot_radius

    # The tree computes sqrt(dx*dx + dy*dy). The original uses math.hypot,
    # which is more accurate, so the two can differ in the last bit. When a
    # cell sits exactly one radius from an obstacle (common when coordinates
    # are on a 0.1 m lattice) that last bit decides occupied or free. Example:
    # obstacle (3.4, 1.4), cell (4.2, 0.8): the tree says 1.0, math.hypot says
    # 1.0000000000000002. Rechecking only these few cells with math.hypot
    # gives exactly the original's answer.
    near_radius = np.flatnonzero(np.abs(dist - robot_radius) <= TIE_TOLERANCE)
    for cell in near_radius:
        cx, cy = centres[cell]
        candidates = tree.query_ball_point((cx, cy), robot_radius + TIE_TOLERANCE)
        occupied[cell] = any(
            math.hypot(points[j, 0] - cx, points[j, 1] - cy) <= robot_radius
            for j in candidates
        )

    return occupied.reshape(len(xs), len(ys))


class GridMap:
    """Grid layout, obstacle map and a fast blocked-cell lookup for the search.

    Cells are addressed in the search by one integer (a flat index) into a
    grid that has an extra ring of blocked cells around it. Cell (ix, iy) of
    the real grid has flat index (ix + 1) * stride + (iy + 1).
    """

    def __init__(
        self,
        ox: Sequence[float],
        oy: Sequence[float],
        resolution: float,
        robot_radius: float,
    ) -> None:
        self.resolution = resolution
        self.robot_radius = robot_radius

        # Same rounding and width rules as the original calc_obstacle_map, so
        # both versions put every cell at the same position.
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        self.x_width = round((self.max_x - self.min_x) / resolution)
        self.y_width = round((self.max_y - self.min_y) / resolution)

        # Cell positions with the original formula, index * resolution + min.
        # numpy does the same float operations, so the values are identical.
        xs = np.arange(self.x_width) * resolution + self.min_x
        ys = np.arange(self.y_width) * resolution + self.min_y

        self.obstacle_map = build_obstacle_map(ox, oy, xs, ys, robot_radius)

        # The original verify_node also rejects a cell whose position is
        # outside [min, max). That rule depends only on the row or column, so
        # it is evaluated once here instead of for every neighbour in the loop.
        inside_x = (xs >= self.min_x) & (xs < self.max_x)
        inside_y = (ys >= self.min_y) & (ys < self.max_y)
        blocked = self.obstacle_map | ~inside_x[:, None] | ~inside_y[None, :]

        # Surround the grid with a one-cell ring of blocked cells. Every
        # neighbour of a real cell is then a valid index, so the search loop
        # needs no bounds checks.
        padded = np.ones((self.x_width + 2, self.y_width + 2), dtype=bool)
        padded[1:-1, 1:-1] = blocked

        # Moving one step in x jumps a whole column of the padded grid.
        self.stride = self.y_width + 2

        # A bytearray, not the numpy array, because the search reads it one
        # element at a time from Python. bytearray[i] returns a plain int,
        # while numpy[i] builds a numpy scalar each time, which made the
        # search roughly 25 to 30 percent slower when measured.
        self.blocked = bytearray(padded.astype(np.uint8).tobytes())
        self.size = len(self.blocked)

    def index_from_position(self, x: float, y: float) -> int:
        """Flat index of the cell nearest to world position (x, y)."""
        # Same rounding as the original calc_xy_index.
        ix = round((x - self.min_x) / self.resolution)
        iy = round((y - self.min_y) / self.resolution)
        # The padded layout only works for cells inside the real grid, and a
        # start or goal outside the map is almost always a caller mistake, so
        # say so clearly instead of returning a misleading result.
        if not (0 <= ix < self.x_width and 0 <= iy < self.y_width):
            raise ValueError(f"position ({x}, {y}) is outside the grid")
        return (ix + 1) * self.stride + (iy + 1)

    def neighbour_steps(self) -> list[tuple[int, float]]:
        """The motion model as (flat index offset, cost) pairs."""
        # Precomputing the offsets turns "move by (dx, dy)" into one integer
        # addition inside the search loop.
        return [(dx * self.stride + dy, cost) for dx, dy, cost in MOTION_MODEL]

    def path_from_parents(
        self, parent: Sequence[int], goal: int
    ) -> tuple[list[float], list[float]]:
        """Follow parent links from the goal back to the start.

        Returns positions in the original order: goal first, start last.
        """
        rx: list[float] = []
        ry: list[float] = []
        node = goal
        while node != -1:
            px, py = divmod(node, self.stride)
            # Remove the border offset, then use the original calc_position
            # formula so the coordinates match the original exactly.
            rx.append((px - 1) * self.resolution + self.min_x)
            ry.append((py - 1) * self.resolution + self.min_y)
            node = parent[node]
        return rx, ry
