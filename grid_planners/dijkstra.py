"""Grid-based Dijkstra planner, refactored for speed.

Based on PathPlanning/Dijkstra/dijkstra.py from PythonRobotics by
Atsushi Sakai (MIT license). The unchanged original is in original/.
"""

import heapq
import math
from array import array
from collections.abc import Sequence

from grid_planners.obstacle_map import MOTION_MODEL, GridMap


class DijkstraPlanner:
    """Dijkstra shortest path on an 8-connected occupancy grid."""

    def __init__(
        self,
        ox: Sequence[float],
        oy: Sequence[float],
        resolution: float,
        robot_radius: float,
    ) -> None:
        """Build the obstacle map.

        ox, oy: obstacle positions [m]
        resolution: grid cell size [m]
        robot_radius: robot radius [m]
        """
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.grid = GridMap(ox, oy, resolution, robot_radius)

        # Same public attributes as the original, for code that reads them.
        # obstacle_map is now a numpy bool array; [ix][iy] still works.
        self.min_x, self.min_y = self.grid.min_x, self.grid.min_y
        self.max_x, self.max_y = self.grid.max_x, self.grid.max_y
        self.x_width, self.y_width = self.grid.x_width, self.grid.y_width
        self.obstacle_map = self.grid.obstacle_map
        self.motion = MOTION_MODEL

    def planning(
        self, sx: float, sy: float, gx: float, gy: float
    ) -> tuple[list[float], list[float]]:
        """Find the shortest path from (sx, sy) to (gx, gy).

        Returns (rx, ry) from goal back to start, like the original, or two
        empty lists if the goal cannot be reached.
        """
        grid = self.grid
        start = grid.index_from_position(sx, sy)
        goal = grid.index_from_position(gx, gy)
        blocked = grid.blocked
        steps = grid.neighbour_steps()

        # One slot per cell instead of a Node object per cell. array("d")
        # stores raw 8-byte floats and array("q") raw 8-byte ints, which uses
        # much less memory than a list of Python objects and was just as fast.
        cost = array("d", [math.inf]) * grid.size
        parent = array("q", [-1]) * grid.size
        closed = bytearray(grid.size)

        cost[start] = 0.0
        # The heap replaces min(open_set, ...), which scanned the whole open
        # set on every iteration. heappop takes O(log n).
        open_heap = [(0.0, start)]

        while open_heap:
            current_cost, current = heapq.heappop(open_heap)

            # heapq cannot lower the cost of an entry already in the heap, so
            # a cell is pushed again whenever a cheaper route is found. The
            # cheapest copy comes out first; any later copy is stale and
            # skipped here ("lazy deletion").
            if closed[current]:
                continue

            if current == goal:
                return grid.path_from_parents(parent, goal)

            closed[current] = 1

            for offset, step_cost in steps:
                neighbour = current + offset
                # The border ring is blocked, so this one check also covers
                # the original's bounds test.
                if blocked[neighbour] or closed[neighbour]:
                    continue
                new_cost = current_cost + step_cost
                if new_cost < cost[neighbour]:
                    cost[neighbour] = new_cost
                    parent[neighbour] = current
                    heapq.heappush(open_heap, (new_cost, neighbour))

        # Every reachable cell was searched without finding the goal.
        return [], []
