"""Grid-based A* planner, refactored for speed.

Based on PathPlanning/AStar/a_star.py from PythonRobotics by Atsushi Sakai
and Nikos Kanargias (MIT license). The unchanged original is in original/.
"""

import heapq
import math
from array import array
from collections.abc import Sequence

from grid_planners.obstacle_map import MOTION_MODEL, GridMap

# Same weight as the original calc_heuristic. 1.0 keeps A* optimal.
HEURISTIC_WEIGHT = 1.0


class AStarPlanner:
    """A* shortest path on an 8-connected occupancy grid."""

    def __init__(
        self,
        ox: Sequence[float],
        oy: Sequence[float],
        resolution: float,
        rr: float,
    ) -> None:
        """Build the obstacle map.

        ox, oy: obstacle positions [m]
        resolution: grid cell size [m]
        rr: robot radius [m]
        """
        self.resolution = resolution
        self.rr = rr
        self.grid = GridMap(ox, oy, resolution, rr)

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
        stride = grid.stride
        goal_px, goal_py = divmod(goal, stride)

        # See dijkstra.py for why these are arrays and a bytearray.
        cost = array("d", [math.inf]) * grid.size
        parent = array("q", [-1]) * grid.size
        closed = bytearray(grid.size)

        cost[start] = 0.0
        # Heap entries are (f, g, cell) with f = g + heuristic. The heap is
        # ordered by f; g rides along so it does not have to be looked up.
        start_px, start_py = divmod(start, stride)
        start_f = HEURISTIC_WEIGHT * math.hypot(start_px - goal_px, start_py - goal_py)
        open_heap = [(start_f, 0.0, start)]

        while open_heap:
            _, current_cost, current = heapq.heappop(open_heap)

            # Stale copy of a cell that was already expanded through a
            # cheaper route (lazy deletion, see dijkstra.py).
            if closed[current]:
                continue

            if current == goal:
                return grid.path_from_parents(parent, goal)

            closed[current] = 1

            for offset, step_cost in steps:
                neighbour = current + offset
                if blocked[neighbour] or closed[neighbour]:
                    continue
                new_cost = current_cost + step_cost
                if new_cost < cost[neighbour]:
                    cost[neighbour] = new_cost
                    parent[neighbour] = current
                    # Straight-line distance to the goal in grid cells, as in
                    # the original. Both cells carry the same +1 border
                    # offset, so it cancels out in the subtraction.
                    px, py = divmod(neighbour, stride)
                    h = HEURISTIC_WEIGHT * math.hypot(px - goal_px, py - goal_py)
                    heapq.heappush(open_heap, (new_cost + h, new_cost, neighbour))

        # Every reachable cell was searched without finding the goal.
        return [], []
