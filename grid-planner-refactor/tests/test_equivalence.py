"""Check that the refactored planners give the same answers as the originals.

For every case both versions are built on the same obstacles. The obstacle
maps must be identical and the paths must have the same length. Paths may
take a different route when two routes tie, so coordinates are not compared
directly; instead the refactored path is checked for validity on its own.
"""

import contextlib
import io
import math

import numpy as np
import pytest

from cases import (
    A_STAR_START_GOAL,
    DIJKSTRA_START_GOAL,
    EXAMPLE_ROBOT_RADIUS,
    example_map,
    load_original,
    random_map,
)
from grid_planners import AStarPlanner, DijkstraPlanner

original_dijkstra = load_original("dijkstra")
original_a_star = load_original("a_star")

PLANNERS = {
    "dijkstra": (original_dijkstra.DijkstraPlanner, DijkstraPlanner, DIJKSTRA_START_GOAL),
    "a_star": (original_a_star.AStarPlanner, AStarPlanner, A_STAR_START_GOAL),
}

# (seed, resolution, robot_radius). Seeds 5 and 23 at 0.2 m / 1.0 m and seed
# 1 at 0.2 m / 0.5 m each contain a cell exactly one radius from an obstacle,
# where a plain KD-tree distance and math.hypot disagree in the last bit.
RANDOM_CASES = [
    (0, 0.5, 1.0),
    (2, 0.3, 1.0),
    (25, 0.4, 1.5),
    (5, 0.2, 1.0),
    (23, 0.2, 1.0),
    (1, 0.2, 0.5),
]


def build_cases() -> list:
    cases = []
    ox, oy = example_map()
    for resolution in (2.0, 1.0, 0.5):
        for name in PLANNERS:
            start_goal = PLANNERS[name][2]
            cases.append(
                pytest.param(
                    name, ox, oy, resolution, EXAMPLE_ROBOT_RADIUS, start_goal,
                    id=f"{name}-example-res{resolution}",
                )
            )
    for seed, resolution, radius in RANDOM_CASES:
        rox, roy, start_goal = random_map(seed, size=30, n_walls=6, n_points=150)
        for name in PLANNERS:
            cases.append(
                pytest.param(
                    name, rox, roy, resolution, radius, start_goal,
                    id=f"{name}-random-seed{seed}-res{resolution}-rr{radius}",
                )
            )
    return cases


def path_length(rx: list[float], ry: list[float]) -> float:
    return sum(
        math.hypot(rx[i + 1] - rx[i], ry[i + 1] - ry[i]) for i in range(len(rx) - 1)
    )


def assert_valid_path(planner, rx, ry, start_goal) -> None:
    """The path must run goal -> start in single 8-connected steps on free cells."""
    sx, sy, gx, gy = start_goal
    res = planner.resolution
    assert len(rx) == len(ry) > 0

    def cell(x: float, y: float) -> tuple[int, int]:
        return round((x - planner.min_x) / res), round((y - planner.min_y) / res)

    assert cell(rx[0], ry[0]) == cell(gx, gy), "path must start at the goal"
    assert cell(rx[-1], ry[-1]) == cell(sx, sy), "path must end at the start"

    cells = [cell(x, y) for x, y in zip(rx, ry)]
    allowed_steps = {(dx, dy) for dx, dy, _ in planner.motion}
    for (x0, y0), (x1, y1) in zip(cells, cells[1:]):
        assert (x1 - x0, y1 - y0) in allowed_steps

    for ix, iy in cells:
        assert 0 <= ix < planner.x_width and 0 <= iy < planner.y_width
        assert not planner.obstacle_map[ix][iy], f"path crosses occupied cell {(ix, iy)}"


@pytest.mark.parametrize("name, ox, oy, resolution, radius, start_goal", build_cases())
def test_same_map_and_path_length(name, ox, oy, resolution, radius, start_goal):
    original_cls, refactored_cls, _ = PLANNERS[name]

    # The originals print the grid size and "Find goal"; keep test output clean.
    with contextlib.redirect_stdout(io.StringIO()):
        original = original_cls(ox, oy, resolution, radius)
        original_rx, original_ry = original.planning(*start_goal)

    refactored = refactored_cls(ox, oy, resolution, radius)
    rx, ry = refactored.planning(*start_goal)

    # Every case here is meant to be solvable. A* returns just the goal point
    # when it fails, so check the original really found a path.
    assert len(original_rx) > 1

    original_map = np.array(original.obstacle_map, dtype=bool)
    assert np.array_equal(original_map, refactored.obstacle_map)
    assert (refactored.x_width, refactored.y_width) == (original.x_width, original.y_width)

    assert path_length(rx, ry) == pytest.approx(
        path_length(original_rx, original_ry), abs=1e-9
    )
    assert_valid_path(refactored, rx, ry, start_goal)


def test_radius_tie_matches_math_hypot():
    """A cell exactly one radius from an obstacle, by float rounding.

    cKDTree reports distance 1.0 here but math.hypot gives
    1.0000000000000002, so the original marks the cell free. The refactored
    map must agree.
    """
    ox, oy = [0.0, 3.4, 10.0], [0.0, 1.4, 10.0]
    with contextlib.redirect_stdout(io.StringIO()):
        original = original_dijkstra.DijkstraPlanner(ox, oy, 0.2, 1.0)
    refactored = DijkstraPlanner(ox, oy, 0.2, 1.0)

    assert math.hypot(3.4 - 21 * 0.2, 1.4 - 4 * 0.2) > 1.0
    assert original.obstacle_map[21][4] is False
    assert not refactored.obstacle_map[21][4]
    assert np.array_equal(np.array(original.obstacle_map, dtype=bool), refactored.obstacle_map)


def walled_off_goal_map() -> tuple[list[float], list[float]]:
    """A 10 m box split in two by a wall at x = 5."""
    ox: list[float] = []
    oy: list[float] = []
    for i in range(11):
        for x, y in ((i, 0), (i, 10), (0, i), (10, i), (5, i)):
            ox.append(float(x))
            oy.append(float(y))
    return ox, oy


def test_unreachable_goal_returns_empty_lists():
    ox, oy = walled_off_goal_map()
    assert DijkstraPlanner(ox, oy, 1.0, 0.5).planning(2.0, 2.0, 8.0, 8.0) == ([], [])
    assert AStarPlanner(ox, oy, 1.0, 0.5).planning(2.0, 2.0, 8.0, 8.0) == ([], [])


def test_unreachable_goal_original_behaviour():
    """Documents what the originals do, as described in the README."""
    ox, oy = walled_off_goal_map()
    with contextlib.redirect_stdout(io.StringIO()):
        # Dijkstra calls min() on an empty open set.
        with pytest.raises(ValueError):
            original_dijkstra.DijkstraPlanner(ox, oy, 1.0, 0.5).planning(2.0, 2.0, 8.0, 8.0)
        # A* prints "Open set is empty.." and returns only the goal point.
        result = original_a_star.AStarPlanner(ox, oy, 1.0, 0.5).planning(2.0, 2.0, 8.0, 8.0)
    assert result == ([8.0], [8.0])


def test_start_outside_grid_raises():
    ox, oy = example_map()
    planner = AStarPlanner(ox, oy, 2.0, 1.0)
    with pytest.raises(ValueError):
        planner.planning(-100.0, 0.0, 50.0, 50.0)
