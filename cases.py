"""Obstacle maps and helpers shared by the tests and the benchmark."""

import importlib.util
import random
from pathlib import Path
from types import ModuleType

import matplotlib

# The original files import matplotlib.pyplot at the top. The Agg backend
# draws nothing on screen, so they import cleanly on a machine with no display.
matplotlib.use("Agg")

REPO_ROOT = Path(__file__).resolve().parent

# Start and goal used in each original file's main() function.
DIJKSTRA_START_GOAL = (-5.0, -5.0, 50.0, 50.0)
A_STAR_START_GOAL = (10.0, 10.0, 50.0, 50.0)
EXAMPLE_ROBOT_RADIUS = 1.0


def example_map() -> tuple[list[float], list[float]]:
    """The obstacle layout from the original main() functions.

    A 70 m x 70 m box with two internal walls.
    """
    ox: list[float] = []
    oy: list[float] = []
    for i in range(-10, 60):
        ox.append(float(i))
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(float(i))
    for i in range(-10, 61):
        ox.append(float(i))
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(float(i))
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(float(i))
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)
    return ox, oy


def random_map(
    seed: int, size: int, n_walls: int, n_points: int, clearance: float = 3.0
) -> tuple[list[float], list[float], tuple[float, float, float, float]]:
    """A seeded random map: a square border, random walls and loose points.

    Returns (ox, oy, (sx, sy, gx, gy)). Start and goal sit near opposite
    corners and no obstacle is placed within `clearance` metres of them.
    Coordinates are rounded to 0.1 m, which creates many cells that sit
    exactly one robot radius from an obstacle. Those are the cases where a
    careless rewrite of the obstacle map would disagree with the original.
    """
    rng = random.Random(seed)
    sx, sy, gx, gy = 5.0, 5.0, size - 5.0, size - 5.0

    def far_from_ends(x: float, y: float) -> bool:
        return all(
            (x - px) ** 2 + (y - py) ** 2 > clearance**2
            for px, py in ((sx, sy), (gx, gy))
        )

    ox: list[float] = []
    oy: list[float] = []
    for i in range(size + 1):
        for x, y in ((i, 0), (i, size), (0, i), (size, i)):
            ox.append(float(x))
            oy.append(float(y))

    for _ in range(n_walls):
        x0 = round(rng.uniform(0, size), 1)
        y0 = round(rng.uniform(0, size), 1)
        length = rng.randint(size // 10, size // 3)
        horizontal = rng.random() < 0.5
        for k in range(length):
            x = min(size, x0 + k) if horizontal else x0
            y = y0 if horizontal else min(size, y0 + k)
            x, y = round(x, 1), round(y, 1)
            if far_from_ends(x, y):
                ox.append(x)
                oy.append(y)

    for _ in range(n_points):
        x = round(rng.uniform(0, size), 1)
        y = round(rng.uniform(0, size), 1)
        if far_from_ends(x, y):
            ox.append(x)
            oy.append(y)

    return ox, oy, (sx, sy, gx, gy)


def load_original(name: str) -> ModuleType:
    """Import original/<name>.py, the unchanged upstream file.

    importlib is used because original/ is not a package and both originals
    and the refactored code have modules called dijkstra and a_star.
    """
    path = REPO_ROOT / "original" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"original_{name}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # Module-level switch in the originals; True would open plot windows.
    module.show_animation = False
    return module
