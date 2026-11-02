# Grid planner refactor

## What this is

This repository is a rewrite of the grid-based Dijkstra and A* planners from [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics), the open-source robotics collection by [Atsushi Sakai](https://github.com/AtsushiSakai) and contributors, released under the MIT license. The upstream files are `PathPlanning/Dijkstra/dijkstra.py` and `PathPlanning/AStar/a_star.py`. Copies of both are in `original/`, unchanged byte for byte, so the two versions can be compared side by side.

The goal was to make both planners faster without changing their answers. The refactored versions build the same obstacle map and use the same grid geometry, motion model, step costs and A* heuristic. They return paths of the same length, in the same goal-to-start order.

```
original/              upstream files, unchanged
grid_planners/
    obstacle_map.py    grid geometry and obstacle map, shared by both planners
    dijkstra.py        DijkstraPlanner
    a_star.py          AStarPlanner
tests/                 pytest equivalence tests
benchmarks/            benchmark script and results.md
cases.py               test maps and a loader for the originals
pyproject.toml         pytest settings only
```

`cases.py` holds the example map from the original `main()` functions, a seeded random map generator, and a function that imports an original file with importlib. The tests and the benchmark both use it. `pyproject.toml` only adds the repository root to pytest's import path, so the tests can import `grid_planners` and `cases`.

## Why the originals are slow

Line numbers refer to `original/dijkstra.py`. `original/a_star.py` has the same structure.

Choosing the next node. Line 75 picks the next node with `min(open_set, key=lambda o: open_set[o].cost)`. That scans every node in the open set on every iteration, so each step gets slower as the open set grows. A* does the same at lines 79 to 83 of `a_star.py`, and the function it passes as `key=` also calls `calc_heuristic` for every open node on every iteration. The extra work shows in the benchmark. At 0.25 m the original A* search took 5,894 ms and the original Dijkstra search took 1,465 ms.

Building the obstacle map. Lines 185 to 193 loop over every grid cell, and for each cell over every obstacle point, calling `math.hypot` each time. The inner loop stops early when it finds an obstacle in range, but most cells are free, and a free cell checks every obstacle point. The work grows with cells times obstacles. On the example map at 0.25 m that is 78,400 cells and 372 obstacle points, so up to 29 million `math.hypot` calls.

Work per neighbour. For each of the 8 neighbours, line 103 creates a new `Node` object and line 106 computes its index. Then `verify_node` (lines 148 to 164) turns the grid index back into a float position and makes four comparisons before it looks at the obstacle map. The open and closed sets are dicts of `Node` objects, so every visited cell costs a Python object and a dict entry.

## What changed and why

The obstacle map (`grid_planners/obstacle_map.py`, `build_obstacle_map`). A cell is occupied if any obstacle point is within the robot radius. That is the same as asking whether the nearest obstacle point is within the radius, and scipy's `cKDTree` finds the nearest point without looking at every point. The new code builds a tree from the obstacle points, makes an array of all cell positions with `numpy.meshgrid`, and queries the tree once for all cells. The map is stored as a numpy bool array with the same `[ix][iy]` layout as the original list of lists. Passing `distance_upper_bound` lets the tree give up early on cells with nothing close by. In a prototype that cut the map build at 0.25 m from 31 ms to 19 ms.

Exact ties at the radius. The tree computes distance as `sqrt(dx*dx + dy*dy)`, while the original uses `math.hypot`, and the two can differ in the last bit. With obstacle coordinates on a 0.1 m lattice, some cells sit exactly one radius from an obstacle, and that last bit decides whether the cell is occupied. For example, with an obstacle at (3.4, 1.4), a cell at (4.2, 0.8) and radius 1.0, the tree says 1.0 and `math.hypot` says 1.0000000000000002, so the original marks the cell free. A plain tree query disagreed with the original in 5 of 40 random maps I generated while testing. The fix rechecks only the cells whose tree distance is within 1e-9 of the radius, using `math.hypot` against the obstacle points near that cell. On the example map that is 444 cells or fewer. With the recheck, every map in the tests matches the original exactly.

The bounds check moves out of the loop. The original `verify_node` rejects a cell whose position is below the minimum or at or above the maximum. That test depends only on the column (for x) or the row (for y), so `GridMap` evaluates it once per column and row and folds the result into one "blocked" grid together with the obstacle map. The rule is the same as before. It is just computed once instead of for every neighbour.

A border of blocked cells. The blocked grid gets a ring of blocked cells around it. Every neighbour of a real cell then has a valid index, so the search loop needs no bounds checks of its own. A prototype without the border, using explicit bounds checks, took 152 ms for the Dijkstra search at 0.25 m. With the border it took 102 ms.

Flat indices and precomputed offsets. Each cell is one integer, `(ix + 1) * stride + (iy + 1)`, where `stride` is the height of a padded column. Moving to a neighbour is one addition with an offset that is worked out once per search (`neighbour_steps`). Float positions are only computed when the final path is built, using the original's formula so the coordinates come out identical.

A bytearray for the blocked lookup. The search reads the blocked grid one cell at a time from Python, and indexing a numpy array that way creates a numpy scalar on every read. In a prototype, the Dijkstra search at 0.25 m took 132 ms with a numpy array and 101 ms with a bytearray. A plain list ran at the same speed as the bytearray. I kept the bytearray because it uses one byte per cell.

A heap with lazy deletion (`planning` in `dijkstra.py` and `a_star.py`). The open set is a `heapq` list of `(cost, cell)` tuples, so taking the cheapest cell costs O(log n) instead of a full scan. `heapq` cannot lower the priority of an entry already in the heap, so when a cheaper route to a cell turns up, the cell is pushed again. The cheaper copy comes out first. When the older copy comes out later, the cell is already marked closed and the copy is skipped. A* pushes `(f, g, cell)`, so the heuristic is computed once when a cell is pushed instead of for every open cell on every iteration.

Flat arrays instead of Node objects. Cost and parent live in `array("d")` and `array("q")`, one slot per cell, and the closed set is a bytearray. In a prototype of the Dijkstra search at 0.25 m, peak memory was 1.4 MB with arrays, 5.1 MB with lists and 9.4 MB with dicts. The array version was also the fastest of the three in that test (96 ms, against 109 ms for lists and 104 ms for dicts).

Ideas I measured and dropped. A numpy table of precomputed heuristic values made the A* search slower (101 ms against 88 ms at 0.25 m) than calling `math.hypot` on each push. Tracking the current cell's (x, y) instead of calling `divmod` gave no gain. Merging the closed and blocked bytearrays into one gave no clear gain either, and breaking A* ties toward larger g was slightly slower. None of these are in the final code.

## Results

Output of `python benchmarks/benchmark.py`, copied from `benchmarks/results.md`. This was measured on a cloud Linux VM (Intel Xeon at 2.80 GHz, 2 logical cores, Python 3.13.17, numpy 2.5.3, scipy 1.18.1). Numbers will vary by machine.

| Map | Res (m) | Cells | Planner | Map build orig (ms) | Map build new (ms) | Search orig (ms) | Search new (ms) | Total orig (ms) | Total new (ms) | Speedup | Peak mem orig (MiB) | Peak mem new (MiB) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| example | 2.0 | 1,225 | Dijkstra | 50.5 | 1.0 | 7.0 | 1.5 | 57.5 | 2.5 | 23x | 0.22 | 0.09 |
| example | 2.0 | 1,225 | A* | 51.8 | 0.8 | 14.9 | 1.4 | 66.7 | 2.2 | 31x | 0.18 | 0.09 |
| example | 1.0 | 4,900 | Dijkstra | 211 | 5.3 | 37.3 | 5.1 | 248 | 10.4 | 24x | 0.82 | 0.32 |
| example | 1.0 | 4,900 | A* | 203 | 5.4 | 89.6 | 4.9 | 293 | 10.4 | 28x | 0.56 | 0.32 |
| example | 0.5 | 19,600 | Dijkstra | 831 | 8.9 | 193 | 23.5 | 1,024 | 32.6 | 31x | 3.37 | 1.23 |
| example | 0.5 | 19,600 | A* | 906 | 8.6 | 736 | 21.2 | 1,610 | 30.1 | 54x | 2.74 | 1.23 |
| example | 0.25 | 78,400 | Dijkstra | 3,566 | 21.6 | 1,465 | 99.0 | 5,031 | 120 | 42x | 14.00 | 4.88 |
| example | 0.25 | 78,400 | A* | 3,668 | 21.9 | 5,894 | 90.9 | 9,562 | 115 | 83x | 11.30 | 4.88 |
| random 100 m | 0.5 | 40,000 | Dijkstra | 4,524 | 15.1 | 623 | 50.8 | 5,147 | 66.4 | 78x | 6.77 | 2.51 |
| random 100 m | 0.5 | 40,000 | A* | 4,485 | 14.8 | 679 | 13.5 | 5,165 | 28.6 | 181x | 1.43 | 2.51 |

Times are medians. The refactored code ran 9 times per row. The originals ran 5, 5, 3, 1 and 1 times for the five cases from the top down, because the slowest ones take several seconds each. The "random 100 m" map is `random_map(seed=7, size=100, n_walls=20, n_points=100)` from `cases.py`, with 962 obstacle points. Peak memory comes from one separate run with tracemalloc switched on, since tracemalloc slows the code down. numpy reports its allocations to tracemalloc, so the arrays are counted.

Total speedup went from 23x on the coarsest grid to 181x for A* on the random map. On the fine grids most of the original time goes into the map build, which dropped from about 3.6 s to about 22 ms at 0.25 m. The search at 0.25 m went from 1,465 ms to 99 ms for Dijkstra and from 5,894 ms to 91 ms for A*.

Peak memory was lower in every row except A* on the random map, where it went from 1.43 MiB to 2.51 MiB. The refactored peak there comes from temporary arrays during the map build. I measured 2.51 MiB at the peak of the build and 0.08 MiB still held once it finishes. The original A* expanded only 5,375 of the 40,000 cells on that map (the original Dijkstra expanded 33,913), so its dicts of `Node` objects stayed small.

## How correctness is checked

`tests/test_equivalence.py` runs the original and refactored versions on the same inputs. The cases are the example map at 2.0, 1.0 and 0.5 m, and six seeded random maps. Each random map is a 30 m box with random walls and loose points on a 0.1 m lattice. Three of them were picked because they contain the exact-radius ties described above. For every case, and for both planners, the test asserts that:

- the obstacle maps are equal (`np.array_equal`) and the grid sizes match,
- the original really found a path,
- the path lengths agree within 1e-9,
- the refactored path starts at the goal cell, ends at the start cell, moves by one of the 8 motions at each step, and never lands on an occupied cell.

Coordinates are not compared directly. When two routes have the same length, the versions can pick different ones. The original Dijkstra replaces an open node when the new cost is equal (`>=` at line 117), while the original A* and the refactored code only replace it when the new cost is lower, and the heap breaks ties in a different order from `min()`.

Separate tests cover the floating point tie example on its own, an unreachable goal in both versions (including what the originals do), and a start position outside the grid. The originals are imported from `original/` with importlib and their module-level `show_animation` is set to `False`. The matplotlib Agg backend is forced so nothing opens on screen, and their print output is discarded.

To check that the tests can fail, I switched the tie recheck off in a scratch copy. 7 of the 22 tests failed. The benchmark also refuses to report a row if the two path lengths differ.

## Trade-offs and differences from the original

Unreachable goal. The original Dijkstra raises `ValueError: min() iterable argument is empty` (line 75) once the open set runs out. The original A* prints "Open set is empty.." and returns `([gx], [gy])`, a one-point path at the goal, which is easy to mistake for a result. Both refactored planners return `([], [])`.

Edge of the grid. When the map width is not a whole multiple of the resolution, the original's position-based bounds rule can let the search step one cell past the end of the `obstacle_map` list, and it crashes with `IndexError`. A small case that shows this: obstacles at (0, 0), (5, 5) and (10, 10), resolution 3.0, robot radius 0.5, start (0, 0), goal (6, 6). Both originals raise `IndexError`. The refactored planners return a path, because the border ring blocks every cell outside the grid. The tests and benchmark maps all have walls at the edge, and on those the two versions behave the same.

Start or goal outside the grid. The refactored planners raise `ValueError` with a clear message. The original has no explicit check for this.

API. The constructors and `planning()` take the same arguments and return the same thing. The attributes `min_x`, `min_y`, `max_x`, `max_y`, `x_width`, `y_width`, `resolution`, `motion` and `obstacle_map` are still there, along with `robot_radius` on Dijkstra and `rr` on A*. `obstacle_map` is now a numpy bool array. `obstacle_map[ix][iy]` still works, but it returns `numpy.bool_` instead of `bool`, so a check written as `is False` would behave differently. `motion` is a list of tuples instead of a list of lists. The helper methods of the original classes (`Node`, `calc_position`, `verify_node`, `calc_obstacle_map`, `calc_final_path`, `calc_heuristic` and the rest) are gone, since the search no longer uses them. The refactored code prints nothing and has no plotting or `show_animation` flag.

Dependencies. The refactored code needs numpy and scipy (for `cKDTree`). The originals only need matplotlib. matplotlib stays in `requirements.txt` because the files in `original/` import it at the top.

Memory per search. Each `planning()` call allocates cost, parent and closed arrays for the whole grid, however little of it the search ends up exploring. On the random map the refactored A* search phase still peaked at 0.97 MiB, below the 1.43 MiB of the whole original run, even though the original explored only 5,375 cells.

Tie tolerance. The recheck catches tree distances within 1e-9 of the radius. The disagreement in the example above is 2.2e-16, so the margin is wide, but a tree distance off by more than 1e-9 would not be rechecked.

## Running it

Written for Python 3.10 or newer, tested on 3.13.

```
pip install -r requirements.txt
python -m pytest
python benchmarks/benchmark.py
```

The tests take about 11 seconds. The benchmark took about six minutes on the VM above, and it overwrites `benchmarks/results.md`. Most of that time is the tracemalloc runs of the original planners, which slow them down by a large factor. The slowest one took 86 s with tracemalloc on and 5.5 s without.

Using a planner directly:

```python
from grid_planners import AStarPlanner

planner = AStarPlanner(ox, oy, resolution=1.0, rr=1.0)
rx, ry = planner.planning(10.0, 10.0, 50.0, 50.0)  # goal first, start last
```
