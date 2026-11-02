"""Time and memory comparison of the original and refactored planners.

Run from the repository root:

    python benchmarks/benchmark.py

Prints a markdown table and writes it to benchmarks/results.md.

A full run took about six minutes on a 2-core cloud VM. Most of that time is
the tracemalloc runs of the original planners, which slow them down a lot.
"""

import contextlib
import gc
import math
import os
import platform
import statistics
import sys
import time
import tracemalloc
from dataclasses import dataclass
from pathlib import Path

# Make the repository root importable when this file is run as a script.
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import numpy as np  # noqa: E402
import scipy  # noqa: E402

from cases import (  # noqa: E402
    A_STAR_START_GOAL,
    DIJKSTRA_START_GOAL,
    EXAMPLE_ROBOT_RADIUS,
    example_map,
    load_original,
    random_map,
)
from grid_planners import AStarPlanner, DijkstraPlanner  # noqa: E402

original_dijkstra = load_original("dijkstra")
original_a_star = load_original("a_star")

# Repeats per measurement. The median is reported. The slowest original
# cases take several seconds each, so they run fewer times to keep the whole
# benchmark under about three minutes.
REFACTORED_REPEATS = 9


@dataclass
class Case:
    label: str
    ox: list[float]
    oy: list[float]
    resolution: float
    robot_radius: float
    dijkstra_start_goal: tuple[float, float, float, float]
    a_star_start_goal: tuple[float, float, float, float]
    original_repeats: int


def build_cases() -> list[Case]:
    ox, oy = example_map()
    cases = [
        Case("example", ox, oy, res, EXAMPLE_ROBOT_RADIUS,
             DIJKSTRA_START_GOAL, A_STAR_START_GOAL, repeats)
        for res, repeats in ((2.0, 5), (1.0, 5), (0.5, 3), (0.25, 1))
    ]
    # 100 m x 100 m with 962 obstacle points, seed chosen so the goal is
    # reachable. Same start and goal for both planners.
    rox, roy, start_goal = random_map(seed=7, size=100, n_walls=20, n_points=100)
    cases.append(Case("random 100 m", rox, roy, 0.5, 1.0, start_goal, start_goal, 1))
    return cases


def run_once(planner_cls, case: Case, start_goal) -> tuple[float, float, object, list, list]:
    """Build the planner and search once.

    Returns (build seconds, search seconds, planner, rx, ry).
    """
    gc.collect()
    # The originals print while they work; send that to /dev/null.
    with open(os.devnull, "w") as sink, contextlib.redirect_stdout(sink):
        t0 = time.perf_counter()
        planner = planner_cls(case.ox, case.oy, case.resolution, case.robot_radius)
        t1 = time.perf_counter()
        rx, ry = planner.planning(*start_goal)
        t2 = time.perf_counter()
    return t1 - t0, t2 - t1, planner, rx, ry


def peak_memory_mib(planner_cls, case: Case, start_goal) -> float:
    """Peak traced memory for one build plus search, in MiB.

    This is a separate run from the timing runs because tracemalloc slows
    Python code down a lot (the originals ran about 2.5 times slower with it
    on). numpy reports its array allocations to tracemalloc, so they count.
    """
    gc.collect()
    with open(os.devnull, "w") as sink, contextlib.redirect_stdout(sink):
        tracemalloc.start()
        planner = planner_cls(case.ox, case.oy, case.resolution, case.robot_radius)
        planner.planning(*start_goal)
        _, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
    return peak / 2**20


def measure(planner_cls, case: Case, start_goal, repeats: int) -> dict:
    builds, searches, totals = [], [], []
    for _ in range(repeats):
        build_s, search_s, planner, rx, ry = run_once(planner_cls, case, start_goal)
        builds.append(build_s)
        searches.append(search_s)
        totals.append(build_s + search_s)
    return {
        "build": statistics.median(builds),
        "search": statistics.median(searches),
        "total": statistics.median(totals),
        "memory": peak_memory_mib(planner_cls, case, start_goal),
        "length": path_length(rx, ry),
        "cells": planner.x_width * planner.y_width,
    }


def path_length(rx: list[float], ry: list[float]) -> float:
    return sum(
        math.hypot(rx[i + 1] - rx[i], ry[i + 1] - ry[i]) for i in range(len(rx) - 1)
    )


def cpu_name() -> str:
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                if line.startswith("model name"):
                    return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def format_ms(seconds: float) -> str:
    ms = seconds * 1000
    return f"{ms:,.0f}" if ms >= 100 else f"{ms:.1f}"


def main() -> None:
    environment = [
        f"Python {platform.python_version()} ({platform.python_implementation()})",
        f"numpy {np.__version__}, scipy {scipy.__version__}",
        f"CPU: {cpu_name()}, {os.cpu_count()} logical cores",
        f"OS: {platform.platform()}",
    ]
    for line in environment:
        print(line)
    print()

    planners = [
        ("Dijkstra", original_dijkstra.DijkstraPlanner, DijkstraPlanner, "dijkstra_start_goal"),
        ("A*", original_a_star.AStarPlanner, AStarPlanner, "a_star_start_goal"),
    ]

    header = (
        "| Map | Res (m) | Cells | Planner "
        "| Map build orig (ms) | Map build new (ms) "
        "| Search orig (ms) | Search new (ms) "
        "| Total orig (ms) | Total new (ms) | Speedup "
        "| Peak mem orig (MiB) | Peak mem new (MiB) |"
    )
    rows = [header, "|" + "---|" * 13]

    started = time.perf_counter()
    for case in build_cases():
        for name, original_cls, refactored_cls, start_goal_field in planners:
            start_goal = getattr(case, start_goal_field)
            print(f"running {name} on {case.label} at {case.resolution} m ...", flush=True)

            # One untimed run of the refactored planner so first-call costs
            # (scipy internals, memory pages) do not land in the first sample.
            run_once(refactored_cls, case, start_goal)

            orig = measure(original_cls, case, start_goal, case.original_repeats)
            new = measure(refactored_cls, case, start_goal, REFACTORED_REPEATS)

            # Sanity check: the benchmark is only meaningful if both versions
            # solved the same problem equally well.
            if abs(orig["length"] - new["length"]) > 1e-9:
                raise RuntimeError(f"path lengths differ for {name} on {case.label}")

            rows.append(
                f"| {case.label} | {case.resolution} | {new['cells']:,} | {name} "
                f"| {format_ms(orig['build'])} | {format_ms(new['build'])} "
                f"| {format_ms(orig['search'])} | {format_ms(new['search'])} "
                f"| {format_ms(orig['total'])} | {format_ms(new['total'])} "
                f"| {orig['total'] / new['total']:.0f}x "
                f"| {orig['memory']:.2f} | {new['memory']:.2f} |"
            )
    elapsed = time.perf_counter() - started

    repeats_note = (
        f"Median of {REFACTORED_REPEATS} runs for the refactored code. Original runs: "
        + ", ".join(f"{c.label} {c.resolution} m: {c.original_repeats}" for c in build_cases())
        + ". Peak memory from one separate tracemalloc run per row."
    )
    table = "\n".join(rows)
    print()
    print(table)
    print()
    print(repeats_note)
    print(f"Benchmark wall time: {elapsed:.0f} s")

    results = REPO_ROOT / "benchmarks" / "results.md"
    results.write_text(
        "# Benchmark results\n\n"
        + "\n".join(f"- {line}" for line in environment)
        + "\n\n"
        + table
        + "\n\n"
        + repeats_note
        + "\n"
    )
    print(f"wrote {results.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
