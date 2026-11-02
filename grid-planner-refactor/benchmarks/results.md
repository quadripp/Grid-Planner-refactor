# Benchmark results

- Python 3.13.17 (CPython)
- numpy 2.5.3, scipy 1.18.1
- CPU: Intel(R) Xeon(R) Processor @ 2.80GHz, 2 logical cores
- OS: Linux-6.18.44-fc-v165-x86_64-with-glibc2.39

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

Median of 9 runs for the refactored code. Original runs: example 2.0 m: 5, example 1.0 m: 5, example 0.5 m: 3, example 0.25 m: 1, random 100 m 0.5 m: 1. Peak memory from one separate tracemalloc run per row.
