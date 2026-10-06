# DASH: Adaptive Pathfinding with Local Search Probes

**Authors:** Nikhil Shivakumar, Anuj Chobe, Rajiv Menon

CS5800 Algorithms course project, Northeastern University

DASH is an A*-based pathfinder that watches for stagnation, launches a short bounded local search (a probe) to escape the stalled region, and merges the probe's route back into the main search as a macro-edge. This repository contains a C++17 implementation, a Python reference port, a test suite, and a benchmark runner for MovingAI grid maps.

The project is an **empirical study**. It does not claim that DASH is faster than A*, and it does not claim a formal proof of optimality. See [Results](#results) and [Limitations](#limitations).

## How DASH works

DASH runs one A* search, the global search, and adds a trigger and a subroutine.

1. **Detect.** Track `h_min`, the lowest heuristic value among expanded nodes. If it has not improved for `k` consecutive global expansions, the search is stalled.
2. **Probe.** Launch a bounded local A* from the node that triggered the stall, with a budget of `probe_budget` expansions. The probe succeeds when it reaches a node whose `h` is strictly below the stalled `h_min`.
3. **Merge.** On success, relax the probe exit in the global search through a macro-edge:

   ```text
   g(exit) = min(g(exit), g(entry) + probe_cost)
   ```

   The probe path is stored with the macro-edge so the final path can be rebuilt cell by cell.
4. **Resume.** The triggering node is then expanded normally and A* continues. If the probe fails, it is discarded. The stagnation counter is reset after both outcomes, so a failed probe cannot re-trigger on the very next expansion.

When the trigger never fires, DASH behaves exactly like A*.

Setup: 4-connected grid, unit edge cost, Manhattan heuristic. Ties on `f` prefer larger `g`, then smaller node id, so runs are deterministic.

## Repository layout

```text
.
|-- README.md
|-- LICENSE
|-- CHANGELOG.md
|-- Makefile
|-- cpp/
|   |-- include/dash.hpp        A*, Weighted A*, DASH, MovingAI parser
|   |-- src/bench.cpp           benchmark runner (dash_bench)
|   `-- tests/test_dash.cpp     unit tests and random-grid check against BFS
|-- python/
|   |-- dash_reference.py       readable port with identical counts
|   `-- tests/                  unit tests and random-grid check against BFS
|-- scripts/make_room_map.py    generates a synthetic room map
|-- data/                       put MovingAI maps here (not committed)
`-- results/                    saved benchmark output
```

## Quick start (C++)

Requires a C++17 compiler (g++ or clang++), `make`, and Python 3.10+ for the optional map generator and Python tests. No third-party libraries.

```bash
make                     # builds build/dash_bench and build/dash_tests
make test                # runs the C++ and Python test suites
make bench-synthetic     # generates a room map and benchmarks all three algorithms
```

Run on any MovingAI map:

```bash
./build/dash_bench data/64room_005.map                          # default start and goal
./build/dash_bench data/64room_005.map 3 510 490 496            # SX SY GX GY
./build/dash_bench data/64room_005.map 3 510 490 496 --k 50 --probe-budget 500 --weighted-w 1.5
```

| Option | Default | Meaning |
|---|---:|---|
| `SX SY GX GY` | first and last open cell | start and goal coordinates |
| `--k N` | 50 | DASH stagnation threshold |
| `--probe-budget N` | 500 | maximum expansions per probe |
| `--weighted-w X` | 1.5 | Weighted A* heuristic weight |
| `--repeats N` | 5 | timing runs per algorithm, median reported |

The runner reports path cost, global expansions, probe expansions, total expansions, median runtime, probe counts, and macro-edges. It also checks that every returned path is a valid walkable route of the reported length.

Example output (`make bench-synthetic`):

```text
Algorithm        Cost   Global exp   Probe exp    Total exp    Time ms   Probe    Ok  Fail   Macro
Standard A*       623        18991           0        18991      2.728
Weighted A*       623        16586           0        16586      2.000
DASH              623        18927      182929       201856     20.424     367     5   362       3
```

## Python reference

`python/dash_reference.py` is a readable port with the same rules and tie-breaking, so its expansion counts match the C++ build exactly. It is useful for reading the algorithm and trying ideas. Do not use it for timing: it is roughly 20 to 25 times slower, and interpreter overhead hides the real cost of probes.

```bash
python3 python/dash_reference.py data/synthetic_rooms.map 3 510 490 496
```

## Tests

`make test` runs both suites. The most important test runs A*, Weighted A* and DASH on random grids (3,000 in C++, 1,000 in Python) and compares each result with breadth-first search:

- A* cost equals the BFS shortest path length.
- Weighted A* cost is between the optimum and `w` times the optimum.
- DASH cost equals the BFS shortest path length, and DASH finds a path whenever one exists.
- Every returned path is walkable, 4-connected, and has the reported length, including paths that pass through macro-edges.
- The test confirms that probes and macro-edges actually occur, so DASH is really exercised.

The suites also cover the parser (valid maps, CRLF, bad headers, short rows), a U-shaped trap, unreachable goals, start equal to goal, and invalid inputs.

## Results

### Reproducible run in this repository

`make bench-synthetic` on a generated 512 by 512 map of 64 by 64 rooms with random doors, using the same query and parameters as the course report. Full output is in [`results/synthetic_rooms.txt`](results/synthetic_rooms.txt).

| Algorithm | Path cost | Global expansions | Probe expansions | Total expansions | Time (ms) | Probes ok / launched |
|---|---:|---:|---:|---:|---:|---:|
| Standard A* | 623 | 18,991 | 0 | 18,991 | 2.7 | n/a |
| Weighted A* (w=1.5) | 623 | 16,586 | 0 | 16,586 | 2.0 | n/a |
| DASH (k=50, budget=500) | 623 | 18,927 | 182,929 | 201,856 | 20.4 | 5 / 367 |

Timings are the median of 5 runs on a shared single-core cloud sandbox, so read them as indicative. Expansion counts and costs are deterministic.

### Course report (original run on 64room_005)

Reported in the course report for the query (3, 510) to (490, 496) on the MovingAI `64room_005` map, with k=50 and w=1.5. These figures come from the original implementation. They have **not** been regenerated with the code in this repository. See [Reproduce on the real benchmark](#reproduce-on-the-real-benchmark).

| Algorithm | Path cost | Global expansions | Time (ms) | Probes |
|---|---:|---:|---:|---:|
| Standard A* | 1131 | 127,760 | 14.3 | n/a |
| Weighted A* (w=1.5) | 1187 | 479,794 | 61.6 | n/a |
| DASH (k=50) | 1131 | 127,689 | 135.3 | 55 |

The report table counts global expansions only, so probe work is not visible in it.

### Reading the results

- **Cost.** DASH returns the same cost as A* in every test and on the benchmark runs above.
- **Global expansions.** DASH ties A*. Its saving is tiny: 64 fewer global expansions in the reproducible run.
- **Total work.** Counting probe expansions, DASH did about 10.6 times the work of A* in the reproducible run, and 362 of its 367 probes failed. That is why it is slower in wall-clock time.
- **Weighted A\*.** In the course report it expanded about 3.8 times as many nodes as A\* on `64room_005`. The synthetic map does not reproduce that: Weighted A\* expanded fewer nodes than A\* there. The blow-up depends on the map.

The practical lesson: an adaptive mechanism only pays off when the global work it avoids exceeds the cost of detecting a stall and running the probe. With a naive trigger and a probe that is itself an A*, that did not happen on these room maps.

### Reproduce on the real benchmark

Download `64room_005.map` from the [MovingAI grid benchmarks](https://www.movingai.com/benchmarks/grids.html), place it in `data/`, and run:

```bash
make
./build/dash_bench data/64room_005.map 3 510 490 496 --k 50 --probe-budget 500 --weighted-w 1.5
```

## Limitations

- Evidence comes from one query on one real map and one synthetic map. A proper evaluation would sweep many queries across room, maze, and game maps and report distributions.
- Every macro-edge weight is the cost of a real path and the heuristic is consistent, so DASH is expected to return the optimal cost. The tests check this on random grids, but that is empirical evidence, not a formal proof.
- Movement is 4-connected. MovingAI scenario files list octile optimal costs, so costs here are not comparable to those figures.
- No map has been found where DASH beats A* in wall-clock time.
- Wall-clock times come from a single machine and should not be read as precise.

## Future work

- Adaptive stagnation threshold based on local branching factor.
- A cheaper probe, for example Jump Point Search, in place of cell-by-cell A*.
- Reuse of work between probes and suppression of repeated probes in the same region.
- Running the probe on a background thread.
- A wider benchmark sweep with an ablation over `k` and the probe budget.

## References

- P. E. Hart, N. J. Nilsson, B. Raphael. A formal basis for the heuristic determination of minimum cost paths. IEEE Trans. Systems Science and Cybernetics, 1968.
- N. R. Sturtevant. Benchmarks for grid-based pathfinding. IEEE Trans. Computational Intelligence and AI in Games, 4(2):144-148, 2012.

## License

MIT License. See [`LICENSE`](LICENSE).
