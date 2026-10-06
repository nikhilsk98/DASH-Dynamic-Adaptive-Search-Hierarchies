# Changelog

## 0.1.0

Initial public release.

- C++17 implementation of Standard A*, Weighted A*, and DASH (`cpp/include/dash.hpp`).
- Benchmark runner `dash_bench` with median timing, separate global and probe expansion counts, and path validation.
- Python reference port with identical expansion counts (`python/dash_reference.py`).
- Path reconstruction for all three algorithms, including cell-by-cell expansion of macro-edges.
- Test suites in C++ and Python, including a random-grid comparison against breadth-first search.
- Synthetic room-map generator and a reproducible benchmark target.
- GitHub Actions workflow.

### Correctness note

The node that launches a successful probe must still be expanded normally. An early prototype skipped that expansion after a successful probe. On random grids the skipping version returned longer paths than A* in a measurable fraction of probe-triggering cases and occasionally failed to find a path that exists. The random-grid tests in `cpp/tests` and `python/tests` guard against this regression.
