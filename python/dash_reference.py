#!/usr/bin/env python3
"""DASH: adaptive pathfinding with local search probes (Python reference).

This is a readable port of the C++ implementation in ../cpp. It follows the
same rules (4-connected grid, unit cost, Manhattan heuristic, same neighbour
order and tie-breaking), so expansion counts match the C++ version exactly.
It is meant for reading and for testing ideas. Do NOT use it for timing:
interpreter overhead is far larger than the cost of the probes.

Algorithms:
- Standard A* (weight = 1) and Weighted A* (weight > 1)
- DASH: A* + stagnation trigger + bounded local A* probe + macro-edge relaxation

Global expansions and probe expansions are reported separately because
probe work is real computation and should not be hidden.
"""

from __future__ import annotations

import argparse
import heapq
import time
from dataclasses import dataclass, field
from pathlib import Path

Point = tuple[int, int]
INF = float("inf")


def manhattan(a: Point, b: Point) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class GridMap:
    """4-connected grid loaded from the standard MovingAI .map format."""

    OPEN_CHARS = frozenset(".GS")

    def __init__(self, width: int, height: int, blocked: list[bool]) -> None:
        self.width = width
        self.height = height
        self.blocked = blocked

    @classmethod
    def from_string(cls, text: str) -> "GridMap":
        lines = text.splitlines()
        if len(lines) < 5:
            raise ValueError("map is too short to be a MovingAI map")

        def header(line: str, key: str) -> str:
            parts = line.strip().split(maxsplit=1)
            if len(parts) != 2 or parts[0].lower() != key:
                raise ValueError(f"expected '{key} <value>' header, got: {line!r}")
            return parts[1].strip()

        header(lines[0], "type")
        try:
            height = int(header(lines[1], "height"))
            width = int(header(lines[2], "width"))
        except ValueError as exc:
            raise ValueError(f"invalid height or width: {exc}") from exc

        if lines[3].strip().lower() != "map":
            raise ValueError("expected 'map' header on line 4")
        if width <= 0 or height <= 0:
            raise ValueError("map width and height must be positive")

        rows = lines[4 : 4 + height]
        if len(rows) != height:
            raise ValueError(f"expected {height} map rows, found {len(rows)}")

        blocked = [True] * (width * height)
        for y, row in enumerate(rows):
            if len(row) != width:
                raise ValueError(f"row {y} has width {len(row)}, expected {width}")
            for x, ch in enumerate(row):
                blocked[y * width + x] = ch not in cls.OPEN_CHARS
        return cls(width, height, blocked)

    @classmethod
    def load_movingai(cls, filename: str | Path) -> "GridMap":
        return cls.from_string(Path(filename).read_text(encoding="utf-8"))

    def node_id(self, p: Point) -> int:
        return p[1] * self.width + p[0]

    def point(self, node_id: int) -> Point:
        return node_id % self.width, node_id // self.width

    def is_open(self, p: Point) -> bool:
        x, y = p
        return 0 <= x < self.width and 0 <= y < self.height and not self.blocked[y * self.width + x]

    def open_cells(self) -> int:
        return sum(1 for b in self.blocked if not b)

    def neighbors(self, node_id: int) -> list[int]:
        """Open 4-neighbours in a fixed order: right, left, down, up."""
        w, h = self.width, self.height
        x, y = node_id % w, node_id // w
        out = []
        if x + 1 < w and not self.blocked[node_id + 1]:
            out.append(node_id + 1)
        if x > 0 and not self.blocked[node_id - 1]:
            out.append(node_id - 1)
        if y + 1 < h and not self.blocked[node_id + w]:
            out.append(node_id + w)
        if y > 0 and not self.blocked[node_id - w]:
            out.append(node_id - w)
        return out


@dataclass
class SearchResult:
    success: bool = False
    path_cost: float = INF
    global_expansions: int = 0
    probe_expansions: int = 0
    probe_invocations: int = 0
    successful_probes: int = 0  # probe reached a node with h below the stalled best
    failed_probes: int = 0      # probe exhausted its budget
    macro_edges: int = 0        # successful probes that improved g(exit)
    runtime_ms: float = 0.0
    path: list[int] = field(default_factory=list)  # node ids, start to goal

    @property
    def total_expansions(self) -> int:
        return self.global_expansions + self.probe_expansions


@dataclass
class ProbeResult:
    success: bool = False
    exit_id: int = -1
    cost: int = 0
    expansions: int = 0
    interior: list[int] = field(default_factory=list)  # strictly between entry and exit


def is_valid_path(grid: GridMap, path: list[int], start: Point, goal: Point, cost: float) -> bool:
    """True if `path` is a walkable 4-connected route whose length equals `cost`."""
    if not path or path[0] != grid.node_id(start) or path[-1] != grid.node_id(goal):
        return False
    for i, node in enumerate(path):
        if not grid.is_open(grid.point(node)):
            return False
        if i > 0 and manhattan(grid.point(path[i - 1]), grid.point(node)) != 1:
            return False
    return len(path) - 1 == cost


class SearchEngine:
    def __init__(self, grid: GridMap) -> None:
        self.map = grid
        self.n = grid.width * grid.height

        # Probe workspace is reused across probes. A generation stamp avoids
        # clearing O(V) arrays before every local search.
        self._probe_g = [INF] * self.n
        self._probe_parent = [-1] * self.n
        self._probe_stamp = [0] * self.n
        self._probe_generation = 0

    def _validate(self, start: Point, goal: Point) -> None:
        if not self.map.is_open(start):
            raise ValueError(f"start is outside the map or blocked: {start}")
        if not self.map.is_open(goal):
            raise ValueError(f"goal is outside the map or blocked: {goal}")

    def astar(self, start: Point, goal: Point, weight: float = 1.0) -> SearchResult:
        """Standard A* (weight = 1) or Weighted A* (weight > 1)."""
        self._validate(start, goal)
        if weight <= 0:
            raise ValueError("weight must be positive")

        t0 = time.perf_counter()
        grid = self.map
        g = [INF] * self.n
        parent = [-1] * self.n
        s, t = grid.node_id(start), grid.node_id(goal)
        g[s] = 0
        # Heap key: lowest f, then larger g (stored negated), then smaller id.
        heap: list[tuple[float, int, int]] = [(weight * manhattan(start, goal), 0, s)]

        out = SearchResult()
        while heap:
            _f, neg_g, u = heapq.heappop(heap)
            gu = -neg_g
            if gu != g[u]:
                continue  # stale heap entry

            out.global_expansions += 1
            if u == t:
                out.success = True
                out.path_cost = g[t]
                break

            for v in grid.neighbors(u):
                cand = gu + 1
                if cand < g[v]:
                    g[v] = cand
                    parent[v] = u
                    heapq.heappush(heap, (cand + weight * manhattan(grid.point(v), goal), -cand, v))

        if out.success:
            x = t
            while x != -1:
                out.path.append(x)
                x = parent[x]
            out.path.reverse()
        out.runtime_ms = (time.perf_counter() - t0) * 1000.0
        return out

    def dash(self, start: Point, goal: Point, k: int = 50, probe_budget: int = 500) -> SearchResult:
        """DASH.

        Runs A* until the best heuristic value has not improved for `k` global
        expansions. The node that triggered the stall is then expanded normally
        AND used as the entry point of a bounded local A* probe. If the probe
        reaches a node whose h is strictly below the stalled best, the probe
        path is injected as a macro-edge:

            g(exit) = min(g(exit), g(entry) + probe_cost)

        If the probe fails it is discarded. The stagnation counter is reset
        after both outcomes so a failed probe cannot re-trigger immediately.
        """
        self._validate(start, goal)
        if k <= 0 or probe_budget <= 0:
            raise ValueError("k and probe_budget must be positive")

        t0 = time.perf_counter()
        grid = self.map
        g = [INF] * self.n
        parent = [-1] * self.n
        macro_index = [-1] * self.n          # exit node -> index into `macros`
        macros: list[list[int]] = []         # interior nodes of each macro-edge
        s, t = grid.node_id(start), grid.node_id(goal)
        g[s] = 0
        heap: list[tuple[float, int, int]] = [(float(manhattan(start, goal)), 0, s)]

        h_min = manhattan(start, goal)
        stagnation = 0
        out = SearchResult()

        while heap:
            _f, neg_g, u = heapq.heappop(heap)
            gu = -neg_g
            if gu != g[u]:
                continue

            out.global_expansions += 1
            if u == t:
                out.success = True
                out.path_cost = g[t]
                break

            h = manhattan(grid.point(u), goal)
            if h < h_min:
                h_min = h
                stagnation = 0
            else:
                stagnation += 1

            if stagnation >= k:
                out.probe_invocations += 1
                probe = self._probe(u, goal, h_min, probe_budget)
                out.probe_expansions += probe.expansions
                stagnation = 0  # reset on success and on failure

                if probe.success:
                    out.successful_probes += 1
                    cand = gu + probe.cost
                    if cand < g[probe.exit_id]:
                        g[probe.exit_id] = cand
                        parent[probe.exit_id] = u
                        macros.append(probe.interior)
                        macro_index[probe.exit_id] = len(macros) - 1
                        heapq.heappush(
                            heap,
                            (cand + manhattan(grid.point(probe.exit_id), goal), -cand, probe.exit_id),
                        )
                        out.macro_edges += 1
                else:
                    out.failed_probes += 1

            # The current node is always expanded normally. Skipping this step
            # after a successful probe can lose the optimal path.
            for v in grid.neighbors(u):
                cand = gu + 1
                if cand < g[v]:
                    g[v] = cand
                    parent[v] = u
                    macro_index[v] = -1  # a normal edge now supersedes any macro-edge
                    heapq.heappush(heap, (cand + manhattan(grid.point(v), goal), -cand, v))

        if out.success:
            x = t
            while x != -1:
                out.path.append(x)
                if macro_index[x] >= 0:
                    out.path.extend(reversed(macros[macro_index[x]]))
                x = parent[x]
            out.path.reverse()
        out.runtime_ms = (time.perf_counter() - t0) * 1000.0
        return out

    def _probe(self, entry: int, goal: Point, target_h: int, budget: int) -> ProbeResult:
        """Bounded local A* from `entry`.

        Succeeds on reaching a node (other than the entry) whose h is strictly
        below `target_h`.
        """
        grid = self.map
        self._probe_generation += 1
        gen = self._probe_generation
        pg, pparent, stamp = self._probe_g, self._probe_parent, self._probe_stamp

        def touch(node: int) -> None:
            if stamp[node] != gen:
                stamp[node] = gen
                pg[node] = INF
                pparent[node] = -1

        touch(entry)
        pg[entry] = 0
        heap: list[tuple[float, int, int]] = [(float(manhattan(grid.point(entry), goal)), 0, entry)]

        out = ProbeResult()
        while heap and out.expansions < budget:
            _f, neg_g, u = heapq.heappop(heap)
            touch(u)
            gu = -neg_g
            if gu != pg[u]:
                continue

            out.expansions += 1
            h = manhattan(grid.point(u), goal)
            if u != entry and h < target_h:
                out.success = True
                out.exit_id = u
                out.cost = int(pg[u])
                x = pparent[u]
                while x != entry and x != -1:
                    out.interior.append(x)
                    x = pparent[x]
                out.interior.reverse()
                return out

            for v in grid.neighbors(u):
                touch(v)
                cand = gu + 1
                if cand < pg[v]:
                    pg[v] = cand
                    pparent[v] = u
                    heapq.heappush(heap, (cand + manhattan(grid.point(v), goal), -cand, v))
        return out


def first_open(grid: GridMap, reverse: bool = False) -> Point:
    ids = range(grid.width * grid.height - 1, -1, -1) if reverse else range(grid.width * grid.height)
    for node in ids:
        if not grid.blocked[node]:
            return grid.point(node)
    raise ValueError("map contains no open cells")


def format_row(name: str, r: SearchResult, is_dash: bool) -> str:
    if not r.success:
        return f"{name:<14} no path found"
    text = (
        f"{name:<14} {int(r.path_cost):>6} {r.global_expansions:>12} {r.probe_expansions:>11} "
        f"{r.total_expansions:>12} {r.runtime_ms:>10.1f}"
    )
    if is_dash:
        text += f"   {r.probe_invocations:>5} {r.successful_probes:>5} {r.failed_probes:>5}   {r.macro_edges:>5}"
    return text


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Compare Standard A*, Weighted A* and DASH on a MovingAI map.")
    p.add_argument("map_file", help="path to a MovingAI .map file")
    p.add_argument("coords", nargs="*", type=int, help="optional SX SY GX GY")
    p.add_argument("--k", type=int, default=50, help="DASH stagnation threshold (default 50)")
    p.add_argument("--probe-budget", type=int, default=500, help="max expansions per probe (default 500)")
    p.add_argument("--weighted-w", type=float, default=1.5, help="Weighted A* weight (default 1.5)")
    args = p.parse_args()
    if len(args.coords) not in (0, 4):
        p.error("provide either all four coordinates SX SY GX GY or none")
    return args


def main() -> int:
    args = parse_args()
    grid = GridMap.load_movingai(args.map_file)
    if args.coords:
        start, goal = (args.coords[0], args.coords[1]), (args.coords[2], args.coords[3])
    else:
        start, goal = first_open(grid), first_open(grid, reverse=True)

    print(f"Loaded {grid.width}x{grid.height} map ({grid.open_cells()} open cells)")
    print(f"Start: {start}  Goal: {goal}")
    print(f"DASH: k={args.k}  probe_budget={args.probe_budget}   Weighted A*: w={args.weighted_w:g}\n")

    engine = SearchEngine(grid)
    a = engine.astar(start, goal, 1.0)
    wa = engine.astar(start, goal, args.weighted_w)
    d = engine.dash(start, goal, args.k, args.probe_budget)

    print(
        f"{'Algorithm':<14} {'Cost':>6} {'Global exp':>12} {'Probe exp':>11} {'Total exp':>12} "
        f"{'Time ms':>10}   {'Probe':>5} {'Ok':>5} {'Fail':>5}   {'Macro':>5}"
    )
    print(format_row("Standard A*", a, False))
    print(format_row("Weighted A*", wa, False))
    print(format_row("DASH", d, True))

    ok = True
    print()
    for name, r in (("Standard A*", a), ("Weighted A*", wa), ("DASH", d)):
        if r.success:
            valid = is_valid_path(grid, r.path, start, goal, r.path_cost)
            print(f"Path check {name:<12} {'ok' if valid else 'FAILED'}")
            ok = ok and valid

    if a.success and d.success:
        print(f"\nDASH / A* cost ratio:             {d.path_cost / a.path_cost:.4f}")
        print(
            f"DASH / A* total expansion ratio:  {d.total_expansions / a.total_expansions:.2f}"
            f"  (global {d.global_expansions / a.global_expansions:.4f})"
        )
    print("\nNote: timings from the Python reference are not comparable to the C++ build.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
