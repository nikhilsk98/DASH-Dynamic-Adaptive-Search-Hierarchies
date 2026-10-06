import random
import subprocess
import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from dash_reference import GridMap, SearchEngine, is_valid_path  # noqa: E402

U_TRAP = """type octile
height 7
width 9
map
.........
..######.
..#......
..#......
..#......
..######.
.........
"""


def bfs_cost(grid: GridMap, s: int, t: int) -> int:
    """Shortest path length by breadth-first search, or -1 if unreachable."""
    dist = {s: 0}
    queue = deque([s])
    while queue:
        u = queue.popleft()
        if u == t:
            return dist[u]
        for v in grid.neighbors(u):
            if v not in dist:
                dist[v] = dist[u] + 1
                queue.append(v)
    return -1


def random_map(rng: random.Random, w: int, h: int, percent_blocked: int) -> GridMap:
    rows = ["".join("#" if rng.randrange(100) < percent_blocked else "." for _ in range(w)) for _ in range(h)]
    return GridMap.from_string(f"type octile\nheight {h}\nwidth {w}\nmap\n" + "\n".join(rows) + "\n")


class ParserTest(unittest.TestCase):
    def test_valid_map(self):
        m = GridMap.from_string(U_TRAP)
        self.assertEqual((m.width, m.height), (9, 7))
        self.assertTrue(m.is_open((0, 0)))
        self.assertFalse(m.is_open((2, 1)))
        self.assertFalse(m.is_open((-1, 0)))
        self.assertFalse(m.is_open((9, 0)))

    def test_symbols(self):
        m = GridMap.from_string("type octile\nheight 1\nwidth 5\nmap\nGS.@T\n")
        self.assertEqual([m.is_open((x, 0)) for x in range(5)], [True, True, True, False, False])

    def test_crlf(self):
        self.assertEqual(GridMap.from_string(U_TRAP.replace("\n", "\r\n")).width, 9)

    def test_bad_input(self):
        bad = [
            "type octile\nheight 2\n",
            "kind octile\nheight 1\nwidth 3\nmap\n...\n",
            "type octile\nheight 1\nwidth 3\nmap\n..\n",
            "type octile\nheight 2\nwidth 3\nmap\n...\n",
            "type octile\nheight x\nwidth 3\nmap\n...\n",
            "type octile\nheight 1\nwidth 3\nnotmap\n...\n",
        ]
        for text in bad:
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    GridMap.from_string(text)


class SearchTest(unittest.TestCase):
    def test_u_trap(self):
        m = GridMap.from_string(U_TRAP)
        e = SearchEngine(m)
        s, t = (0, 0), (5, 3)
        a, wa, d = e.astar(s, t, 1.0), e.astar(s, t, 1.5), e.dash(s, t, 5, 50)

        self.assertTrue(a.success and wa.success and d.success)
        self.assertEqual(a.path_cost, 14)
        self.assertEqual(d.path_cost, a.path_cost)
        self.assertGreaterEqual(d.probe_invocations, 1)
        self.assertGreaterEqual(wa.path_cost, a.path_cost)
        for r in (a, wa, d):
            self.assertTrue(is_valid_path(m, r.path, s, t, r.path_cost))
        self.assertEqual(d.total_expansions, d.global_expansions + d.probe_expansions)
        self.assertEqual((a.probe_expansions, a.probe_invocations), (0, 0))

    def test_no_path(self):
        m = GridMap.from_string("type octile\nheight 3\nwidth 5\nmap\n..#..\n..#..\n..#..\n")
        e = SearchEngine(m)
        s, t = (0, 0), (4, 2)
        self.assertFalse(e.astar(s, t, 1.0).success)
        self.assertFalse(e.astar(s, t, 2.0).success)
        d = e.dash(s, t, 2, 20)
        self.assertFalse(d.success)
        self.assertEqual(d.path, [])

    def test_start_equals_goal(self):
        m = GridMap.from_string(U_TRAP)
        e = SearchEngine(m)
        for r in (e.astar((0, 0), (0, 0)), e.dash((0, 0), (0, 0), 3, 10)):
            self.assertTrue(r.success)
            self.assertEqual(r.path_cost, 0)
            self.assertEqual(len(r.path), 1)

    def test_invalid_inputs(self):
        m = GridMap.from_string(U_TRAP)
        e = SearchEngine(m)
        with self.assertRaises(ValueError):
            e.astar((2, 1), (5, 3))  # blocked start
        with self.assertRaises(ValueError):
            e.astar((0, 0), (99, 99))  # out of bounds goal
        with self.assertRaises(ValueError):
            e.astar((0, 0), (5, 3), 0.0)
        with self.assertRaises(ValueError):
            e.dash((0, 0), (5, 3), 0, 10)
        with self.assertRaises(ValueError):
            e.dash((0, 0), (5, 3), 5, 0)

    def test_random_grids_match_bfs(self):
        """A*, Weighted A* and DASH are checked against BFS on random maps.

        This is the test that catches DASH losing the optimal path, for
        example if the node that launches a successful probe is not expanded.
        """
        rng = random.Random(12345)
        trials = with_probes = with_macro = 0
        for _ in range(1000):
            w, h = rng.randint(8, 30), rng.randint(8, 30)
            m = random_map(rng, w, h, rng.choice([20, 30, 40]))
            open_ids = [i for i in range(w * h) if not m.blocked[i]]
            if len(open_ids) < 2:
                continue
            si, ti = rng.sample(open_ids, 2)
            s, t = m.point(si), m.point(ti)
            truth = bfs_cost(m, si, ti)
            e = SearchEngine(m)
            trials += 1

            a = e.astar(s, t, 1.0)
            wa = e.astar(s, t, 1.5)
            d = e.dash(s, t, rng.choice([2, 3, 5, 10]), rng.choice([20, 50, 200]))

            if truth < 0:
                self.assertFalse(a.success or wa.success or d.success)
                continue
            self.assertTrue(a.success and a.path_cost == truth)
            self.assertTrue(is_valid_path(m, a.path, s, t, a.path_cost))
            self.assertTrue(wa.success and truth <= wa.path_cost <= 1.5 * truth + 1e-9)
            self.assertTrue(is_valid_path(m, wa.path, s, t, wa.path_cost))
            self.assertTrue(d.success, f"DASH failed to find an existing path {s}->{t}")
            self.assertEqual(d.path_cost, truth, f"DASH suboptimal {s}->{t}")
            self.assertTrue(is_valid_path(m, d.path, s, t, d.path_cost))
            with_probes += d.probe_invocations > 0
            with_macro += d.macro_edges > 0

        # The test must actually exercise probes and macro-edge reconstruction.
        self.assertGreater(with_probes, 150)
        self.assertGreater(with_macro, 50)


class CliTest(unittest.TestCase):
    SCRIPT = str(Path(__file__).resolve().parents[1] / "dash_reference.py")

    def run_cli(self, *args: str) -> subprocess.CompletedProcess:
        with tempfile.TemporaryDirectory() as tmp:
            map_path = Path(tmp) / "u_trap.map"
            map_path.write_text(U_TRAP, encoding="utf-8")
            return subprocess.run(
                [sys.executable, self.SCRIPT, str(map_path), *args],
                capture_output=True,
                text=True,
            )

    def test_runs_and_validates_paths(self):
        r = self.run_cli("0", "0", "5", "3", "--k", "5", "--probe-budget", "50")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("DASH / A* cost ratio:             1.0000", r.stdout)
        self.assertIn("Path check DASH         ok", r.stdout)
        self.assertNotIn("FAILED", r.stdout)

    def test_default_start_and_goal(self):
        r = self.run_cli()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("Start: (0, 0)  Goal: (8, 6)", r.stdout)

    def test_rejects_partial_coordinates(self):
        self.assertEqual(self.run_cli("1", "2", "3").returncode, 2)

    def test_rejects_non_numeric_coordinates(self):
        self.assertEqual(self.run_cli("a", "b", "c", "d").returncode, 2)


if __name__ == "__main__":
    unittest.main()
