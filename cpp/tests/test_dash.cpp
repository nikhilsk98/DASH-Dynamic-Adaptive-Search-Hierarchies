// Self-contained tests for the DASH implementation. No external framework.
//
// The key test is the random-grid check: A*, Weighted A* and DASH are run on
// thousands of random maps and compared against breadth-first search, which
// is an independent ground truth for shortest path length on a unit-cost grid.

#include <cstdio>
#include <deque>
#include <functional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "dash.hpp"

namespace {

int g_failed = 0;
int g_checks = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_failed;                                                            \
      std::printf("  FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);        \
    }                                                                        \
  } while (0)

bool throws(const std::function<void()>& fn) {
  try {
    fn();
  } catch (const std::exception&) {
    return true;
  }
  return false;
}

// BFS shortest path length on the grid, or -1 if unreachable.
int bfs_cost(const dash::GridMap& m, dash::Point s, dash::Point t) {
  std::vector<int> dist(m.size(), -1);
  std::deque<int> q;
  const int si = m.node_id(s), ti = m.node_id(t);
  dist[si] = 0;
  q.push_back(si);
  while (!q.empty()) {
    const int u = q.front();
    q.pop_front();
    if (u == ti) return dist[u];
    int nb[4];
    const int c = m.neighbors(u, nb);
    for (int i = 0; i < c; ++i) {
      if (dist[nb[i]] < 0) {
        dist[nb[i]] = dist[u] + 1;
        q.push_back(nb[i]);
      }
    }
  }
  return -1;
}

const char* kUTrap =
    "type octile\nheight 7\nwidth 9\nmap\n"
    ".........\n"
    "..######.\n"
    "..#......\n"
    "..#......\n"
    "..#......\n"
    "..######.\n"
    ".........\n";

void test_parser() {
  std::printf("parser\n");
  const dash::GridMap m = dash::GridMap::from_string(kUTrap);
  CHECK(m.width() == 9 && m.height() == 7);
  CHECK(m.is_open(dash::Point{0, 0}));
  CHECK(!m.is_open(dash::Point{2, 1}));
  CHECK(!m.is_open(dash::Point{-1, 0}));
  CHECK(!m.is_open(dash::Point{9, 0}));

  // Windows line endings are accepted.
  std::string crlf;
  for (const char* p = kUTrap; *p; ++p) {
    if (*p == '\n') crlf += '\r';
    crlf += *p;
  }
  CHECK(dash::GridMap::from_string(crlf).width() == 9);

  // Traversable symbols G and S, everything else blocked.
  const dash::GridMap sym = dash::GridMap::from_string("type octile\nheight 1\nwidth 5\nmap\nGS.@T\n");
  CHECK(sym.is_open(dash::Point{0, 0}) && sym.is_open(dash::Point{1, 0}) && sym.is_open(dash::Point{2, 0}));
  CHECK(!sym.is_open(dash::Point{3, 0}) && !sym.is_open(dash::Point{4, 0}));

  CHECK(throws([] { dash::GridMap::from_string("type octile\nheight 2\n"); }));
  CHECK(throws([] { dash::GridMap::from_string("kind octile\nheight 1\nwidth 3\nmap\n...\n"); }));
  CHECK(throws([] { dash::GridMap::from_string("type octile\nheight 1\nwidth 3\nmap\n..\n"); }));
  CHECK(throws([] { dash::GridMap::from_string("type octile\nheight 2\nwidth 3\nmap\n...\n"); }));
  CHECK(throws([] { dash::GridMap::from_string("type octile\nheight x\nwidth 3\nmap\n...\n"); }));
  CHECK(throws([] { dash::GridMap::from_string("type octile\nheight 1\nwidth 3\nnotmap\n...\n"); }));
  CHECK(throws([] { dash::GridMap::load_movingai("does/not/exist.map"); }));
}

void test_u_trap() {
  std::printf("u-trap\n");
  const dash::GridMap m = dash::GridMap::from_string(kUTrap);
  dash::SearchEngine e(m);
  const dash::Point s{0, 0}, t{5, 3};

  const auto a = e.astar(s, t, 1.0);
  const auto wa = e.astar(s, t, 1.5);
  const auto d = e.dash(s, t, 5, 50);

  CHECK(a.success && wa.success && d.success);
  CHECK(a.path_cost == 14);
  CHECK(d.path_cost == a.path_cost);
  CHECK(d.probe_invocations >= 1);
  CHECK(dash::is_valid_path(m, a.path, s, t, a.path_cost));
  CHECK(dash::is_valid_path(m, wa.path, s, t, wa.path_cost));
  CHECK(dash::is_valid_path(m, d.path, s, t, d.path_cost));
  CHECK(wa.path_cost >= a.path_cost);
  CHECK(d.total_expansions() == d.global_expansions + d.probe_expansions);
  CHECK(a.probe_expansions == 0 && a.probe_invocations == 0);
}

void test_edge_cases() {
  std::printf("edge cases\n");
  // A full wall splits the map: no path exists.
  const dash::GridMap split = dash::GridMap::from_string(
      "type octile\nheight 3\nwidth 5\nmap\n..#..\n..#..\n..#..\n");
  dash::SearchEngine e(split);
  const dash::Point s{0, 0}, t{4, 2};
  CHECK(!e.astar(s, t, 1.0).success);
  CHECK(!e.astar(s, t, 2.0).success);
  const auto d = e.dash(s, t, 2, 20);
  CHECK(!d.success && d.path.empty());

  // Start equals goal.
  const auto same = e.astar(s, s, 1.0);
  CHECK(same.success && same.path_cost == 0 && same.path.size() == 1);
  const auto same_d = e.dash(s, s, 3, 10);
  CHECK(same_d.success && same_d.path_cost == 0 && same_d.probe_invocations == 0);

  // Invalid inputs.
  CHECK(throws([&] { e.astar(dash::Point{2, 0}, t, 1.0); }));    // blocked start
  CHECK(throws([&] { e.astar(s, dash::Point{99, 99}, 1.0); }));  // out of bounds goal
  CHECK(throws([&] { e.astar(s, t, 0.0); }));
  CHECK(throws([&] { e.dash(s, t, 0, 10); }));
  CHECK(throws([&] { e.dash(s, t, 5, 0); }));
}

dash::GridMap random_map(std::mt19937& rng, int w, int h, int percent_blocked) {
  std::ostringstream os;
  os << "type octile\nheight " << h << "\nwidth " << w << "\nmap\n";
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      os << (static_cast<int>(rng() % 100) < percent_blocked ? '#' : '.');
    }
    os << '\n';
  }
  return dash::GridMap::from_string(os.str());
}

void test_random_grids() {
  std::printf("random grids vs BFS\n");
  std::mt19937 rng(12345);
  const int ks[] = {2, 3, 5, 10};
  const int budgets[] = {20, 50, 200};
  const int blocked[] = {20, 30, 40};

  int trials = 0, with_probes = 0, with_macro = 0, unreachable = 0;
  int astar_bad = 0, wa_bad = 0, dash_bad = 0;

  for (int iter = 0; iter < 3000; ++iter) {
    const int w = 8 + static_cast<int>(rng() % 23);
    const int h = 8 + static_cast<int>(rng() % 23);
    const dash::GridMap m = random_map(rng, w, h, blocked[rng() % 3]);

    std::vector<int> open;
    for (int i = 0; i < m.size(); ++i) {
      if (m.is_open(i)) open.push_back(i);
    }
    if (open.size() < 2) continue;
    const int si = open[rng() % open.size()];
    int ti = open[rng() % open.size()];
    while (ti == si) ti = open[rng() % open.size()];
    const dash::Point s = m.point(si), t = m.point(ti);

    const int truth = bfs_cost(m, s, t);
    dash::SearchEngine e(m);
    ++trials;
    if (truth < 0) ++unreachable;

    const auto a = e.astar(s, t, 1.0);
    const auto wa = e.astar(s, t, 1.5);
    const auto d = e.dash(s, t, ks[rng() % 4], budgets[rng() % 3]);

    if (truth < 0) {
      if (a.success || wa.success || d.success) ++astar_bad;
      continue;
    }

    if (!a.success || a.path_cost != truth || !dash::is_valid_path(m, a.path, s, t, a.path_cost)) ++astar_bad;
    if (!wa.success || wa.path_cost < truth || wa.path_cost > 1.5 * truth + 1e-9 ||
        !dash::is_valid_path(m, wa.path, s, t, wa.path_cost)) {
      ++wa_bad;
    }
    if (!d.success || d.path_cost != truth || !dash::is_valid_path(m, d.path, s, t, d.path_cost)) ++dash_bad;

    if (d.probe_invocations > 0) ++with_probes;
    if (d.macro_edges > 0) ++with_macro;
  }

  std::printf("  trials=%d unreachable=%d with_probes=%d with_macro_edges=%d\n", trials,
              unreachable, with_probes, with_macro);
  std::printf("  A* mismatches=%d  Weighted A* violations=%d  DASH mismatches=%d\n", astar_bad,
              wa_bad, dash_bad);
  CHECK(astar_bad == 0);
  CHECK(wa_bad == 0);
  CHECK(dash_bad == 0);
  // Make sure the test actually exercises probes and macro-edge paths.
  CHECK(with_probes >= 500);
  CHECK(with_macro >= 200);
}

}  // namespace

int main() {
  test_parser();
  test_u_trap();
  test_edge_cases();
  test_random_grids();
  std::printf("\n%d checks, %d failed\n", g_checks, g_failed);
  return g_failed == 0 ? 0 : 1;
}
