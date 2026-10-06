// dash_bench: compare Standard A*, Weighted A* and DASH on a MovingAI grid map.

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "dash.hpp"

namespace {

struct Options {
  std::string map_file;
  std::vector<int> coords;  // empty or exactly four values: sx sy gx gy
  int k = 50;
  int probe_budget = 500;
  double weighted_w = 1.5;
  int repeats = 5;
};

bool to_int(const std::string& s, int& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const long v = std::strtol(s.c_str(), &end, 10);
  if (errno != 0 || *end != '\0' || v < INT_MIN || v > INT_MAX) return false;
  out = static_cast<int>(v);
  return true;
}

bool to_double(const std::string& s, double& out) {
  if (s.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const double v = std::strtod(s.c_str(), &end);
  if (errno != 0 || *end != '\0') return false;
  out = v;
  return true;
}

void usage(const char* prog) {
  std::fprintf(stderr,
               "usage: %s MAP [SX SY GX GY] [--k N] [--probe-budget N]\n"
               "          [--weighted-w X] [--repeats N]\n\n"
               "  MAP               path to a MovingAI .map file\n"
               "  SX SY GX GY       start and goal (x, y). Default: first and last open cell\n"
               "  --k N             DASH stagnation threshold (default 50)\n"
               "  --probe-budget N  maximum expansions per DASH probe (default 500)\n"
               "  --weighted-w X    Weighted A* heuristic weight (default 1.5)\n"
               "  --repeats N       timing runs per algorithm, median reported (default 5)\n",
               prog);
}

bool parse_args(int argc, char** argv, Options& o) {
  std::vector<std::string> positional;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: %s needs a value\n", name);
        return nullptr;
      }
      return argv[++i];
    };
    auto bad = [&](const char* name, const char* v) {
      std::fprintf(stderr, "error: invalid value for %s: '%s'\n", name, v);
      return false;
    };
    if (a == "--k") {
      const char* v = need("--k");
      if (!v) return false;
      if (!to_int(v, o.k)) return bad("--k", v);
    } else if (a == "--probe-budget") {
      const char* v = need("--probe-budget");
      if (!v) return false;
      if (!to_int(v, o.probe_budget)) return bad("--probe-budget", v);
    } else if (a == "--weighted-w") {
      const char* v = need("--weighted-w");
      if (!v) return false;
      if (!to_double(v, o.weighted_w)) return bad("--weighted-w", v);
    } else if (a == "--repeats") {
      const char* v = need("--repeats");
      if (!v) return false;
      if (!to_int(v, o.repeats)) return bad("--repeats", v);
    } else if (a == "-h" || a == "--help") {
      return false;
    } else if (a.rfind("--", 0) == 0) {
      std::fprintf(stderr, "error: unknown option %s\n", a.c_str());
      return false;
    } else {
      positional.push_back(a);
    }
  }
  if (positional.empty()) return false;
  o.map_file = positional[0];
  if (positional.size() == 5) {
    for (size_t i = 1; i < 5; ++i) {
      int v = 0;
      if (!to_int(positional[i], v)) {
        std::fprintf(stderr, "error: invalid coordinate: '%s'\n", positional[i].c_str());
        return false;
      }
      o.coords.push_back(v);
    }
  } else if (positional.size() != 1) {
    std::fprintf(stderr, "error: provide either all four coordinates SX SY GX GY or none\n");
    return false;
  }
  if (o.k <= 0 || o.probe_budget <= 0 || o.weighted_w <= 0.0 || o.repeats <= 0) {
    std::fprintf(stderr, "error: --k, --probe-budget, --weighted-w and --repeats must be positive\n");
    return false;
  }
  return true;
}

dash::Point first_open(const dash::GridMap& m, bool reverse) {
  const int n = m.size();
  for (int i = 0; i < n; ++i) {
    const int id = reverse ? n - 1 - i : i;
    if (m.is_open(id)) return m.point(id);
  }
  throw std::runtime_error("map contains no open cells");
}

double median(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

template <typename Fn>
dash::SearchResult timed(int repeats, Fn run) {
  std::vector<double> times;
  dash::SearchResult last;
  for (int i = 0; i < repeats; ++i) {
    last = run();
    times.push_back(last.runtime_ms);
  }
  last.runtime_ms = median(times);
  return last;
}

void print_row(const char* name, const dash::SearchResult& r, bool is_dash) {
  if (!r.success) {
    std::printf("%-14s no path found\n", name);
    return;
  }
  std::printf("%-14s %6d %12lld %11lld %12lld %10.3f", name, r.path_cost, r.global_expansions,
              r.probe_expansions, r.total_expansions(), r.runtime_ms);
  if (is_dash) {
    std::printf("   %5d %5d %5d   %5d", r.probe_invocations, r.successful_probes,
                r.failed_probes, r.macro_edges);
  }
  std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  if (!parse_args(argc, argv, o)) {
    usage(argv[0]);
    return 2;
  }

  try {
    const dash::GridMap map = dash::GridMap::load_movingai(o.map_file);
    dash::Point start, goal;
    if (o.coords.empty()) {
      start = first_open(map, false);
      goal = first_open(map, true);
    } else {
      start = {o.coords[0], o.coords[1]};
      goal = {o.coords[2], o.coords[3]};
    }

    std::printf("Loaded %dx%d map (%d open cells)\n", map.width(), map.height(), map.open_cells());
    std::printf("Start: (%d, %d)  Goal: (%d, %d)\n", start.x, start.y, goal.x, goal.y);
    std::printf("DASH: k=%d  probe_budget=%d   Weighted A*: w=%g   time: median of %d run%s\n\n",
                o.k, o.probe_budget, o.weighted_w, o.repeats, o.repeats == 1 ? "" : "s");

    dash::SearchEngine engine(map);
    const auto a = timed(o.repeats, [&] { return engine.astar(start, goal, 1.0); });
    const auto wa = timed(o.repeats, [&] { return engine.astar(start, goal, o.weighted_w); });
    const auto d = timed(o.repeats, [&] { return engine.dash(start, goal, o.k, o.probe_budget); });

    std::printf("%-14s %6s %12s %11s %12s %10s   %5s %5s %5s   %5s\n", "Algorithm", "Cost",
                "Global exp", "Probe exp", "Total exp", "Time ms", "Probe", "Ok", "Fail", "Macro");
    print_row("Standard A*", a, false);
    print_row("Weighted A*", wa, false);
    print_row("DASH", d, true);

    bool ok = true;
    auto check = [&](const char* name, const dash::SearchResult& r) {
      if (!r.success) return;
      const bool valid = dash::is_valid_path(map, r.path, start, goal, r.path_cost);
      std::printf("Path check %-12s %s\n", name, valid ? "ok" : "FAILED");
      ok = ok && valid;
    };
    std::printf("\n");
    check("Standard A*", a);
    check("Weighted A*", wa);
    check("DASH", d);

    if (a.success && d.success) {
      std::printf("\nDASH / A* cost ratio:             %.4f\n",
                  static_cast<double>(d.path_cost) / a.path_cost);
      std::printf("DASH / A* total expansion ratio:  %.2f  (global %.4f)\n",
                  static_cast<double>(d.total_expansions()) / a.total_expansions(),
                  static_cast<double>(d.global_expansions) / a.global_expansions);
    }
    return ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
