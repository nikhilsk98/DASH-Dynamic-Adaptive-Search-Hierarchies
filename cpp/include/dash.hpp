// DASH: adaptive pathfinding with local search probes.
//
// Header-only C++17 implementation of
//   - Standard A* (weight = 1) and Weighted A* (weight > 1)
//   - DASH: A* plus a stagnation trigger, a bounded local A* probe, and
//     macro-edge relaxation of the probe exit into the global search.
//
// Grids are 4-connected with unit edge cost and a Manhattan heuristic.
// Global expansions and probe expansions are counted separately because
// probe work is real computation and should never be hidden.

#pragma once

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace dash {

constexpr int kInf = std::numeric_limits<int>::max() / 2;

struct Point {
  int x = 0;
  int y = 0;
};

inline int manhattan(Point a, Point b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// ---------------------------------------------------------------------------
// Grid map (MovingAI .map format)
// ---------------------------------------------------------------------------

class GridMap {
 public:
  static GridMap from_string(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
      if (!line.empty() && line.back() == '\r') line.pop_back();
      lines.push_back(line);
    }
    if (lines.size() < 5) {
      throw std::runtime_error("map is too short to be a MovingAI map");
    }

    parse_header(lines[0], "type");
    const int height = parse_int(parse_header(lines[1], "height"), "height");
    const int width = parse_int(parse_header(lines[2], "width"), "width");

    if (to_lower(trim(lines[3])) != "map") {
      throw std::runtime_error("expected 'map' header on line 4");
    }
    if (width <= 0 || height <= 0) {
      throw std::runtime_error("map width and height must be positive");
    }
    if (lines.size() < static_cast<size_t>(4 + height)) {
      throw std::runtime_error("expected " + std::to_string(height) +
                               " map rows, found " +
                               std::to_string(lines.size() - 4));
    }

    GridMap m;
    m.width_ = width;
    m.height_ = height;
    m.blocked_.assign(static_cast<size_t>(width) * height, 1);

    // Traversable MovingAI symbols. Everything else is treated as blocked.
    for (int y = 0; y < height; ++y) {
      const std::string& row = lines[4 + y];
      if (static_cast<int>(row.size()) != width) {
        throw std::runtime_error("row " + std::to_string(y) + " has width " +
                                 std::to_string(row.size()) + ", expected " +
                                 std::to_string(width));
      }
      for (int x = 0; x < width; ++x) {
        const char c = row[x];
        m.blocked_[m.node_id(x, y)] = !(c == '.' || c == 'G' || c == 'S');
      }
    }
    return m;
  }

  static GridMap load_movingai(const std::string& filename) {
    std::ifstream f(filename, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open map file: " + filename);
    std::ostringstream buf;
    buf << f.rdbuf();
    return from_string(buf.str());
  }

  int width() const { return width_; }
  int height() const { return height_; }
  int size() const { return width_ * height_; }

  int node_id(int x, int y) const { return y * width_ + x; }
  int node_id(Point p) const { return node_id(p.x, p.y); }
  Point point(int id) const { return Point{id % width_, id / width_}; }

  bool in_bounds(int x, int y) const {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
  }
  bool is_open(Point p) const {
    return in_bounds(p.x, p.y) && !blocked_[node_id(p)];
  }
  bool is_open(int id) const { return id >= 0 && id < size() && !blocked_[id]; }

  int open_cells() const {
    return static_cast<int>(std::count(blocked_.begin(), blocked_.end(), 0));
  }

  // Writes the open 4-neighbours of `id` into `out` and returns the count.
  // Order is fixed (right, left, down, up) so runs are deterministic.
  int neighbors(int id, int out[4]) const {
    const int x = id % width_;
    const int y = id / width_;
    int n = 0;
    if (x + 1 < width_ && !blocked_[id + 1]) out[n++] = id + 1;
    if (x > 0 && !blocked_[id - 1]) out[n++] = id - 1;
    if (y + 1 < height_ && !blocked_[id + width_]) out[n++] = id + width_;
    if (y > 0 && !blocked_[id - width_]) out[n++] = id - width_;
    return n;
  }

 private:
  static std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
  }
  static std::string to_lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
  }
  static std::string parse_header(const std::string& line, const std::string& key) {
    std::istringstream ls(line);
    std::string k, v;
    ls >> k >> v;
    if (to_lower(k) != key || v.empty()) {
      throw std::runtime_error("expected '" + key + " <value>' header, got: '" + line + "'");
    }
    return v;
  }
  static int parse_int(const std::string& s, const std::string& what) {
    try {
      size_t used = 0;
      const int v = std::stoi(s, &used);
      if (used != s.size()) throw std::invalid_argument("trailing characters");
      return v;
    } catch (const std::exception&) {
      throw std::runtime_error("invalid " + what + " value: '" + s + "'");
    }
  }

  int width_ = 0;
  int height_ = 0;
  std::vector<uint8_t> blocked_;
};

// ---------------------------------------------------------------------------
// Results
// ---------------------------------------------------------------------------

struct SearchResult {
  bool success = false;
  int path_cost = kInf;
  long long global_expansions = 0;
  long long probe_expansions = 0;
  int probe_invocations = 0;
  int successful_probes = 0;  // probe reached a node with h below the stalled best
  int failed_probes = 0;      // probe exhausted its budget
  int macro_edges = 0;        // successful probes that improved g(exit)
  double runtime_ms = 0.0;
  std::vector<int> path;      // node ids from start to goal (empty on failure)

  long long total_expansions() const { return global_expansions + probe_expansions; }
};

// True if `path` is a walkable 4-connected route from start to goal whose
// length equals `expected_cost`.
inline bool is_valid_path(const GridMap& m, const std::vector<int>& path,
                          Point start, Point goal, int expected_cost) {
  if (path.empty()) return false;
  if (path.front() != m.node_id(start) || path.back() != m.node_id(goal)) return false;
  for (size_t i = 0; i < path.size(); ++i) {
    if (!m.is_open(path[i])) return false;
    if (i > 0 && manhattan(m.point(path[i - 1]), m.point(path[i])) != 1) return false;
  }
  return static_cast<int>(path.size()) - 1 == expected_cost;
}

// ---------------------------------------------------------------------------
// Search engine
// ---------------------------------------------------------------------------

namespace detail {

struct Entry {
  double f;
  int g;
  int id;
};

// Lowest f first. Ties prefer larger g (deeper nodes), then smaller id so
// that runs are fully deterministic.
struct LowerPriority {
  bool operator()(const Entry& a, const Entry& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g < b.g;
    return a.id > b.id;
  }
};

class MinHeap {
 public:
  bool empty() const { return v_.empty(); }
  void clear() { v_.clear(); }
  void push(const Entry& e) {
    v_.push_back(e);
    std::push_heap(v_.begin(), v_.end(), LowerPriority{});
  }
  Entry pop() {
    std::pop_heap(v_.begin(), v_.end(), LowerPriority{});
    const Entry e = v_.back();
    v_.pop_back();
    return e;
  }

 private:
  std::vector<Entry> v_;
};

}  // namespace detail

class SearchEngine {
 public:
  explicit SearchEngine(const GridMap& map) : map_(map) {
    const size_t n = static_cast<size_t>(map.size());
    probe_g_.assign(n, kInf);
    probe_parent_.assign(n, -1);
    probe_stamp_.assign(n, 0);
  }

  // Standard A* (weight = 1) or Weighted A* (weight > 1).
  SearchResult astar(Point start, Point goal, double weight = 1.0) {
    validate_endpoints(start, goal);
    if (weight <= 0.0) throw std::invalid_argument("weight must be positive");

    const auto t0 = std::chrono::steady_clock::now();
    const size_t n = static_cast<size_t>(map_.size());
    std::vector<int> g(n, kInf);
    std::vector<int> parent(n, -1);
    detail::MinHeap open;

    const int s = map_.node_id(start);
    const int t = map_.node_id(goal);
    g[s] = 0;
    open.push({weight * manhattan(start, goal), 0, s});

    SearchResult out;
    while (!open.empty()) {
      const detail::Entry e = open.pop();
      if (e.g != g[e.id]) continue;  // stale heap entry

      ++out.global_expansions;
      if (e.id == t) {
        out.success = true;
        out.path_cost = g[t];
        break;
      }

      int nbr[4];
      const int cnt = map_.neighbors(e.id, nbr);
      for (int i = 0; i < cnt; ++i) {
        const int v = nbr[i];
        const int cand = e.g + 1;
        if (cand < g[v]) {
          g[v] = cand;
          parent[v] = e.id;
          open.push({cand + weight * manhattan(map_.point(v), goal), cand, v});
        }
      }
    }

    if (out.success) {
      for (int x = t; x != -1; x = parent[x]) out.path.push_back(x);
      std::reverse(out.path.begin(), out.path.end());
    }
    out.runtime_ms = elapsed_ms(t0);
    return out;
  }

  // DASH.
  //
  // Runs A* until the best heuristic value has not improved for `k` global
  // expansions. The node that triggered the stall is then expanded normally
  // AND used as the entry point of a bounded local A* probe. If the probe
  // reaches a node whose h is strictly below the stalled best, the probe
  // path is injected as a macro-edge:
  //
  //     g(exit) = min(g(exit), g(entry) + probe_cost)
  //
  // If the probe fails, it is discarded. The stagnation counter is reset
  // after both outcomes so a failed probe cannot re-trigger immediately.
  SearchResult dash(Point start, Point goal, int k = 50, int probe_budget = 500) {
    validate_endpoints(start, goal);
    if (k <= 0 || probe_budget <= 0) {
      throw std::invalid_argument("k and probe_budget must be positive");
    }

    const auto t0 = std::chrono::steady_clock::now();
    const size_t n = static_cast<size_t>(map_.size());
    std::vector<int> g(n, kInf);
    std::vector<int> parent(n, -1);
    std::vector<int> macro_index(n, -1);        // exit node -> index into `macros`
    std::vector<std::vector<int>> macros;       // interior nodes of each macro-edge
    detail::MinHeap open;

    const int s = map_.node_id(start);
    const int t = map_.node_id(goal);
    g[s] = 0;
    open.push({static_cast<double>(manhattan(start, goal)), 0, s});

    int h_min = manhattan(start, goal);
    int stagnation = 0;
    SearchResult out;

    while (!open.empty()) {
      const detail::Entry e = open.pop();
      if (e.g != g[e.id]) continue;

      ++out.global_expansions;
      if (e.id == t) {
        out.success = true;
        out.path_cost = g[t];
        break;
      }

      const int h = manhattan(map_.point(e.id), goal);
      if (h < h_min) {
        h_min = h;
        stagnation = 0;
      } else {
        ++stagnation;
      }

      if (stagnation >= k) {
        ++out.probe_invocations;
        const ProbeResult probe = run_probe(e.id, goal, h_min, probe_budget);
        out.probe_expansions += probe.expansions;
        stagnation = 0;  // reset on success and on failure

        if (probe.success) {
          ++out.successful_probes;
          const int cand = e.g + probe.cost;
          if (cand < g[probe.exit_id]) {
            g[probe.exit_id] = cand;
            parent[probe.exit_id] = e.id;
            macros.push_back(probe.interior);
            macro_index[probe.exit_id] = static_cast<int>(macros.size()) - 1;
            open.push({cand + static_cast<double>(manhattan(map_.point(probe.exit_id), goal)),
                       cand, probe.exit_id});
            ++out.macro_edges;
          }
        } else {
          ++out.failed_probes;
        }
      }

      // The current node is always expanded normally. Skipping this step
      // after a successful probe can lose the optimal path.
      int nbr[4];
      const int cnt = map_.neighbors(e.id, nbr);
      for (int i = 0; i < cnt; ++i) {
        const int v = nbr[i];
        const int cand = e.g + 1;
        if (cand < g[v]) {
          g[v] = cand;
          parent[v] = e.id;
          macro_index[v] = -1;  // a normal edge now supersedes any macro-edge
          open.push({cand + static_cast<double>(manhattan(map_.point(v), goal)), cand, v});
        }
      }
    }

    if (out.success) {
      for (int x = t; x != -1; x = parent[x]) {
        out.path.push_back(x);
        if (macro_index[x] >= 0) {
          const std::vector<int>& interior = macros[macro_index[x]];
          for (size_t i = interior.size(); i-- > 0;) out.path.push_back(interior[i]);
        }
      }
      std::reverse(out.path.begin(), out.path.end());
    }
    out.runtime_ms = elapsed_ms(t0);
    return out;
  }

 private:
  struct ProbeResult {
    bool success = false;
    int exit_id = -1;
    int cost = 0;
    long long expansions = 0;
    std::vector<int> interior;  // nodes strictly between entry and exit
  };

  // Bounded local A* from `entry`. Succeeds on reaching a node (other than
  // the entry) whose h is strictly below `target_h`. The working arrays are
  // allocated once and invalidated with a generation stamp, so a probe costs
  // time proportional to the work it does, not to the map size.
  ProbeResult run_probe(int entry, Point goal, int target_h, int budget) {
    if (++probe_generation_ == 0) {  // wraparound: clear stamps once
      std::fill(probe_stamp_.begin(), probe_stamp_.end(), 0u);
      probe_generation_ = 1;
    }
    const uint32_t gen = probe_generation_;
    auto touch = [&](int id) {
      if (probe_stamp_[id] != gen) {
        probe_stamp_[id] = gen;
        probe_g_[id] = kInf;
        probe_parent_[id] = -1;
      }
    };

    probe_heap_.clear();
    touch(entry);
    probe_g_[entry] = 0;
    probe_heap_.push({static_cast<double>(manhattan(map_.point(entry), goal)), 0, entry});

    ProbeResult out;
    while (!probe_heap_.empty() && out.expansions < budget) {
      const detail::Entry e = probe_heap_.pop();
      touch(e.id);
      if (e.g != probe_g_[e.id]) continue;

      ++out.expansions;
      const int h = manhattan(map_.point(e.id), goal);
      if (e.id != entry && h < target_h) {
        out.success = true;
        out.exit_id = e.id;
        out.cost = probe_g_[e.id];
        for (int x = probe_parent_[e.id]; x != entry && x != -1; x = probe_parent_[x]) {
          out.interior.push_back(x);
        }
        std::reverse(out.interior.begin(), out.interior.end());
        return out;
      }

      int nbr[4];
      const int cnt = map_.neighbors(e.id, nbr);
      for (int i = 0; i < cnt; ++i) {
        const int v = nbr[i];
        touch(v);
        const int cand = probe_g_[e.id] + 1;
        if (cand < probe_g_[v]) {
          probe_g_[v] = cand;
          probe_parent_[v] = e.id;
          probe_heap_.push({cand + static_cast<double>(manhattan(map_.point(v), goal)), cand, v});
        }
      }
    }
    return out;
  }

  void validate_endpoints(Point start, Point goal) const {
    if (!map_.is_open(start)) throw std::invalid_argument("start is outside the map or blocked");
    if (!map_.is_open(goal)) throw std::invalid_argument("goal is outside the map or blocked");
  }

  static double elapsed_ms(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  }

  const GridMap& map_;
  std::vector<int> probe_g_;
  std::vector<int> probe_parent_;
  std::vector<uint32_t> probe_stamp_;
  uint32_t probe_generation_ = 0;
  detail::MinHeap probe_heap_;
};

}  // namespace dash
