#include "jps_vo_planner/jps3d.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>

namespace jps_vo
{
  namespace
  {
    struct QItem
    {
      double f, g;
      uint64_t key;
      // equal f (within 1e-9): prefer the larger g, i.e. the state closer to the goal; optimality is unaffected
      bool operator>(const QItem &o) const { return f > o.f + 1e-9 || (std::fabs(f - o.f) <= 1e-9 && g < o.g); }
    };

    // d' is "natural" for d (eq. 6): d' != 0 and d'_i in {0, d_i}
    bool isSubDir(const Eigen::Vector3i &dp, const Eigen::Vector3i &d)
    {
      if (dp == Eigen::Vector3i::Zero())
        return false;
      for (int i = 0; i < 3; ++i)
        if (dp(i) != 0 && dp(i) != d(i))
          return false;
      return true;
    }
  } // namespace

  Eigen::Vector3i Jps3D::dirVec(int idx) { return Eigen::Vector3i(idx / 9 - 1, (idx / 3) % 3 - 1, idx % 3 - 1); }
  int Jps3D::dirIdx(const Eigen::Vector3i &d) { return (d.x() + 1) * 9 + (d.y() + 1) * 3 + (d.z() + 1); }

  double Jps3D::hD(const Eigen::Vector3i &n, const Eigen::Vector3i &goal) const
  {
    std::array<int, 3> a{std::abs(n.x() - goal.x()), std::abs(n.y() - goal.y()), std::abs(n.z() - goal.z())};
    std::sort(a.begin(), a.end());
    return cellSize() * ((std::sqrt(3.0) - std::sqrt(2.0)) * a[0] + (std::sqrt(2.0) - 1.0) * a[1] + a[2]);
  }

  bool Jps3D::legal(const Eigen::Vector3i &x, const Eigen::Vector3i &d) const
  {
    // eq. (3): every cell x + m with m_i in {0, d_i}, m != 0, is free (no corner cutting)
    for (int mx = 0; mx <= std::abs(d.x()); ++mx)
      for (int my = 0; my <= std::abs(d.y()); ++my)
        for (int mz = 0; mz <= std::abs(d.z()); ++mz)
        {
          if (mx == 0 && my == 0 && mz == 0)
            continue;
          const Eigen::Vector3i c(x.x() + mx * d.x(), x.y() + my * d.y(), x.z() + mz * d.z());
          if (!freeCell(c))
            return false;
        }
    return true;
  }

  uint32_t Jps3D::forcedMask(const Eigen::Vector3i &x, const Eigen::Vector3i &d) const
  {
    // occupancy of the 3x3x3 window (bit k = offset dirVec(k) free; centre = x itself, free)
    uint32_t occ = 0;
    for (int k = 0; k < 27; ++k)
      if (k == kNoDir || freeCell(Eigen::Vector3i(x + dirVec(k))))
        occ |= 1u << k;
    const uint64_t key = (static_cast<uint64_t>(tie_rule_ == TieRule::Strict) << 32) |
                         (static_cast<uint64_t>(dirIdx(d)) << 27) | occ;
    auto it = forced_cache_.find(key);
    if (it != forced_cache_.end())
      return it->second;

    auto freeOff = [&](const Eigen::Vector3i &o) {
      if (o.cwiseAbs().maxCoeff() > 1)
        return false;
      return ((occ >> dirIdx(o)) & 1u) != 0;
    };
    // legality (3) of a move a -> a + m inside the window; the centre may be touched by the box (corner),
    // it is only excluded as a path vertex
    auto legalOff = [&](const Eigen::Vector3i &a, const Eigen::Vector3i &m) {
      for (int mx = 0; mx <= std::abs(m.x()); ++mx)
        for (int my = 0; my <= std::abs(m.y()); ++my)
          for (int mz = 0; mz <= std::abs(m.z()); ++mz)
          {
            if (mx == 0 && my == 0 && mz == 0)
              continue;
            if (!freeOff(Eigen::Vector3i(a.x() + mx * m.x(), a.y() + my * m.y(), a.z() + mz * m.z())))
              return false;
          }
      return true;
    };

    // shortest legal path from the parent cell p = -d to every window cell, never visiting the centre
    std::array<double, 27> dist;
    dist.fill(1e18);
    std::array<bool, 27> done{};
    const Eigen::Vector3i p = -d;
    dist[dirIdx(p)] = 0.0;
    for (int it2 = 0; it2 < 27; ++it2)
    {
      int u = -1;
      for (int k = 0; k < 27; ++k)
        if (!done[k] && k != kNoDir && dist[k] < 1e17 && (u < 0 || dist[k] < dist[u]))
          u = k;
      if (u < 0)
        break;
      done[u] = true;
      const Eigen::Vector3i a = dirVec(u);
      for (int mk = 0; mk < 27; ++mk)
      {
        if (mk == kNoDir)
          continue;
        const Eigen::Vector3i m = dirVec(mk);
        const Eigen::Vector3i b = a + m;
        if (b.cwiseAbs().maxCoeff() > 1 || b == Eigen::Vector3i::Zero() || !freeOff(b) || !legalOff(a, m))
          continue;
        const double nd = dist[u] + m.cast<double>().norm();
        if (nd < dist[dirIdx(b)])
          dist[dirIdx(b)] = nd;
      }
    }

    uint32_t mask = 0;
    const double len_pd = d.cast<double>().norm();
    for (int k = 0; k < 27; ++k)
    {
      if (k == kNoDir)
        continue;
      const Eigen::Vector3i dp = dirVec(k);
      if (isSubDir(dp, d) || !freeOff(dp) || !legalOff(Eigen::Vector3i::Zero(), dp))
        continue;
      // eq. (7): pruned iff an alternative p -> n avoiding x is not longer than p -> x -> n.
      // Tie rule for diagonal arrivals: literal (<=, as written in eq. 7) or strict (<, Harabor & Grastien [2]);
      // see tie_rule_ in the header
      const double via = len_pd + dp.cast<double>().norm();
      const bool diag = (d.x() != 0) + (d.y() != 0) + (d.z() != 0) >= 2;
      const bool pruned = (diag && tie_rule_ == TieRule::Strict) ? dist[k] < via - 1e-9 : dist[k] <= via + 1e-9;
      if (!pruned)
        mask |= 1u << k;
    }
    forced_cache_.emplace(key, mask);
    return mask;
  }

  bool Jps3D::jump(const Eigen::Vector3i &x0, const Eigen::Vector3i &d, const Eigen::Vector3i &goal, Eigen::Vector3i &out) const
  {
    // eq. (8), iterative along d with the recursive lower-dimensional scans for diagonals; memoised per (cell, d)
    if (goal != memo_goal_)
    {
      jump_memo_.clear();
      memo_goal_ = goal;
    }
    const Eigen::Vector3i dims = grid_->dims();
    auto key = [&](const Eigen::Vector3i &c) {
      return ((static_cast<uint64_t>(c.z()) * dims.y() + c.y()) * dims.x() + c.x()) * 27 + dirIdx(d);
    };
    auto cellOf = [&](int64_t f) {
      return Eigen::Vector3i(static_cast<int>(f % dims.x()), static_cast<int>((f / dims.x()) % dims.y()),
                             static_cast<int>(f / (static_cast<int64_t>(dims.x()) * dims.y())));
    };
    auto flat = [&](const Eigen::Vector3i &c) { return (static_cast<int64_t>(c.z()) * dims.y() + c.y()) * dims.x() + c.x(); };

    std::vector<uint64_t> visited;
    int64_t result = -1;
    Eigen::Vector3i x = x0;
    const int support = (d.x() != 0) + (d.y() != 0) + (d.z() != 0);
    while (true)
    {
      if (inside(x))
      {
        auto it = jump_memo_.find(key(x));
        if (it != jump_memo_.end())
        {
          result = it->second;
          break;
        }
        visited.push_back(key(x));
      }
      if (!legal(x, d))
        break;
      const Eigen::Vector3i n = x + d;
      if (n == goal || forcedMask(n, d) != 0)
      {
        result = flat(n);
        break;
      }
      bool stop = false;
      if (support >= 2)
      {
        for (int k = 0; k < 27 && !stop; ++k)
        {
          const Eigen::Vector3i dp = dirVec(k);
          if (dp == d || !isSubDir(dp, d))
            continue;
          Eigen::Vector3i tmp;
          stop = jump(n, dp, goal, tmp);
        }
      }
      if (stop)
      {
        result = flat(n);
        break;
      }
      x = n;
    }
    for (uint64_t k : visited)
      jump_memo_[k] = result;
    if (result < 0)
      return false;
    out = cellOf(result);
    return true;
  }

  void Jps3D::successors(const Eigen::Vector3i &x, int dir_in, const Eigen::Vector3i &goal,
                         std::vector<std::pair<Eigen::Vector3i, int>> &out) const
  {
    out.clear();
    uint32_t dirs = 0;
    if (dir_in == kNoDir)
      dirs = ((1u << 27) - 1) & ~(1u << kNoDir); // start: all 26 directions
    else
    {
      const Eigen::Vector3i d = dirVec(dir_in);
      for (int k = 0; k < 27; ++k)
        if (isSubDir(dirVec(k), d))
          dirs |= 1u << k; // natural neighbours (6)
      dirs |= forcedMask(x, d);
    }
    for (int k = 0; k < 27; ++k)
    {
      if (!((dirs >> k) & 1u))
        continue;
      Eigen::Vector3i j;
      if (jump(x, dirVec(k), goal, j))
        out.emplace_back(j, k);
    }
  }

  Jps3D::Result Jps3D::search(const Eigen::Vector3i &start, const Eigen::Vector3i &goal, const EdgeWeight &w,
                              double h_scale, bool dir_dependent) const
  {
    Result res;
    if (!freeCell(start) || !freeCell(goal))
      return res;
    const Eigen::Vector3i dims = grid_->dims();
    auto cellKey = [&](const Eigen::Vector3i &n) {
      return (static_cast<uint64_t>(n.z()) * dims.y() + n.y()) * dims.x() + n.x();
    };
    auto stateKey = [&](const Eigen::Vector3i &n, int dir) { return cellKey(n) * 27 + dir; };
    // closure / best-g bookkeeping key: the state itself, or only its cell for direction-independent weights
    auto bookKey = [&](uint64_t state) { return dir_dependent ? state : (state / 27) * 27; };
    auto cellOf = [&](uint64_t ck) {
      return Eigen::Vector3i(static_cast<int>(ck % dims.x()), static_cast<int>((ck / dims.x()) % dims.y()),
                             static_cast<int>(ck / (static_cast<uint64_t>(dims.x()) * dims.y())));
    };

    struct Node
    {
      double g;
      uint64_t parent;
      bool closed;
    };
    std::unordered_map<uint64_t, Node> nodes; // keyed by bookKey
    nodes.reserve(1 << 18);
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;

    const uint64_t s0 = stateKey(start, kNoDir);
    nodes[bookKey(s0)] = Node{0.0, s0, false};
    open.push({h_scale * hD(start, goal), 0.0, s0});
    std::vector<std::pair<Eigen::Vector3i, int>> succ;
    while (!open.empty())
    {
      const QItem top = open.top();
      open.pop();
      Node &cur = nodes[bookKey(top.key)];
      if (cur.closed || top.g > cur.g + 1e-12)
        continue;
      cur.closed = true;
      const double g_cur = cur.g;
      const Eigen::Vector3i x = cellOf(top.key / 27);
      const int din = static_cast<int>(top.key % 27);
      ++res.expansions;

      if (x == goal)
      {
        res.ok = true;
        res.cost = g_cur;
        std::vector<uint64_t> chain{top.key};
        while (chain.back() != s0)
          chain.push_back(nodes[bookKey(chain.back())].parent);
        std::reverse(chain.begin(), chain.end());
        for (size_t i = 0; i < chain.size(); ++i)
        {
          res.cells.push_back(cellOf(chain[i] / 27));
          if (i > 0)
            res.dirs.push_back(static_cast<int>(chain[i] % 27));
        }
        return res;
      }

      successors(x, din, goal, succ);
      for (const auto &sj : succ)
      {
        const uint64_t sk = stateKey(sj.first, sj.second);
        auto it = nodes.find(bookKey(sk));
        if (it != nodes.end() && it->second.closed)
          continue;
        const double ng = g_cur + w(x, sj.first, din, sj.second); // eq. (9) with weights w_e
        if (it == nodes.end())
          nodes.emplace(bookKey(sk), Node{ng, top.key, false});
        else if (ng < it->second.g - 1e-12)
          it->second = Node{ng, top.key, false};
        else
          continue;
        open.push({ng + h_scale * hD(sj.first, goal), ng, sk});
      }
    }
    return res;
  }

  Jps3D::Result Jps3D::searchGeometric(const Eigen::Vector3i &start, const Eigen::Vector3i &goal) const
  {
    const double res = cellSize();
    return search(
        start, goal,
        [res](const Eigen::Vector3i &a, const Eigen::Vector3i &b, int, int) { return (b - a).cast<double>().norm() * res; },
        1.0, false);
  }

  Jps3D::Result Jps3D::astar26(const Eigen::Vector3i &start, const Eigen::Vector3i &goal) const
  {
    Result res;
    if (!freeCell(start) || !freeCell(goal))
      return res;
    const Eigen::Vector3i dims = grid_->dims();
    auto key = [&](const Eigen::Vector3i &n) { return (static_cast<uint64_t>(n.z()) * dims.y() + n.y()) * dims.x() + n.x(); };
    auto cellOf = [&](uint64_t ck) {
      return Eigen::Vector3i(static_cast<int>(ck % dims.x()), static_cast<int>((ck / dims.x()) % dims.y()),
                             static_cast<int>(ck / (static_cast<uint64_t>(dims.x()) * dims.y())));
    };
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, uint64_t> parent;
    std::unordered_map<uint64_t, bool> closed;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
    g[key(start)] = 0.0;
    open.push({hD(start, goal), 0.0, key(start)});
    while (!open.empty())
    {
      const QItem top = open.top();
      open.pop();
      if (closed[top.key] || top.g > g[top.key] + 1e-12)
        continue;
      closed[top.key] = true;
      ++res.expansions;
      const Eigen::Vector3i x = cellOf(top.key);
      if (x == goal)
      {
        res.ok = true;
        res.cost = g[top.key];
        std::vector<uint64_t> chain{top.key};
        while (chain.back() != key(start))
          chain.push_back(parent[chain.back()]);
        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
          res.cells.push_back(cellOf(*it));
        return res;
      }
      for (int k = 0; k < 27; ++k)
      {
        if (k == kNoDir)
          continue;
        const Eigen::Vector3i d = dirVec(k);
        if (!legal(x, d))
          continue;
        const Eigen::Vector3i n = x + d;
        const uint64_t nk = key(n);
        if (closed.count(nk) && closed[nk])
          continue;
        const double ng = g[top.key] + d.cast<double>().norm() * cellSize(); // eq. (2)
        auto it = g.find(nk);
        if (it == g.end() || ng < it->second - 1e-12)
        {
          g[nk] = ng;
          parent[nk] = top.key;
          open.push({ng + hD(n, goal), ng, nk});
        }
      }
    }
    return res;
  }

} // namespace jps_vo
