#include "cone_path_planner/path_finder.hpp"
#include <stack>
#include <cmath>
#include <algorithm>

namespace cone_path_planner {

static inline double angle_difference(double a1, double a2) {
  double diff = std::fmod((a1 - a2) + 3.14159265358979323846, 2.0 * 3.14159265358979323846);
  if (diff < 0) diff += 2.0 * 3.14159265358979323846;
  return diff - 3.14159265358979323846;
}

PathFinder::PathFinder(int kMaxLen, double thr_abs, double thr_dir)
  : kMaxLen_(kMaxLen), thr_abs_(thr_abs), thr_dir_(thr_dir) {}

std::vector<std::vector<int>> PathFinder::find_configurations(
    int start_node,
    const std::vector<std::vector<bool>>& adj_matrix,
    const std::vector<geometry_msgs::msg::Point>& cones,
    const std::string& cone_type,
    const std::unordered_set<int>& used_cones)
{
  std::vector<std::vector<int>> end_configurations;
  constexpr double eps = 1e-6;

  std::stack<std::pair<int,int>> st;
  st.push({start_node, 0});
  std::vector<int> current_path(kMaxLen_, -1);

  while (!st.empty()) {
    auto [cur, depth] = st.top(); st.pop();

    if (depth >= kMaxLen_) {
      continue;
    }
    current_path[depth] = cur;
    for (int i = depth+1; i < kMaxLen_; ++i) current_path[i] = -1;

    std::vector<int> neighbors;
    for (size_t i = 0; i < adj_matrix[cur].size(); ++i) {
      if (!adj_matrix[cur][i]) continue;

      if (used_cones.count(static_cast<int>(i))) continue;

      bool in_path = false;
      for (int j = 0; j <= depth; ++j) if (current_path[j] == static_cast<int>(i)) { in_path = true; break; }
      if (in_path) continue;

      bool ok = true;
      if (depth >= 1) {
        const int last  = current_path[depth];
        const int prev  = current_path[depth-1];

        const double v1x = cones[last].x - cones[prev].x;
        const double v1y = cones[last].y - cones[prev].y;
        const double v2x = cones[i].x    - cones[last].x;
        const double v2y = cones[i].y    - cones[last].y;

        const double l1 = std::hypot(v1x, v1y);
        const double l2 = std::hypot(v2x, v2y);
        if (l1 < eps || l2 < eps) ok = false;
        else {
          const double a1 = std::atan2(v1y, v1x);
          const double a2 = std::atan2(v2y, v2x);
          const double d_abs = std::abs(angle_difference(a2, a1));
          const double d_sgn = angle_difference(a2, a1);

          if (d_abs > thr_abs_) ok = false;
          if (ok) {
            if (cone_type == "left"  && d_sgn < -thr_dir_) ok = false;
            if (cone_type == "right" && d_sgn >  thr_dir_) ok = false;
          }
        }
      }

      if (ok) neighbors.push_back(static_cast<int>(i));
    }

    if (neighbors.empty() || depth + 1 >= kMaxLen_) {
      std::vector<int> final_path;
      final_path.reserve(depth+1);
      for (int i = 0; i <= depth; ++i) final_path.push_back(current_path[i]);
      end_configurations.push_back(std::move(final_path));
    } else {
      for (int nb : neighbors) st.push({nb, depth+1});
    }
  }

  return end_configurations;
}

}  // namespace cone_path_planner
