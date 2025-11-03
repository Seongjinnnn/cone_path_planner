#include "cone_path_planner/cone_sorter.hpp"
#include <cmath>

namespace cone_path_planner {

ConeSorter::ConeSorter(double max_dist_to_first)
  : max_dist_to_first_(max_dist_to_first) {}

std::pair<int, int> ConeSorter::sort_cones(
    const std::vector<geometry_msgs::msg::Point>& cones,
    const geometry_msgs::msg::Point& vehicle_pos,
    double vehicle_yaw)
{
  double best_left_dist_sq  = max_dist_to_first_ * max_dist_to_first_;
  double best_right_dist_sq = max_dist_to_first_ * max_dist_to_first_;
  int best_left_idx = -1, best_right_idx = -1;

  const double dir_x = std::cos(vehicle_yaw);
  const double dir_y = std::sin(vehicle_yaw);

  for (size_t i = 0; i < cones.size(); ++i) {
    const double rel_x = cones[i].x - vehicle_pos.x;
    const double rel_y = cones[i].y - vehicle_pos.y;

    const double dot = rel_x * dir_x + rel_y * dir_y;
    if (dot < 0.0) continue;

    const double dist_sq = rel_x*rel_x + rel_y*rel_y;
    if (dist_sq > max_dist_to_first_ * max_dist_to_first_) continue;

    const double cross_z = rel_x * dir_y - rel_y * dir_x;
    if (cross_z < 0) {
      if (dist_sq < best_left_dist_sq) { best_left_dist_sq = dist_sq; best_left_idx = static_cast<int>(i); }
    } else {
      if (dist_sq < best_right_dist_sq){ best_right_dist_sq = dist_sq; best_right_idx = static_cast<int>(i); }
    }
  }

  return {best_left_idx, best_right_idx};
}

}  // namespace cone_path_planner
