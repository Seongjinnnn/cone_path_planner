#include "cone_path_planner/centerline_calculator.hpp"
#include <limits>

namespace cone_path_planner {

std::vector<geometry_msgs::msg::Point> CenterlineCalculator::calculate_centerline(
    const std::vector<geometry_msgs::msg::Point>& left_path,
    const std::vector<geometry_msgs::msg::Point>& right_path)
{
  if (left_path.empty() || right_path.empty()) return {};

  std::vector<geometry_msgs::msg::Point> centerline;
  centerline.reserve(std::min(left_path.size(), right_path.size()));

  const auto& shorter = (left_path.size() < right_path.size()) ? left_path : right_path;
  const auto& longer  = (left_path.size() < right_path.size()) ? right_path : left_path;

  for (const auto& p1 : shorter) {
    double best = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Point p2_best;
    for (const auto& p2 : longer) {
      const double dx = p1.x - p2.x, dy = p1.y - p2.y;
      const double dsq = dx*dx + dy*dy;
      if (dsq < best) { best = dsq; p2_best = p2; }
    }
    geometry_msgs::msg::Point mid;
    mid.x = 0.5 * (p1.x + p2_best.x);
    mid.y = 0.5 * (p1.y + p2_best.y);
    centerline.push_back(mid);
  }

  return centerline;
}

}  // namespace cone_path_planner
