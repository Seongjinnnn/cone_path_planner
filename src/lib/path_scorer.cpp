#include "cone_path_planner/path_scorer.hpp"
#include <cmath>

namespace cone_path_planner {

static inline double angle_difference(double a1, double a2) {
  double diff = std::fmod((a1 - a2) + 3.14159265358979323846, 2.0 * 3.14159265358979323846);
  if (diff < 0) diff += 2.0 * 3.14159265358979323846;
  return diff - 3.14159265358979323846;
}

PathScorer::PathScorer(double angle_weight, double length_weight)
  : angle_weight_(angle_weight), length_weight_(length_weight) {}

std::vector<double> PathScorer::score_configurations(
    const std::vector<std::vector<int>>& configs,
    const std::vector<geometry_msgs::msg::Point>& cones)
{
  std::vector<double> scores;
  scores.reserve(configs.size());

  constexpr double eps = 1e-6;

  for (const auto& cfg : configs) {
    if (cfg.size() < 2) { scores.push_back(1e9); continue; }

    const double number_cost = 1.0 / static_cast<double>(cfg.size());

    double total_angle = 0.0;
    int angle_cnt = 0;
    if (cfg.size() >= 3) {
      for (size_t i = 0; i + 2 < cfg.size(); ++i) {
        const int p1 = cfg[i], p2 = cfg[i+1], p3 = cfg[i+2];
        const double v1x = cones[p2].x - cones[p1].x;
        const double v1y = cones[p2].y - cones[p1].y;
        const double v2x = cones[p3].x - cones[p2].x;
        const double v2y = cones[p3].y - cones[p2].y;
        const double l1 = std::hypot(v1x, v1y);
        const double l2 = std::hypot(v2x, v2y);
        if (l1 < eps || l2 < eps) continue;
        const double a1 = std::atan2(v1y, v1x);
        const double a2 = std::atan2(v2y, v2x);
        total_angle += std::abs(angle_difference(a2, a1));
        angle_cnt++;
      }
    }
    const double mean_angle = (angle_cnt > 0) ? (total_angle / angle_cnt) : 0.0;
    const double score = angle_weight_ * mean_angle + length_weight_ * number_cost;
    scores.push_back(score);
  }

  return scores;
}

}  // namespace cone_path_planner
