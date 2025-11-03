#ifndef PATH_SCORER_HPP_
#define PATH_SCORER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>

namespace cone_path_planner {

class PathScorer {
public:
  PathScorer(double angle_weight, double length_weight);

  std::vector<double> score_configurations(
      const std::vector<std::vector<int>>& configs,
      const std::vector<geometry_msgs::msg::Point>& cones);

private:
  double angle_weight_;
  double length_weight_;
};

}  // namespace cone_path_planner

#endif  // PATH_SCORER_HPP_
