#ifndef CENTERLINE_CALCULATOR_HPP_
#define CENTERLINE_CALCULATOR_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace cone_path_planner {

class CenterlineCalculator {
public:
  CenterlineCalculator() = default;

  std::vector<geometry_msgs::msg::Point> calculate_centerline(
      const std::vector<geometry_msgs::msg::Point>& left_path,
      const std::vector<geometry_msgs::msg::Point>& right_path);
};

}  // namespace cone_path_planner

#endif  // CENTERLINE_CALCULATOR_HPP_
