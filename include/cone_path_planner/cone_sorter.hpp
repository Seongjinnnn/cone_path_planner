#ifndef CONE_SORTER_HPP_
#define CONE_SORTER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace cone_path_planner {

class ConeSorter {
public:
  ConeSorter(double max_dist_to_first);

  std::pair<int, int> sort_cones(
    const std::vector<geometry_msgs::msg::Point>& cones,
    const geometry_msgs::msg::Point& vehicle_pos,
    double vehicle_yaw);

private:
  double max_dist_to_first_;
};

}  // namespace cone_path_planner

#endif  // CONE_SORTER_HPP_
