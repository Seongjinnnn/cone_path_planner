#ifndef CONE_MATCHER_HPP_
#define CONE_MATCHER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <vector>

namespace cone_path_planner {

class ConeMatcher {
public:
  ConeMatcher(double major_r, double minor_r, double min_track_w);

  void run_cone_matching(
      const std::vector<int>& left_path,
      const std::vector<int>& right_path,
      const std::vector<geometry_msgs::msg::Point>& cones,
      std::vector<geometry_msgs::msg::Point>& final_left,
      std::vector<geometry_msgs::msg::Point>& final_right);

private:
  bool do_paths_cross(
      const std::vector<int>& left_path,
      const std::vector<int>& right_path,
      const std::vector<geometry_msgs::msg::Point>& cones);

  int get_orientation(
      const geometry_msgs::msg::Point& p,
      const geometry_msgs::msg::Point& q,
      const geometry_msgs::msg::Point& r);

  bool on_segment(
      const geometry_msgs::msg::Point& p,
      const geometry_msgs::msg::Point& q,
      const geometry_msgs::msg::Point& r);

  bool do_segments_intersect(
      const geometry_msgs::msg::Point& p1,
      const geometry_msgs::msg::Point& q1,
      const geometry_msgs::msg::Point& p2,
      const geometry_msgs::msg::Point& q2);

  std::vector<geometry_msgs::msg::Point> merge_virtual_cones(
      const std::vector<int>& real_path_indices,
      const std::vector<geometry_msgs::msg::Point>& virtual_cones,
      const std::vector<geometry_msgs::msg::Point>& all_cones);

  double major_r_;
  double minor_r_;
  double min_track_w_;
};

}  // namespace cone_path_planner

#endif  // CONE_MATCHER_HPP_
