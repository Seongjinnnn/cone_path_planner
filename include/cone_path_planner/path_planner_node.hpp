#ifndef PATH_PLANNER_NODE_HPP_
#define PATH_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <vector>
#include <memory>
#include <unordered_set>

#include "cone_sorter.hpp"
#include "path_finder.hpp"
#include "path_scorer.hpp"
#include "cone_matcher.hpp"
#include "centerline_calculator.hpp"

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode();

private:
  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);



  // Parameters
  std::string target_frame_;
  double max_dist_to_first_;
  int max_n_neighbors_;
  double max_dist_;
  int kMaxLen_;
  double thr_abs_;
  double thr_dir_;
  double angle_weight_;
  double length_weight_;
  double major_r_;
  double minor_r_;
  double min_track_w_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_publisher_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  nav_msgs::msg::Path path_msg_;

  std::unique_ptr<cone_path_planner::ConeSorter> cone_sorter_;
  std::unique_ptr<cone_path_planner::PathFinder> path_finder_;
  std::unique_ptr<cone_path_planner::PathScorer> path_scorer_;
  std::unique_ptr<cone_path_planner::ConeMatcher> cone_matcher_;
  std::unique_ptr<cone_path_planner::CenterlineCalculator> centerline_calculator_;
};

#endif  // PATH_PLANNER_NODE_HPP_
