// cone_sorter_node.cpp
#include "cone_path_planner/path_planner_node.hpp"

#include "cone_path_planner/cone_sorter.hpp"
#include "cone_path_planner/path_finder.hpp"
#include "cone_path_planner/path_scorer.hpp"
#include "cone_path_planner/cone_matcher.hpp"
#include "cone_path_planner/centerline_calculator.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <algorithm>
#include <utility>
#include <stack>
#include <limits>
#include <sstream>
#include <unordered_set>

using namespace cone_path_planner;

constexpr double PI = 3.14159265358979323846;

static inline double angle_difference(double a1, double a2) {
  double diff = std::fmod((a1 - a2) + PI, 2.0 * PI);
  if (diff < 0) diff += 2.0 * PI;
  return diff - PI;
}

static inline builtin_interfaces::msg::Duration seconds(double s) {
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(std::floor(s));
  d.nanosec = static_cast<uint32_t>((s - d.sec) * 1e9);
  return d;
}

static inline geometry_msgs::msg::Point transformPoint(
    const geometry_msgs::msg::Point& p,
    const geometry_msgs::msg::TransformStamped& T)
{
  tf2::Quaternion q(
    T.transform.rotation.x,
    T.transform.rotation.y,
    T.transform.rotation.z,
    T.transform.rotation.w);
  tf2::Matrix3x3 R(q);

  geometry_msgs::msg::Point out;
  out.x = R[0][0]*p.x + R[0][1]*p.y + R[0][2]*p.z + T.transform.translation.x;
  out.y = R[1][0]*p.x + R[1][1]*p.y + R[1][2]*p.z + T.transform.translation.y;
  out.z = R[2][0]*p.x + R[2][1]*p.y + R[2][2]*p.z + T.transform.translation.z;
  return out;
}

PathPlannerNode::PathPlannerNode()
: Node("path_planner_node")
{
  this->declare_parameter<std::string>("target_frame", "odom");
  this->declare_parameter<std::string>("vehicle_frame", "mld_base_link");
  this->declare_parameter<double>("max_dist_to_first", 5.0);
  this->declare_parameter<int>("max_n_neighbors", 5);
  this->declare_parameter<double>("max_dist", 7.0);
  this->declare_parameter<int>("kMaxLen", 12);
  this->declare_parameter<double>("thr_abs", 1.22);
  this->declare_parameter<double>("thr_dir", 0.698);
  this->declare_parameter<double>("angle_weight", 1000.0);
  this->declare_parameter<double>("length_weight", 5000.0);
  this->declare_parameter<double>("major_r", 8.0);
  this->declare_parameter<double>("minor_r", 4.0);
  this->declare_parameter<double>("min_track_w", 3.0);

  this->get_parameter("target_frame", target_frame_);
  this->get_parameter("max_dist_to_first", max_dist_to_first_);
  this->get_parameter("max_n_neighbors", max_n_neighbors_);
  this->get_parameter("max_dist", max_dist_);
  this->get_parameter("kMaxLen", kMaxLen_);
  this->get_parameter("thr_abs", thr_abs_);
  this->get_parameter("thr_dir", thr_dir_);
  this->get_parameter("angle_weight", angle_weight_);
  this->get_parameter("length_weight", length_weight_);
  this->get_parameter("major_r", major_r_);
  this->get_parameter("minor_r", minor_r_);
  this->get_parameter("min_track_w", min_track_w_);

  subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/collected_cones", rclcpp::QoS(10),
    std::bind(&PathPlannerNode::topic_callback, this, std::placeholders::_1));

      vis_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/debug/sorted_cones_vis", 10);
      path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
      path_msg_.header.frame_id = target_frame_;
  
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Path Planner Node started. Target frame: %s", target_frame_.c_str());

  cone_sorter_ = std::make_unique<cone_path_planner::ConeSorter>(max_dist_to_first_);
  path_finder_ = std::make_unique<cone_path_planner::PathFinder>(kMaxLen_, thr_abs_, thr_dir_);
  path_scorer_ = std::make_unique<cone_path_planner::PathScorer>(angle_weight_, length_weight_);
  cone_matcher_ = std::make_unique<cone_path_planner::ConeMatcher>(major_r_, minor_r_, min_track_w_);
  centerline_calculator_ = std::make_unique<cone_path_planner::CenterlineCalculator>();
}

void PathPlannerNode::topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped T_src_to_target;
  try {
    T_src_to_target = tf_buffer_->lookupTransform(
      target_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF %s -> %s failed: %s",
                msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    return;
  }

  std::vector<geometry_msgs::msg::Point> transformed_cones;
  transformed_cones.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    transformed_cones.push_back(transformPoint(pose.position, T_src_to_target));
  }
  RCLCPP_INFO(this->get_logger(), "Transformed %zu cones -> '%s'.",
              transformed_cones.size(), target_frame_.c_str());

  std::string vehicle_frame;
  this->get_parameter("vehicle_frame", vehicle_frame);

  geometry_msgs::msg::TransformStamped T_base;
  try {
    T_base = tf_buffer_->lookupTransform(
      target_frame_, vehicle_frame, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF %s -> %s failed: %s",
                vehicle_frame.c_str(), target_frame_.c_str(), ex.what());
    return;
  }

  geometry_msgs::msg::Point vehicle_pos;
  vehicle_pos.x = T_base.transform.translation.x;
  vehicle_pos.y = T_base.transform.translation.y;

  tf2::Quaternion q(
    T_base.transform.rotation.x,
    T_base.transform.rotation.y,
    T_base.transform.rotation.z,
    T_base.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  RCLCPP_INFO(this->get_logger(), "Vehicle: x=%.2f, y=%.2f, yaw=%.2f",
              vehicle_pos.x, vehicle_pos.y, yaw);

  auto [best_left_idx, best_right_idx] = cone_sorter_->sort_cones(transformed_cones, vehicle_pos, yaw);

  RCLCPP_INFO(this->get_logger(), "Seeds -> left:%d right:%d", best_left_idx, best_right_idx);
  if (transformed_cones.size() < 2) return;

  std::vector<std::vector<double>> dist_matrix(transformed_cones.size(), std::vector<double>(transformed_cones.size(), 0.0));
  for (size_t i = 0; i < transformed_cones.size(); ++i) {
    for (size_t j = i; j < transformed_cones.size(); ++j) {
      const double dx = transformed_cones[i].x - transformed_cones[j].x;
      const double dy = transformed_cones[i].y - transformed_cones[j].y;
      const double d  = std::sqrt(dx*dx + dy*dy);
      dist_matrix[i][j] = d;
      dist_matrix[j][i] = d;
    }
  }

  std::vector<std::vector<int>> k_nearest_indices(transformed_cones.size());
  for (size_t i = 0; i < transformed_cones.size(); ++i) {
    std::vector<std::pair<double,int>> v;
    v.reserve(transformed_cones.size()-1);
    for (size_t j = 0; j < transformed_cones.size(); ++j) {
      if (i == j) continue;
      v.push_back({dist_matrix[i][j], static_cast<int>(j)});
    }
    std::sort(v.begin(), v.end(), [](auto& a, auto& b){ return a.first < b.first; });
    for (int k = 0; k < max_n_neighbors_ && k < static_cast<int>(v.size()); ++k) {
      k_nearest_indices[i].push_back(v[k].second);
    }
  }

  std::vector<std::vector<bool>> adj_matrix(transformed_cones.size(), std::vector<bool>(transformed_cones.size(), false));
  for (size_t i = 0; i < transformed_cones.size(); ++i) {
    for (int nb : k_nearest_indices[i]) {
      if (dist_matrix[i][nb] < max_dist_) adj_matrix[i][nb] = true;
    }
  }
  for (size_t i = 0; i < transformed_cones.size(); ++i) {
    for (size_t j = i; j < transformed_cones.size(); ++j) {
      if (adj_matrix[i][j] || adj_matrix[j][i]) {
        adj_matrix[i][j] = adj_matrix[j][i] = true;
      }
    }
  }

  if (best_left_idx != -1 && best_right_idx != -1) {
    adj_matrix[best_left_idx][best_right_idx] = false;
    adj_matrix[best_right_idx][best_left_idx] = false;
  }

  std::vector<int> best_left_path, best_right_path;
  std::unordered_set<int> used_cones;

  if (best_left_idx != -1) {
    auto left_cfgs = path_finder_->find_configurations(best_left_idx, adj_matrix, transformed_cones, "left", used_cones);
    if (!left_cfgs.empty()) {
      auto scores = path_scorer_->score_configurations(left_cfgs, transformed_cones);
      int pick = static_cast<int>(std::distance(scores.begin(), std::min_element(scores.begin(), scores.end())));
      best_left_path = left_cfgs[pick];
    }
  }

  for (int idx : best_left_path) {
    used_cones.insert(idx);
  }

  if (best_right_idx != -1) {
    if (used_cones.count(best_right_idx) == 0) {
      auto right_cfgs = path_finder_->find_configurations(best_right_idx, adj_matrix, transformed_cones, "right", used_cones);
      if (!right_cfgs.empty()) {
        auto scores = path_scorer_->score_configurations(right_cfgs, transformed_cones);
        int pick = static_cast<int>(std::distance(scores.begin(), std::min_element(scores.begin(), scores.end())));
        best_right_path = right_cfgs[pick];
      }
    }
  }

  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = target_frame_;
    del.header.stamp    = this->get_clock()->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    for (size_t i = 0; i < transformed_cones.size(); ++i) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = target_frame_;
      mk.header.stamp    = this->get_clock()->now();
      mk.ns = "cones";
      mk.id = static_cast<int>(i);
      mk.type = visualization_msgs::msg::Marker::CUBE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position = transformed_cones[i];
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.3;
      mk.color.a = 1.0;
      mk.lifetime = seconds(0.2);

      const bool in_left  = std::find(best_left_path.begin(),  best_left_path.end(),  static_cast<int>(i))  != best_left_path.end();
      const bool in_right = std::find(best_right_path.begin(), best_right_path.end(), static_cast<int>(i)) != best_right_path.end();

      if (in_left)      { mk.color.r = 0.0; mk.color.g = 0.0; mk.color.b = 1.0; }
      else if (in_right){ mk.color.r = 1.0; mk.color.g = 1.0; mk.color.b = 0.0; }
      else              { mk.color.r = 0.5; mk.color.g = 0.5; mk.color.b = 0.5; }
      arr.markers.push_back(mk);
    }

    visualization_msgs::msg::Marker strip;
    strip.header.frame_id = target_frame_;
    strip.header.stamp    = this->get_clock()->now();
    strip.ns = "path_lines";
    strip.id = static_cast<int>(transformed_cones.size());
    strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    strip.action = visualization_msgs::msg::Marker::ADD;
    strip.pose.orientation.w = 1.0;
    strip.scale.x = 0.1;
    strip.color.a = 1.0;
    strip.color.r = 0.0; strip.color.g = 0.0; strip.color.b = 1.0;
    strip.lifetime = seconds(0.2);
    for (int idx : best_left_path) strip.points.push_back(transformed_cones[idx]);
    arr.markers.push_back(strip);

    visualization_msgs::msg::Marker strip_r = strip;
    strip_r.id = static_cast<int>(transformed_cones.size()) + 1;
    strip_r.color.r = 1.0; strip_r.color.g = 1.0; strip_r.color.b = 0.0;
    strip_r.points.clear();
    for (int idx : best_right_path) strip_r.points.push_back(transformed_cones[idx]);
    arr.markers.push_back(strip_r);

    vis_publisher_->publish(arr);
  }

  std::vector<geometry_msgs::msg::Point> final_left, final_right;
  cone_matcher_->run_cone_matching(best_left_path, best_right_path, transformed_cones, final_left, final_right);

  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = target_frame_;
    del.header.stamp    = this->get_clock()->now();
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    int id = 0;
    for (const auto& p : final_left) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = target_frame_;
      mk.header.stamp    = this->get_clock()->now();
      mk.ns = "final_path_cones";
      mk.id = id++;
      mk.type = visualization_msgs::msg::Marker::CUBE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position = p;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.3;
      mk.color.a = 1.0;
      mk.color.r = 0.0; mk.color.g = 0.0; mk.color.b = 1.0;
      arr.markers.push_back(mk);
    }
    for (const auto& p : final_right) {
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = target_frame_;
      mk.header.stamp    = this->get_clock()->now();
      mk.ns = "final_path_cones";
      mk.id = id++;
      mk.type = visualization_msgs::msg::Marker::CUBE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position = p;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.3;
      mk.color.a = 1.0;
      mk.color.r = 1.0; mk.color.g = 1.0; mk.color.b = 0.0;
      arr.markers.push_back(mk);
    }

    visualization_msgs::msg::Marker left_strip;
    left_strip.header.frame_id = target_frame_;
    left_strip.header.stamp    = this->get_clock()->now();
    left_strip.ns = "final_path_lines";
    left_strip.id = id++;
    left_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    left_strip.action = visualization_msgs::msg::Marker::ADD;
    left_strip.pose.orientation.w = 1.0;
    left_strip.scale.x = 0.15;
    left_strip.color.a = 1.0;
    left_strip.color.r = 0.0; left_strip.color.g = 0.0; left_strip.color.b = 1.0;
    left_strip.points = final_left;
    arr.markers.push_back(left_strip);

    visualization_msgs::msg::Marker right_strip = left_strip;
    right_strip.id = id++;
    right_strip.color.r = 1.0; right_strip.color.g = 1.0; right_strip.color.b = 0.0;
    right_strip.points = final_right;
    arr.markers.push_back(right_strip);

    vis_publisher_->publish(arr);
  }

  auto centerline = centerline_calculator_->calculate_centerline(final_left, final_right);

  // Publish path for path_follower
  path_msg_.poses.clear();
  path_msg_.header.stamp = this->get_clock()->now();
  path_msg_.header.frame_id = target_frame_;

  tf2::Quaternion q_path;
  for (size_t i = 0; i < centerline.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = path_msg_.header.stamp;
    ps.header.frame_id = path_msg_.header.frame_id;
    ps.pose.position = centerline[i];

    double yaw = 0.0;
    if (i < centerline.size() - 1) {
      // Calculate yaw from current and next point
      double dx = centerline[i+1].x - centerline[i].x;
      double dy = centerline[i+1].y - centerline[i].y;
      yaw = std::atan2(dy, dx);
    } else if (centerline.size() > 1) {
      // For the last point, use yaw from previous segment
      double dx = centerline[i].x - centerline[i-1].x;
      double dy = centerline[i].y - centerline[i-1].y;
      yaw = std::atan2(dy, dx);
    }
    // If only one point, yaw remains 0.0 (or could use vehicle yaw if available)

    q_path.setRPY(0, 0, yaw);
    ps.pose.orientation.x = q_path.x();
    ps.pose.orientation.y = q_path.y();
    ps.pose.orientation.z = q_path.z();
    ps.pose.orientation.w = q_path.w();

    path_msg_.poses.push_back(ps);
  }
  path_publisher_->publish(path_msg_);

  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker del;
  del.header.frame_id = target_frame_;
  del.header.stamp    = this->get_clock()->now();
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  visualization_msgs::msg::Marker line;
  line.header.frame_id = target_frame_;
  line.header.stamp    = this->get_clock()->now();
  line.ns = "centerline";
  line.id = 4000;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.2;
  line.color.a = 1.0;
  line.color.r = 0.0;
  line.color.g = 1.0;
  line.color.b = 0.0;
  line.points = centerline;

  arr.markers.push_back(line);
  vis_publisher_->publish(arr);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
