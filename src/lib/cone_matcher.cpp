#include "cone_path_planner/cone_matcher.hpp"
#include <cmath>
#include <limits>
#include <algorithm>

namespace cone_path_planner {

ConeMatcher::ConeMatcher(double major_r, double minor_r, double min_track_w)
  : major_r_(major_r), minor_r_(minor_r), min_track_w_(min_track_w) {}

void ConeMatcher::run_cone_matching(
    const std::vector<int>& left_path,
    const std::vector<int>& right_path,
    const std::vector<geometry_msgs::msg::Point>& cones,
    std::vector<geometry_msgs::msg::Point>& final_left,
    std::vector<geometry_msgs::msg::Point>& final_right)
{
  if (do_paths_cross(left_path, right_path, cones)) {
    // RCLCPP_WARN(this->get_logger(), "Left and right paths cross. Discarding path.");
    final_left.clear();
    final_right.clear();
    return;
  }

  if (left_path.size() < 2 && right_path.size() < 2) {
    // RCLCPP_WARN(this->get_logger(), "Not enough cones for matching.");
    return;
  }

  std::vector<geometry_msgs::msg::Point> left_dirs, right_dirs;
  auto make_dir = [](double dx, double dy, bool left_side) {
    geometry_msgs::msg::Point d;
    double nx = left_side ?  dy : -dy;
    double ny = left_side ? -dx :  dx;
    double mag = std::hypot(nx, ny);
    if (mag > 1e-6) { d.x = nx / mag; d.y = ny / mag; } else { d.x = 1.0; d.y = 0.0; }
    return d;
  };

  if (left_path.size() >= 2) {
    left_dirs.reserve(left_path.size());
    for (size_t i = 0; i < left_path.size(); ++i) {
      double dx, dy;
      if      (i == 0)                   { dx = cones[left_path[1]].x - cones[left_path[0]].x; dy = cones[left_path[1]].y - cones[left_path[0]].y; }
      else if (i == left_path.size() -1) { dx = cones[left_path[i]].x - cones[left_path[i-1]].x; dy = cones[left_path[i]].y - cones[left_path[i-1]].y; }
      else                               { dx = cones[left_path[i+1]].x - cones[left_path[i-1]].x; dy = cones[left_path[i+1]].y - cones[left_path[i-1]].y; }
      left_dirs.push_back(make_dir(dx, dy, true));
    }
  }
  if (right_path.size() >= 2) {
    right_dirs.reserve(right_path.size());
    for (size_t i = 0; i < right_path.size(); ++i) {
      double dx, dy;
      if      (i == 0)                    { dx = cones[right_path[1]].x - cones[right_path[0]].x; dy = cones[right_path[1]].y - cones[right_path[0]].y; }
      else if (i == right_path.size() -1) { dx = cones[right_path[i]].x - cones[right_path[i-1]].x; dy = cones[right_path[i]].y - cones[right_path[i-1]].y; }
      else                                { dx = cones[right_path[i+1]].x - cones[right_path[i-1]].x; dy = cones[right_path[i+1]].y - cones[right_path[i-1]].y; }
      right_dirs.push_back(make_dir(dx, dy, false));
    }
  }

  std::vector<int> left_to_right(left_path.size(), -1);
  if (left_path.size() >= 2 && !right_path.empty()) {
    for (size_t i = 0; i < left_path.size(); ++i) {
      double best = std::numeric_limits<double>::infinity();
      int best_j = -1;

      const auto& L = cones[left_path[i]];
      const double sa = std::atan2(left_dirs[i].y, left_dirs[i].x);

      for (size_t j = 0; j < right_path.size(); ++j) {
        const auto& R = cones[right_path[j]];
        const double vx = R.x - L.x, vy = R.y - L.y;
        const double rx =  vx * std::cos(-sa) - vy * std::sin(-sa);
        const double ry =  vx * std::sin(-sa) + vy * std::cos(-sa);
        const double ell = (rx*rx)/(major_r_*major_r_) + (ry*ry)/(minor_r_*minor_r_);
        if (ell < 1.0) {
          const double d2 = vx*vx + vy*vy;
          if (d2 < best) { best = d2; best_j = static_cast<int>(j); }
        }
      }
      left_to_right[i] = best_j;
    }
  }

  std::vector<int> right_to_left(right_path.size(), -1);
  if (right_path.size() >= 2 && !left_path.empty()) {
    for (size_t i = 0; i < right_path.size(); ++i) {
      double best = std::numeric_limits<double>::infinity();
      int best_j = -1;

      const auto& R = cones[right_path[i]];
      const double sa = std::atan2(right_dirs[i].y, right_dirs[i].x);

      for (size_t j = 0; j < left_path.size(); ++j) {
        const auto& L = cones[left_path[j]];
        const double vx = L.x - R.x, vy = L.y - R.y;
        const double rx =  vx * std::cos(-sa) - vy * std::sin(-sa);
        const double ry =  vx * std::sin(-sa) + vy * std::cos(-sa);
        const double ell = (rx*rx)/(major_r_*major_r_) + (ry*ry)/(minor_r_*minor_r_);
        if (ell < 1.0) {
          const double d2 = vx*vx + vy*vy;
          if (d2 < best) { best = d2; best_j = static_cast<int>(j); }
        }
      }
      right_to_left[i] = best_j;
    }
  }

  std::vector<geometry_msgs::msg::Point> left_virtual, right_virtual;

  if (left_path.size() >= 2) {
    for (size_t i = 0; i < left_to_right.size(); ++i) {
      if (left_to_right[i] != -1) continue;
      geometry_msgs::msg::Point v;
      v.x = cones[left_path[i]].x + left_dirs[i].x * min_track_w_;
      v.y = cones[left_path[i]].y + left_dirs[i].y * min_track_w_;
      right_virtual.push_back(v);
    }
  }
  if (right_path.size() >= 2) {
    for (size_t i = 0; i < right_to_left.size(); ++i) {
      if (right_to_left[i] != -1) continue;
      geometry_msgs::msg::Point v;
      v.x = cones[right_path[i]].x + right_dirs[i].x * min_track_w_;
      v.y = cones[right_path[i]].y + right_dirs[i].y * min_track_w_;
      left_virtual.push_back(v);
    }
  }

  final_left  = merge_virtual_cones(left_path,  left_virtual,  cones);
  final_right = merge_virtual_cones(right_path, right_virtual, cones);
}

std::vector<geometry_msgs::msg::Point> ConeMatcher::merge_virtual_cones(
    const std::vector<int>& real_path_indices,
    const std::vector<geometry_msgs::msg::Point>& virtual_cones,
    const std::vector<geometry_msgs::msg::Point>& all_cones)
{
  std::vector<geometry_msgs::msg::Point> merged;
  merged.reserve(real_path_indices.size() + virtual_cones.size());

  for (int idx : real_path_indices) merged.push_back(all_cones[idx]);
  if (virtual_cones.empty()) return merged;
  if (merged.empty()) return virtual_cones;

  for (const auto& v : virtual_cones) {
    if (merged.size() < 2) { merged.push_back(v); continue; }

    double d1 = std::numeric_limits<double>::infinity();
    double d2 = std::numeric_limits<double>::infinity();
    int i1 = -1, i2 = -1;
    for (size_t i = 0; i < merged.size(); ++i) {
      const double dx = v.x - merged[i].x;
      const double dy = v.y - merged[i].y;
      const double dsq = dx*dx + dy*dy;
      if (dsq < d1) { d2 = d1; i2 = i1; d1 = dsq; i1 = static_cast<int>(i); }
      else if (dsq < d2) { d2 = dsq; i2 = static_cast<int>(i); }
    }
    if (i1 == -1 || i2 == -1) continue;
    if (std::abs(i1 - i2) != 1) continue;

    const auto& p1 = merged[i1];
    const auto& p2 = merged[i2];
    const double v1x = p1.x - v.x, v1y = p1.y - v.y;
    const double v2x = p2.x - v.x, v2y = p2.y - v.y;
    const double dot = v1x*v2x + v1y*v2y;

    int insert_idx;
    if (dot < 0) insert_idx = std::max(i1, i2);
    else {
      if (i1 == 0) insert_idx = 0;
      else if (i1 == static_cast<int>(merged.size()) - 1) insert_idx = static_cast<int>(merged.size());
      else insert_idx = i1;
    }
    merged.insert(merged.begin() + insert_idx, v);
  }

  return merged;
}


bool ConeMatcher::do_paths_cross(
    const std::vector<int>& left_path,
    const std::vector<int>& right_path,
    const std::vector<geometry_msgs::msg::Point>& cones)
{
  if (left_path.size() < 2 || right_path.size() < 2) {
    return false;
  }

  for (size_t i = 0; i < left_path.size() - 1; ++i) {
    for (size_t j = 0; j < right_path.size() - 1; ++j) {
      const auto& p1 = cones[left_path[i]];
      const auto& q1 = cones[left_path[i+1]];
      const auto& p2 = cones[right_path[j]];
      const auto& q2 = cones[right_path[j+1]];
      if (do_segments_intersect(p1, q1, p2, q2)) {
        return true;
      }
    }
  }
  return false;
}


int ConeMatcher::get_orientation(
    const geometry_msgs::msg::Point& p,
    const geometry_msgs::msg::Point& q,
    const geometry_msgs::msg::Point& r)
{
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (std::abs(val) < 1e-9) return 0;  // Collinear
  return (val > 0) ? 1 : 2; // Clockwise or Counterclockwise
}

bool ConeMatcher::on_segment(
    const geometry_msgs::msg::Point& p,
    const geometry_msgs::msg::Point& q,
    const geometry_msgs::msg::Point& r)
{
  return (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
          q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y));
}

bool ConeMatcher::do_segments_intersect(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& q1,
    const geometry_msgs::msg::Point& p2,
    const geometry_msgs::msg::Point& q2)
{
  int o1 = get_orientation(p1, q1, p2);
  int o2 = get_orientation(p1, q1, q2);
  int o3 = get_orientation(p2, q2, p1);
  int o4 = get_orientation(p2, q2, q1);

  if (o1 != o2 && o3 != o4) {
    return true;
  }

  // Special Cases for collinear points
  if (o1 == 0 && on_segment(p1, p2, q1)) return true;
  if (o2 == 0 && on_segment(p1, q2, q1)) return true;
  if (o3 == 0 && on_segment(p2, p1, q2)) return true;
  if (o4 == 0 && on_segment(p2, q1, q2)) return true;

  return false;
}

}  // namespace cone_path_planner
