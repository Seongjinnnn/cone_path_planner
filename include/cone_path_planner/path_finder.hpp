#ifndef PATH_FINDER_HPP_
#define PATH_FINDER_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>
#include <unordered_set>

namespace cone_path_planner {

class PathFinder {
public:
  PathFinder(int kMaxLen, double thr_abs, double thr_dir);

  std::vector<std::vector<int>> find_configurations(
      int start_node,
      const std::vector<std::vector<bool>>& adj_matrix,
      const std::vector<geometry_msgs::msg::Point>& cones,
      const std::string& cone_type,
      const std::unordered_set<int>& used_cones);

private:
  int kMaxLen_;
  double thr_abs_;
  double thr_dir_;
};

}  // namespace cone_path_planner

#endif  // PATH_FINDER_HPP_
