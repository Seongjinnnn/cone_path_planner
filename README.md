# cone_path_planner

## Description

The `cone_path_planner` package is responsible for taking a collection of detected cones, sorting them into left and right paths, and generating a centerline path for the vehicle to follow.

## Dependencies

- `rclcpp`
- `geometry_msgs`
- `tf2`
- `tf2_ros`
- `visualization_msgs`
- `builtin_interfaces`
- `nav_msgs`

## Parameters

The following parameters can be configured for the `path_planner_node`:

- `target_frame`: The target frame for the planned path. (default: "odom")
- `vehicle_frame`: The frame of the vehicle. (default: "mld_base_link")
- `max_dist_to_first`: Maximum distance to the first cone. (default: 5.0)
- `max_n_neighbors`: Maximum number of neighbors to consider for path finding. (default: 5)
- `max_dist`: Maximum distance between cones in a path. (default: 7.0)
- `kMaxLen`: Maximum length of a cone path. (default: 12)
- `thr_abs`: Absolute angle threshold for path finding. (default: 1.22)
- `thr_dir`: Directional angle threshold for path finding. (default: 0.698)
- `angle_weight`: Weight for the angle score in path scoring. (default: 1000.0)
- `length_weight`: Weight for the length score in path scoring. (default: 5000.0)
- `major_r`: Major radius for cone matching. (default: 8.0)
- `minor_r`: Minor radius for cone matching. (default: 4.0)
- `min_track_w`: Minimum track width for virtual cone generation. (default: 3.0)

## Usage

To launch the package, run the following command:

```bash
ros2 launch cone_path_planner cone_path_planner.launch.py
```
