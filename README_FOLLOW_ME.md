# Go2 Navigation: Dynamic Follow Me Strategy

This guide explains the changes made to the Nav2 launch files and how to use the dynamic object following strategy on your Unitree Go2 robot.

## What `navigation.launch.py` Does

The `navigation.launch.py` file in the `go2_nav` package is your primary entry point for starting the Nav2 stack. It includes:
- **Localization**: Uses `localization_launch.py` (e.g. AMCL) to know where the robot is in the map.
- **Navigation Core**: Uses `navigation_launch.py` which spins up all Nav2 servers (Planner, Controller, Recoveries, BT Navigator).
- **Behavior Tree Configuration**: It dynamically sets the `default_bt_xml_filename` argument to point to `follow_point.xml`.

By passing `default_bt_xml_filename` natively in this launch file, we bypass the limitations of `.yaml` files (which cannot use ROS 2 package variables like `get_package_share_directory`) while still making the follow-me behavior the default mode for this specific launch profile.

---

## How to Implement and Use the Follow Me Strategy

The Follow Me strategy leverages Nav2's `GoalUpdater` and `TruncatePath` behavior tree nodes.

### 1. The Behavior Tree (`follow_point.xml`)
The custom tree is located at `src/go2_nav/behavior_trees/follow_point.xml`.
When active, Nav2 will:
1. Listen for new 3D points/poses on the `/goal_update` topic.
2. Continually compute a path to that point.
3. Automatically trim the path by `1.0` meters (`<TruncatePath distance="1.0" .../>`) to ensure the robot stops before colliding with the tracked object.
4. Issue velocity commands via the local trajectory planner (DWB) to follow the path.

### 2. Running the Code
Build the updated package on your Jetson and source the workspace:
```bash
colcon build --packages-select go2_nav
source install/setup.bash
```

Launch the navigation stack:
```bash
ros2 launch go2_nav navigation.launch.py
```
*Note: Because we modified `navigation.launch.py`, Nav2 will automatically load the `follow_point.xml` tree.*

### 3. Start Following!
To command the robot to start following:
1. Send a standard `NavigateToPose` action goal to Nav2 (using an Action Client in your code, or via RViz). The initial goal doesn't matter too much as it will be overwritten.
2. Have your perception node publish the detected object's coordinates (as a `geometry_msgs/msg/PoseStamped`) to the `/goal_update` topic at a high rate (e.g., 10 Hz).

**Example of publishing a target manually:**
```bash
ros2 topic pub /goal_update geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}"
```

As long as the `/goal_update` topic receives new poses, the robot will seamlessly adjust its path and follow the object, keeping a 1.0m clearance.

### 4. How to Revert to Normal Navigation
If you want to use standard Point A to Point B navigation without the following behavior, you can override the launch argument when starting the launch file:
```bash
ros2 launch go2_nav navigation.launch.py default_bt_xml_filename:=/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml
```
(Replace `/opt/ros/foxy/` with your ROS 2 installation path if necessary).
