# ROS Noetic – Two Mobile Robotics Course Projects

This repository contains two independent ROS 1 Noetic catkin packages based on the uploaded PERUT525 course-project report.

## Projects

1. **`dijkstra_planner`** – 20×20, four-connected grid path planning with Dijkstra's algorithm. It visualizes the grid, obstacles, explored cells, current cell, distance labels, start, goal and final path in RViz. It does not require Gazebo or a physical robot.
2. **`turtlebot_obstacle_avoidance`** – reactive LiDAR obstacle avoidance for TurtleBot3 Burger in Gazebo. It subscribes to `/scan` and publishes velocity commands to `/cmd_vel`.

Both packages use Python 3 (`#!/usr/bin/env python3`) and `rospy`, and are intended for Ubuntu 20.04 + ROS Noetic.

## Repository layout

```text
ros_noetic_two_projects/
├── README.md
├── .gitignore
└── catkin_ws/
    └── src/
        ├── dijkstra_planner/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── launch/dijkstra.launch
        │   ├── scripts/dijkstra_node.py
        │   └── rviz/dijkstra.rviz
        └── turtlebot_obstacle_avoidance/
            ├── CMakeLists.txt
            ├── package.xml
            ├── launch/obstacle_avoidance.launch
            └── scripts/obstacle_avoidance.py
```

## 1. Install dependencies

```bash
sudo apt update
sudo apt install ros-noetic-desktop-full ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
```

If you already have ROS Noetic installed, install the missing TurtleBot3 packages as needed.

## 2. Build the repository

Copy/clone this repository, then:

```bash
cd ~/ros_noetic_two_projects/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

To source this workspace automatically in new terminals:

```bash
echo "source ~/ros_noetic_two_projects/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Adjust the path if you cloned the repository somewhere else.

---

## Project 1: Dijkstra path planning (RViz only)

### Run

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_noetic_two_projects/catkin_ws/devel/setup.bash
roscore
```

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_noetic_two_projects/catkin_ws/devel/setup.bash
roslaunch dijkstra_planner dijkstra.launch
```

The launch file starts the Dijkstra node and RViz. The RViz configuration subscribes to `/dijkstra/markers`.

### Parameters

The node exposes ROS parameters under the `~` namespace:

- `grid_width` (default `20`)
- `grid_height` (default `20`)
- `cell_size` (default `0.5` metres)
- `start_x`, `start_y` (defaults `1`, `1`)
- `goal_x`, `goal_y` (defaults `18`, `18`)
- `animation_rate` (default `10.0` Hz)
- `show_distance_labels` (default `true`)

Obstacle cells are defined in `scripts/dijkstra_node.py` in `build_obstacles()`. The defaults reproduce the barrier layout described in the report. Edit this function to change the environment.

### RViz topics

- `/dijkstra/markers` – `visualization_msgs/MarkerArray`
- Fixed frame: `map`

Marker namespaces distinguish grid, obstacles, visited cells, current cell, path, start, goal and distance labels.

---

## Project 2: TurtleBot3 Burger LiDAR obstacle avoidance

### Run Gazebo

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
roscore
```

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Terminal 3:

```bash
source /opt/ros/noetic/setup.bash
source ~/ros_noetic_two_projects/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot_obstacle_avoidance obstacle_avoidance.launch
```

Optional Terminal 4 for RViz:

```bash
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup rviz.launch
```

The controller uses the report's decision rule:

- If front range is at least `0.7 m`, move forward at `0.18 m/s`.
- If front range is below `0.7 m`, stop forward motion and rotate towards the side with the larger measured clearance at `0.8 rad/s`.

### ROS interfaces

- Subscriber: `/scan` (`sensor_msgs/LaserScan`)
- Publisher: `/cmd_vel` (`geometry_msgs/Twist`)

Parameters can be changed in `launch/obstacle_avoidance.launch` or supplied as private parameters:
`front_threshold`, `forward_speed`, `turn_speed`, `invalid_range`, `scan_topic`, and `cmd_vel_topic`.

## Important notes

- Project 1 is a grid-search visualization. It does not drive a robot and does not use Gazebo, SLAM, or sensor data.
- Project 2 is a simple reactive controller, not a complete navigation stack. It does not guarantee escape from every concave or trap-shaped obstacle layout.
- Test in simulation first. Do not run the controller on physical hardware without additional testing, emergency-stop provisions, and suitable safety measures.
- If `roslaunch turtlebot3_gazebo ...` is not found, install the TurtleBot3 simulation packages and resolve dependencies with `rosdep`.

## GitHub upload

From the repository root:

```bash
git init
git add .
git commit -m "Add two ROS Noetic mobile robotics projects"
git branch -M main
git remote add origin https://github.com/YOUR_USERNAME/YOUR_REPOSITORY.git
git push -u origin main
```

Replace the remote URL with your own GitHub repository URL.
