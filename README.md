# dijkstra algorithm

## Projects 1

1. **`dijkstra_planner`** – 20×20, four-connected grid path planning with Dijkstra's algorithm. It visualizes the grid, obstacles, explored cells, current cell, distance labels, start, goal and final path in RViz. It does not require Gazebo or a physical robot.

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

