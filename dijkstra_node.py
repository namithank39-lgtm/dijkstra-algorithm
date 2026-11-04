#!/usr/bin/env python3
"""
Dijkstra grid planner for ROS Noetic.

Publishes a MarkerArray on /dijkstra/markers.
RViz marker namespaces:
grid, obstacles, visited, current, path,
start, goal, distance_labels.
"""

import heapq
import math

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class DijkstraPlanner:

    def __init__(self):

        rospy.init_node("dijkstra_path_planner")

        self.width = int(rospy.get_param("~grid_width", 20))
        self.height = int(rospy.get_param("~grid_height", 20))
        self.cell_size = float(rospy.get_param("~cell_size", 0.5))

        self.start = (
            int(rospy.get_param("~start_x", 1)),
            int(rospy.get_param("~start_y", 1)),
        )

        self.goal = (
            int(rospy.get_param("~goal_x", 18)),
            int(rospy.get_param("~goal_y", 18)),
        )

        self.rate_hz = max(
            0.1,
            float(rospy.get_param("~animation_rate", 10.0))
        )

        self.show_labels = bool(
            rospy.get_param("~show_distance_labels", True)
        )

        self.frame_id = "map"

        if self.width <= 0 or self.height <= 0 or self.cell_size <= 0:
            raise ValueError(
                "Grid dimensions and cell_size must be positive."
            )

        if not self.in_bounds(self.start) or not self.in_bounds(self.goal):
            raise ValueError(
                "Start and goal must be inside the configured grid."
            )

        self.obstacles = self.build_obstacles()

        if self.start in self.obstacles or self.goal in self.obstacles:
            raise ValueError(
                "Start and goal cannot be obstacle cells."
            )

        self.publisher = rospy.Publisher(
            "/dijkstra/markers",
            MarkerArray,
            queue_size=1,
            latch=True
        )

        self.marker_id = 0

        rospy.sleep(0.5)

    # Check whether a cell is inside the grid

    def in_bounds(self, cell):

        x, y = cell

        return (
            0 <= x < self.width and
            0 <= y < self.height
        )

    # Define obstacle positions

    def build_obstacles(self):

        blocked = set()

        # Vertical obstacle 1

        for y in range(5, 16):
            blocked.add((7, y))

        # Horizontal obstacle

        for x in range(7, 15):
            blocked.add((x, 15))

        # Vertical obstacle 2

        for y in range(2, 10):
            blocked.add((13, y))

        # Horizontal obstacle 2

        for x in range(2, 6):
            blocked.add((x, 10))

        return {
            cell for cell in blocked
            if self.in_bounds(cell)
        }

    # Calculate the centre position of a grid cell

    def cell_center(self, cell):

        x, y = cell

        return Point(
            (x + 0.5) * self.cell_size,
            (y + 0.5) * self.cell_size,
            0.0
        )

    # Create a ROS visualization marker

    def make_marker(self, namespace, marker_type, marker_id=0):

        marker = Marker()

        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()

        marker.ns = namespace
        marker.id = marker_id

        marker.type = marker_type
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.lifetime = rospy.Duration(0)

        return marker

    # Create cube markers for grid cells

    def cube_list(self, namespace, cells, color,
                  scale_z=0.035, z=0.0):

        marker = self.make_marker(
            namespace,
            Marker.CUBE_LIST
        )

        marker.scale.x = self.cell_size * 0.92
        marker.scale.y = self.cell_size * 0.92
        marker.scale.z = scale_z

        (
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a
        ) = color

        for cell in sorted(cells):

            point = self.cell_center(cell)
            point.z = z

            marker.points.append(point)

        return marker

    # Create a marker for a particular cell

    def point_marker(self, namespace, cell,
                     color, scale, z=0.12):

        marker = self.make_marker(
            namespace,
            Marker.CUBE
        )

        marker.pose.position = self.cell_center(cell)
        marker.pose.position.z = z

        (
            marker.scale.x,
            marker.scale.y,
            marker.scale.z
        ) = scale

        (
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a
        ) = color

        return marker

    # Display the final shortest path

    def path_marker(self, path):

        marker = self.make_marker(
            "path",
            Marker.LINE_STRIP
        )

        marker.pose.orientation.w = 1.0

        marker.scale.x = max(
            0.035,
            self.cell_size * 0.10
        )

        (
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a
        ) = (0.0, 0.9, 0.2, 1.0)

        for cell in path:

            p = self.cell_center(cell)
            p.z = 0.20

            marker.points.append(p)

        return marker

    # Display cumulative distance values

    def distance_markers(self, distances):

        markers = []

        if not self.show_labels:
            return markers

        marker_id = 0

        for (x, y), distance in sorted(distances.items()):

            marker = self.make_marker(
                "distance_labels",
                Marker.TEXT_VIEW_FACING,
                marker_id
            )

            marker_id += 1

            marker.pose.position = self.cell_center((x, y))
            marker.pose.position.z = 0.10

            marker.scale.z = max(
                0.07,
                self.cell_size * 0.20
            )

            (
                marker.color.r,
                marker.color.g,
                marker.color.b,
                marker.color.a
            ) = (0.1, 0.1, 0.1, 1.0)

            marker.text = str(distance)

            markers.append(marker)

        return markers

    # Publish all visualization markers

    def publish(self, visited, current=None,
                path=None, distances=None):

        array = MarkerArray()

        # Grid

        array.markers.append(
            self.cube_list(
                "grid",
                (
                    (x, y)
                    for x in range(self.width)
                    for y in range(self.height)
                ),
                (0.75, 0.75, 0.75, 0.35),
                scale_z=0.025,
                z=-0.02
            )
        )

        # Obstacles

        array.markers.append(
            self.cube_list(
                "obstacles",
                self.obstacles,
                (0.15, 0.15, 0.15, 1.0)
            )
        )

        # Visited cells

        array.markers.append(
            self.cube_list(
                "visited",
                visited,
                (0.15, 0.45, 0.95, 0.75),
                z=0.045
            )
        )

        # Current cell

        if current is not None:

            array.markers.append(
                self.point_marker(
                    "current",
                    current,
                    (1.0, 0.65, 0.0, 1.0),
                    (
                        self.cell_size * 0.75,
                        self.cell_size * 0.75,
                        0.10
                    ),
                    z=0.10
                )
            )

        # Final path

        if path:
            array.markers.append(
                self.path_marker(path)
            )

        # Start marker

        array.markers.append(
            self.point_marker(
                "start",
                self.start,
                (0.0, 0.8, 0.1, 1.0),
                (
                    self.cell_size * 0.80,
                    self.cell_size * 0.80,
                    0.16
                ),
                z=0.15
            )
        )

        # Goal marker

        array.markers.append(
            self.point_marker(
                "goal",
                self.goal,
                (0.95, 0.1, 0.1, 1.0),
                (
                    self.cell_size * 0.80,
                    self.cell_size * 0.80,
                    0.16
                ),
                z=0.15
            )
        )

        # Distance labels

        if distances is not None:

            array.markers.extend(
                self.distance_markers(distances)
            )

        self.publisher.publish(array)

    # Generate valid neighbouring cells

    def neighbours(self, cell):

        x, y = cell

        candidates = (
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1)
        )

        return [
            nxt for nxt in candidates
            if self.in_bounds(nxt)
            and nxt not in self.obstacles
        ]

    # Reconstruct shortest path

    def reconstruct_path(self, previous):

        path = []

        node = self.goal

        while node != self.start:

            path.append(node)

            if node not in previous:
                return []

            node = previous[node]

        path.append(self.start)

        path.reverse()

        return path

    # Main Dijkstra algorithm

    def run(self):

        distances = {
            self.start: 0
        }

        previous = {}

        visited = set()

        queue = [
            (0, self.start)
        ]

        rate = rospy.Rate(self.rate_hz)

        rospy.loginfo(
            "Dijkstra planner started: %s -> %s",
            self.start,
            self.goal
        )

        while queue and not rospy.is_shutdown():

            current_distance, current = heapq.heappop(queue)

            if current in visited:
                continue

            visited.add(current)

            self.publish(
                visited,
                current,
                distances=distances
            )

            if current == self.goal:
                break

            for nxt in self.neighbours(current):

                if nxt in visited:
                    continue

                new_distance = current_distance + 1

                if new_distance < distances.get(nxt, math.inf):

                    distances[nxt] = new_distance

                    previous[nxt] = current

                    heapq.heappush(
                        queue,
                        (new_distance, nxt)
                    )

            rate.sleep()

        if rospy.is_shutdown():
            return

        # Generate final path

        path = (
            self.reconstruct_path(previous)
            if self.goal in distances
            else []
        )

        self.publish(
            visited,
            None,
            path=path,
            distances=distances
        )

        if path:

            rospy.loginfo(
                "Path found. Number of cells: %d; movement cost: %d",
                len(path),
                len(path) - 1
            )

        else:

            rospy.logwarn(
                "No collision-free path exists for this grid."
            )

        rospy.loginfo(
            "Visualization remains active. Press Ctrl+C to stop."
        )

        rospy.spin()


if __name__ == "__main__":

    try:

        DijkstraPlanner().run()

    except (rospy.ROSInterruptException, KeyboardInterrupt):

        pass
