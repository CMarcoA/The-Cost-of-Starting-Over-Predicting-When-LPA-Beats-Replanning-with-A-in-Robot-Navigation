import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

from path_planning_sim.planner_factory import create_planner


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.declare_parameter('planner_type', 'astar')
        planner_type = self.get_parameter('planner_type').value
        self.planner = create_planner(planner_type)

        self.get_logger().info(f'Using planner: {planner_type}')

        self.timer = self.create_timer(1.0, self.publish_path)

        self.grid = None
        self.width = 0
        self.height = 0
        self.resolution = 1.0

        self.start = None
        self.goal = None

    def map_callback(self, msg: OccupancyGrid):
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution

        self.grid = []
        for y in range(self.height):
            row = []
            for x in range(self.width):
                index = y * self.width + x
                value = msg.data[index]

                if value > 50 or value < 0:
                    row.append(1)
                else:
                    row.append(0)

            self.grid.append(row)

        middle_row = self.height // 2
        self.start = (1, middle_row)
        self.goal = (self.width - 2, middle_row)

        self.get_logger().info(
            f'Received map | size={self.width}x{self.height} | start={self.start} | goal={self.goal}'
        )

    def publish_path(self):
        if self.grid is None or self.start is None or self.goal is None:
            self.get_logger().info('Waiting for map...')
            return

        start_time = time.perf_counter()
        cell_path = self.planner.plan(self.grid, self.start, self.goal)
        end_time = time.perf_counter()

        planning_time_ms = (end_time - start_time) * 1000.0
        path_length = len(cell_path)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for x, y in cell_path:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'

            pose.pose.position.x = (x + 0.5) * self.resolution
            pose.pose.position.y = (y + 0.5) * self.resolution
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info(
            f'Published path with {path_length} cells | planning time: {planning_time_ms:.3f} ms | grid={self.width}x{self.height}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
