import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class GridPublisher(Node):
    def __init__(self):
        super().__init__('grid_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, '/start_goal_markers', 10)
        self.timer = self.create_timer(1.0, self.publish_grid)

        self.declare_parameter('grid_size', 10)
        self.grid_size = int(self.get_parameter('grid_size').value)

        self.width = self.grid_size
        self.height = self.grid_size
        self.resolution = 1.0

        self.declare_parameter('start_x', 1)
        self.declare_parameter('start_y', 1)
        self.declare_parameter('goal_x', self.width - 2)
        self.declare_parameter('goal_y', self.height - 2)

        self.start_x = int(self.get_parameter('start_x').value)
        self.start_y = int(self.get_parameter('start_y').value)
        self.goal_x = int(self.get_parameter('goal_x').value)
        self.goal_y = int(self.get_parameter('goal_y').value)

        self.get_logger().info(
            f'Grid size: {self.width}x{self.height}, start=({self.start_x},{self.start_y}), goal=({self.goal_x},{self.goal_y})'
        )

        self.layout_toggle = False
        self.publish_count = 0

        self.wall_x = self.width // 2
        self.gap_top_y = self.height // 4
        self.gap_bottom_y = (3 * self.height) // 4

    def make_layout_with_gap(self, gap_y):
        data = [0] * (self.width * self.height)

        for y in range(self.height):
            if y != gap_y:
                index = y * self.width + self.wall_x
                data[index] = 100

        return data

    def make_layout_1(self):
        return self.make_layout_with_gap(self.gap_bottom_y)

    def make_layout_2(self):
        return self.make_layout_with_gap(self.gap_top_y)

    def publish_grid(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        origin = Pose()
        origin.position.x = 0.0
        origin.position.y = 0.0
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        msg.info.origin = origin

        self.publish_count += 1
        if self.publish_count % 5 == 0:
            self.layout_toggle = not self.layout_toggle

        if self.layout_toggle:
            msg.data = self.make_layout_2()
            self.get_logger().info(f'Published {self.width}x{self.height} layout 2')
        else:
            msg.data = self.make_layout_1()
            self.get_logger().info(f'Published {self.width}x{self.height} layout 1')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()