import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class GridPublisher(Node):
    def __init__(self):
        super().__init__('grid_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_grid)

        self.width = 10
        self.height = 10
        self.resolution = 1.0

        self.layout_toggle = False
        self.publish_count = 0

    def make_layout_with_gaps(self, wall1_gap_y, wall2_gap_y):
        data = [0] * (self.width * self.height)

        # Wall 1 at x = 4
        for y in range(self.height):
            if y != wall1_gap_y:
                index = y * self.width + 4
                data[index] = 100

        # Wall 2 at x = 6
        for y in range(self.height):
            if y != wall2_gap_y:
                index = y * self.width + 6
                data[index] = 100

        return data

    def make_layout_1(self):
        # First wall opens low, second wall opens high
        # Forces an S-shaped reroute
        return self.make_layout_with_gaps(wall1_gap_y=7, wall2_gap_y=2)

    def make_layout_2(self):
        # First wall opens high, second wall opens low
        # Forces the opposite reroute
        return self.make_layout_with_gaps(wall1_gap_y=2, wall2_gap_y=7)

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
            self.get_logger().info('Published 10x10 medium layout 2')
        else:
            msg.data = self.make_layout_1()
            self.get_logger().info('Published 10x10 medium layout 1')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()