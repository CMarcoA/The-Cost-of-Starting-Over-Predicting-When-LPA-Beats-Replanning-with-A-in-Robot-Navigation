import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class GridPublisher(Node):
    def __init__(self):
        super().__init__('grid_publisher')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)

        # High disturbance: change every 1 second
        self.timer = self.create_timer(1.0, self.publish_grid)

        self.width = 10
        self.height = 10
        self.resolution = 1.0

        self.layout_toggle = False

    def make_layout_with_gaps(self, gap_x3, gap_x5, gap_x7):
        data = [0] * (self.width * self.height)

        # Wall at x = 3
        for y in range(self.height):
            if y != gap_x3:
                index = y * self.width + 3
                data[index] = 100

        # Wall at x = 5
        for y in range(self.height):
            if y != gap_x5:
                index = y * self.width + 5
                data[index] = 100

        # Wall at x = 7
        for y in range(self.height):
            if y != gap_x7:
                index = y * self.width + 7
                data[index] = 100

        return data

    def make_layout_1(self):
        # Forces a top -> bottom -> top snake
        return self.make_layout_with_gaps(
            gap_x3=2,
            gap_x5=7,
            gap_x7=3
        )

    def make_layout_2(self):
        # Forces a bottom -> top -> mid snake
        return self.make_layout_with_gaps(
            gap_x3=7,
            gap_x5=2,
            gap_x7=6
        )

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

        # High disturbance: flip every publish (every 1 second)
        self.layout_toggle = not self.layout_toggle

        if self.layout_toggle:
            msg.data = self.make_layout_1()
            self.get_logger().info('Published 10x10 high layout 1')
        else:
            msg.data = self.make_layout_2()
            self.get_logger().info('Published 10x10 high layout 2')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()