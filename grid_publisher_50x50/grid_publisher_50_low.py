import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class GridPublisher50(Node):
    def __init__(self):
        super().__init__('grid_publisher_50')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_grid)

        self.width = 50
        self.height = 50
        self.resolution = 1.0

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
        # Low disturbance layout 1: lower opening
        return self.make_layout_with_gap(self.gap_bottom_y)

    def make_layout_2(self):
        # Low disturbance layout 2: upper opening
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
            self.get_logger().info('Published 50x50 low layout 2')
        else:
            msg.data = self.make_layout_1()
            self.get_logger().info('Published 50x50 low layout 1')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridPublisher50()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()