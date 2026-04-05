import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class GridPublisher30(Node):
    def __init__(self):
        super().__init__('grid_publisher_30')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)

        # High disturbance: switch every 1 second
        self.timer = self.create_timer(1.0, self.publish_grid)

        self.width = 30
        self.height = 30
        self.resolution = 1.0

        self.layout_toggle = False

    def set_vertical_wall(self, data, wall_x, gap_y):
        for y in range(self.height):
            if y != gap_y:
                index = y * self.width + wall_x
                data[index] = 100

    def make_layout_1(self):
        data = [0] * (self.width * self.height)

        # Three major walls with one-cell gaps
        self.set_vertical_wall(data, wall_x=7, gap_y=4)
        self.set_vertical_wall(data, wall_x=15, gap_y=22)
        self.set_vertical_wall(data, wall_x=23, gap_y=8)

        # Extra blockers to make the disturbance feel larger
        for x in range(9, 14):
            data[10 * self.width + x] = 100

        for x in range(17, 22):
            data[18 * self.width + x] = 100

        return data

    def make_layout_2(self):
        data = [0] * (self.width * self.height)

        # Walls shift positions and gap locations
        self.set_vertical_wall(data, wall_x=9, gap_y=24)
        self.set_vertical_wall(data, wall_x=17, gap_y=6)
        self.set_vertical_wall(data, wall_x=24, gap_y=20)

        # Different extra blockers
        for x in range(11, 16):
            data[20 * self.width + x] = 100

        for x in range(18, 23):
            data[9 * self.width + x] = 100

        return data

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

        # High disturbance: flip every publish
        self.layout_toggle = not self.layout_toggle

        if self.layout_toggle:
            msg.data = self.make_layout_1()
            self.get_logger().info('Published 30x30 high layout 1')
        else:
            msg.data = self.make_layout_2()
            self.get_logger().info('Published 30x30 high layout 2')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridPublisher30()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()