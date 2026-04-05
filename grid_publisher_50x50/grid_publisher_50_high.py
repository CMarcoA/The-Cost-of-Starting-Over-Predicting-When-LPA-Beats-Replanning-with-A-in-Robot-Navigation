import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class GridPublisher50(Node):
    def __init__(self):
        super().__init__('grid_publisher_50')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)

        # High disturbance: switch every 1 second
        self.timer = self.create_timer(1.0, self.publish_grid)

        self.width = 50
        self.height = 50
        self.resolution = 1.0

        self.layout_toggle = False

    def set_vertical_wall(self, data, wall_x, gap_y):
        for y in range(self.height):
            if y != gap_y:
                index = y * self.width + wall_x
                data[index] = 100

    def add_horizontal_segment(self, data, row_y, start_x, end_x):
        for x in range(start_x, end_x + 1):
            index = row_y * self.width + x
            data[index] = 100

    def make_layout_1(self):
        data = [0] * (self.width * self.height)

        # Major walls
        self.set_vertical_wall(data, wall_x=12, gap_y=8)
        self.set_vertical_wall(data, wall_x=25, gap_y=40)
        self.set_vertical_wall(data, wall_x=38, gap_y=14)

        # Attached stubs: obstructive, but not sealing the full corridor
        self.add_horizontal_segment(data, row_y=28, start_x=12, end_x=20)
        self.add_horizontal_segment(data, row_y=18, start_x=25, end_x=33)
        self.add_horizontal_segment(data, row_y=36, start_x=38, end_x=45)

        return data

    def make_layout_2(self):
        data = [0] * (self.width * self.height)

        # Shifted walls and very different gaps
        self.set_vertical_wall(data, wall_x=10, gap_y=41)
        self.set_vertical_wall(data, wall_x=24, gap_y=10)
        self.set_vertical_wall(data, wall_x=37, gap_y=34)

        # Different attached stubs
        self.add_horizontal_segment(data, row_y=14, start_x=10, end_x=18)
        self.add_horizontal_segment(data, row_y=30, start_x=24, end_x=32)
        self.add_horizontal_segment(data, row_y=22, start_x=37, end_x=45)

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
            self.get_logger().info('Published 50x50 high layout 1')
        else:
            msg.data = self.make_layout_2()
            self.get_logger().info('Published 50x50 high layout 2')

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GridPublisher50()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()