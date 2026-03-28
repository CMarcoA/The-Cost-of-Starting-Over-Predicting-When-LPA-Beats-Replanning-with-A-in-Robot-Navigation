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

    def make_layout_1(self):
        data = [0] * (self.width * self.height)

        # wall at x = 4, gap at y = 7
        for y in range(self.height):
            if y != 7:
                index = y * self.width + 4
                data[index] = 100
        return data

    def make_layout_2(self):
        data = [0] * (self.width * self.height)

        # same wall, but gap moves to y = 2
        for y in range(self.height):
            if y != 2:
                index = y * self.width + 4
                data[index] = 100

        return data

