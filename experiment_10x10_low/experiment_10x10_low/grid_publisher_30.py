#!/usr/bin/env python3
import copy

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


class GridPublisher30x30Low:
    """30x30 low-disturbance experiment.

    Low disturbance:
    - small/local map changes
    - every 5 seconds

    Dynamic pattern:
    A central horizontal wall has two nearby crossing gaps.
    Every 5 seconds we switch which gap is open.
    """

    def __init__(self):
        self.pub = rospy.Publisher("/experiment_grid", OccupancyGrid, queue_size=1, latch=True)
        self.width = 30
        self.height = 30
        self.resolution = 1.0
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 1.0)
        self.change_period_sec = rospy.get_param("~change_period_sec", 5.0)

        self.base_grid = self._build_base_grid()
        self.grid = copy.deepcopy(self.base_grid)
        self.last_change_time = rospy.Time.now()
        self.stage = 0
        self._apply_stage()

        rospy.loginfo("grid_publisher_30.py running for 30x30 LOW disturbance")
        rospy.loginfo("LOW = small changes, every 5 sec")

    def _build_base_grid(self):
        grid = [[0 for _ in range(self.width)] for _ in range(self.height)]

        # Central wall on row 15. The gap state will be controlled dynamically.
        for c in range(2, 28):
            grid[15][c] = 100

        # Fixed obstacles to make the path less trivial.
        fixed_blocks = [
            (5, 5), (5, 6), (6, 5), (6, 6),
            (8, 20), (9, 20), (10, 20),
            (20, 8), (20, 9), (20, 10),
            (22, 22), (22, 23), (23, 22), (23, 23),
        ]
        for r, c in fixed_blocks:
            grid[r][c] = 100

        return grid

    def _apply_stage(self):
        self.grid = copy.deepcopy(self.base_grid)

        # Two nearby candidate gaps in the central wall.
        # stage 0: left gap open, right gap closed
        # stage 1: left gap closed, right gap open
        # stage 2: left gap open, right gap closed
        # stage 3: both gaps open
        left_gap = 13
        right_gap = 16

        if self.stage == 0:
            self.grid[15][left_gap] = 0
            self.grid[15][right_gap] = 100
        elif self.stage == 1:
            self.grid[15][left_gap] = 100
            self.grid[15][right_gap] = 0
        elif self.stage == 2:
            self.grid[15][left_gap] = 0
            self.grid[15][right_gap] = 100
        elif self.stage == 3:
            self.grid[15][left_gap] = 0
            self.grid[15][right_gap] = 0

        rospy.loginfo(f"[30x30_low] map stage={self.stage}")

    def _build_msg(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        info = MapMetaData()
        info.map_load_time = rospy.Time.now()
        info.resolution = self.resolution
        info.width = self.width
        info.height = self.height
        info.origin = Pose()
        msg.info = info

        flat = []
        for row in self.grid:
            flat.extend(row)
        msg.data = flat
        return msg

    def spin(self):
        rate = rospy.Rate(self.publish_rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if (now - self.last_change_time).to_sec() >= self.change_period_sec:
                self.stage = (self.stage + 1) % 4
                self._apply_stage()
                self.last_change_time = now

            self.pub.publish(self._build_msg())
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("grid_publisher_30x30_low")
    node = GridPublisher30x30Low()
    node.spin()