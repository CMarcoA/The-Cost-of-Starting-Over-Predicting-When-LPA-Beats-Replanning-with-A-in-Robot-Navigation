#!/usr/bin/env python3
import copy

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose


class GridPublisher10x10Low:
    """10x10 low-disturbance experiment.

    Low disturbance definition for this run:
    - small changes only
    - every 5 seconds

    Dynamic pattern:
    A horizontal wall splits the map, with two nearby crossing gaps.
    Every 5 seconds we close one gap and open the other, which changes only 1-2 cells.
    """

    def __init__(self):
        self.pub = rospy.Publisher("/experiment_grid", OccupancyGrid, queue_size=1, latch=True)
        self.width = 10
        self.height = 10
        self.resolution = 1.0
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 1.0)
        self.change_period_sec = rospy.get_param("~change_period_sec", 5.0)

        self.base_grid = self._build_base_grid()
        self.grid = copy.deepcopy(self.base_grid)
        self.last_change_time = rospy.Time.now()
        self.stage = 0
        self._apply_stage()

        rospy.loginfo("grid_publisher.py running for 10x10 LOW disturbance")
        rospy.loginfo("LOW = small changes, every 5 sec")

    def _build_base_grid(self):
        grid = [[0 for _ in range(self.width)] for _ in range(self.height)]

        # Static wall skeleton on row 5 from col 1..8.
        # The disturbance controller below will decide whether col 4 or col 6 is the active gap.
        for c in range(1, 9):
            grid[5][c] = 100

        # A couple of fixed obstacles so the path is not completely trivial.
        grid[2][2] = 100
        grid[2][3] = 100
        grid[7][7] = 100
        grid[7][8] = 100
        return grid

    def _apply_stage(self):
        self.grid = copy.deepcopy(self.base_grid)

        # stage 0: left gap open, right gap closed
        # stage 1: left gap closed, right gap open
        # stage 2: left gap open, right gap closed
        # stage 3: both gaps open (recovery / neutral)
        if self.stage == 0:
            self.grid[5][4] = 0
            self.grid[5][6] = 100
        elif self.stage == 1:
            self.grid[5][4] = 100
            self.grid[5][6] = 0
        elif self.stage == 2:
            self.grid[5][4] = 0
            self.grid[5][6] = 100
        elif self.stage == 3:
            self.grid[5][4] = 0
            self.grid[5][6] = 0

        rospy.loginfo(f"[10x10_low] map stage={self.stage}")

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
    rospy.init_node("grid_publisher_10x10_low")
    node = GridPublisher10x10Low()
    node.spin()
