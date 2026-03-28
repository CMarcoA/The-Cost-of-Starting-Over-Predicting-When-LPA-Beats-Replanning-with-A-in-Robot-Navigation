#!/usr/bin/env python3
import csv
import os
from typing import List, Tuple

import rospy
from nav_msgs.msg import OccupancyGrid

from planner_factory import create_planner

Grid = List[List[int]]
Cell = Tuple[int, int]


class PlannerExperimentNode:
    def __init__(self):
        self.planner_name = rospy.get_param("~planner", "astar")
        self.allow_diagonal = rospy.get_param("~allow_diagonal", False)
        self.grid_topic = rospy.get_param("~grid_topic", "/experiment_grid")
        self.start = tuple(rospy.get_param("~start", [1, 1]))
        self.goal = tuple(rospy.get_param("~goal", [8, 8]))
        self.run_label = rospy.get_param("~run_label", "10x10_low")
        self.csv_path = rospy.get_param(
            "~csv_path",
            f"/tmp/{self.run_label}_{self.planner_name}.csv",
        )

        self.planner = create_planner(self.planner_name, self.allow_diagonal)
        self.prev_grid = None
        self.update_index = -1
        self.history = []

        self._ensure_csv_header()
        rospy.Subscriber(self.grid_topic, OccupancyGrid, self._grid_callback, queue_size=1)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo(
            f"planner_node started | planner={self.planner_name} | run={self.run_label} | csv={self.csv_path}"
        )

    def _ensure_csv_header(self):
        folder = os.path.dirname(self.csv_path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        if not os.path.exists(self.csv_path):
            with open(self.csv_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        "run_label",
                        "planner",
                        "update_index",
                        "changed_cells",
                        "planning_time_us",
                        "path_length",
                        "expanded",
                    ]
                )

    def _grid_callback(self, msg: OccupancyGrid):
        grid = self._msg_to_grid(msg)

        if self.prev_grid is not None and grid == self.prev_grid:
            return

        changed_cells = self._diff_cells(self.prev_grid, grid) if self.prev_grid is not None else []

        if self.prev_grid is None:
            self.planner.initialize(grid, self.start, self.goal)
        else:
            self.planner.update_grid(grid, changed_cells)

        self.update_index += 1
        result = self.planner.replan()
        self.prev_grid = [row[:] for row in grid]

        row = {
            "run_label": self.run_label,
            "planner": self.planner_name,
            "update_index": self.update_index,
            "changed_cells": len(changed_cells),
            "planning_time_us": round(result["planning_time_us"], 2),
            "path_length": result["path_length"],
            "expanded": result["expanded"],
        }
        self.history.append(row)
        self._append_csv(row)

        rospy.loginfo(
            f"[{self.run_label}] planner={self.planner_name} update={self.update_index} "
            f"changed={len(changed_cells)} planning_time_us={row['planning_time_us']} "
            f"path_length={row['path_length']} expanded={row['expanded']}"
        )

    def _append_csv(self, row):
        with open(self.csv_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    row["run_label"],
                    row["planner"],
                    row["update_index"],
                    row["changed_cells"],
                    row["planning_time_us"],
                    row["path_length"],
                    row["expanded"],
                ]
            )

    def _msg_to_grid(self, msg: OccupancyGrid) -> Grid:
        width = msg.info.width
        height = msg.info.height
        data = list(msg.data)
        grid = []
        for r in range(height):
            row = []
            for c in range(width):
                value = data[r * width + c]
                row.append(1 if value >= 50 else 0)
            grid.append(row)
        return grid

    def _diff_cells(self, old: Grid, new: Grid) -> List[Cell]:
        changed = []
        for r in range(len(new)):
            for c in range(len(new[0])):
                if old[r][c] != new[r][c]:
                    changed.append((r, c))
        return changed

    def _on_shutdown(self):
        if not self.history:
            return
        avg_us = sum(item["planning_time_us"] for item in self.history) / len(self.history)
        last_path = self.history[-1]["path_length"]
        rospy.loginfo(
            f"[{self.run_label}] FINAL SUMMARY | planner={self.planner_name} "
            f"updates={len(self.history)} avg_planning_time_us={round(avg_us, 2)} "
            f"last_path_length={last_path} csv={self.csv_path}"
        )


if __name__ == "__main__":
    rospy.init_node("planner_experiment_node")
    PlannerExperimentNode()
    rospy.spin()
