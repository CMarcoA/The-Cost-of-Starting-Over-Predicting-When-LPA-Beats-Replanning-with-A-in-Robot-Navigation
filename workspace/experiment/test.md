cd ~/lpa_star_ws/src/path_planning_sim/path_planning_sim
gedit grid_publisher.py

# rebuild
cd ~/lpa_star_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash

# test 10x10 / 30x30 / 50x50 - TERMINAL 1
cd ~/lpa_star_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run path_planning_sim grid_publisher --ros-args -p grid_size:=30
## specific for 50x50
ros2 run path_planning_sim grid_publisher --ros-args -p grid_size:=50 -p start_x:=1 -p start_y:=1 -p goal_x:=48 -p goal_y:=48

# test algorithm - TERMINAL 2
cd ~/lpa_star_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run path_planning_sim planner_node --ros-args -p planner_type:=astar