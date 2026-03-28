10x10 low-disturbance experiment

Files included:
- __init__.py
- astar.py
- lpastar.py
- planner_factory.py
- planner_node.py
- grid_publisher.py
- grid_publisher_30.py
- grid_publisher_50.py

Default experiment settings in this step:
- grid size: 10x10
- disturbance level: low
- low = small changes every 5 sec
- default start: [1, 1]
- default goal: [8, 8]
- map topic: /experiment_grid

Suggested run order:
1) Start roscore
2) Run grid publisher:
   rosrun <your_pkg> grid_publisher.py
3) Run A* node:
   rosrun <your_pkg> planner_node.py _planner:=astar _run_label:=10x10_low
4) Run LPA* node:
   rosrun <your_pkg> planner_node.py _planner:=lpastar _run_label:=10x10_low

Metrics are appended to:
- /tmp/10x10_low_astar.csv
- /tmp/10x10_low_lpastar.csv

What should happen:
- The wall opening alternates every 5 seconds.
- The path should switch between the left crossing and the right crossing.
- A* recomputes from scratch each time.
- LPA* should usually expand fewer states after the first plan.
