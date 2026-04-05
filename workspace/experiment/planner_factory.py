from path_planning_sim.astar import AStarPlanner
from path_planning_sim.lpastar import LPAStarPlanner


def create_planner(planner_type: str):
    if planner_type == 'astar':
        return AStarPlanner()
    elif planner_type == 'lpastar':
        return LPAStarPlanner()
    else:
        raise ValueError(f'Unknown planner_type: {planner_type}')
