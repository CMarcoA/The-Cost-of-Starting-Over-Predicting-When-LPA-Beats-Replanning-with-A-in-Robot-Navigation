from .astar import AStarPlanner
from .lpastar import LPAStarPlanner


def create_planner(name: str, allow_diagonal: bool = False):
    key = name.strip().lower()
    if key in {"astar", "a*"}:
        return AStarPlanner(allow_diagonal=allow_diagonal)
    if key in {"lpastar", "lpa*", "lifelong_planning_astar"}:
        return LPAStarPlanner(allow_diagonal=allow_diagonal)
    raise ValueError(f"Unknown planner: {name}")
