import heapq
import time
from typing import Dict, List, Optional, Tuple

Grid = List[List[int]]
Cell = Tuple[int, int]


class AStarPlanner:
    """Restart planner: recomputes from scratch on every map update."""

    def __init__(self, allow_diagonal: bool = False):
        self.allow_diagonal = allow_diagonal
        self.grid: Optional[Grid] = None
        self.start: Optional[Cell] = None
        self.goal: Optional[Cell] = None
        self.last_result = None

    def initialize(self, grid: Grid, start: Cell, goal: Cell) -> None:
        self.grid = [row[:] for row in grid]
        self.start = start
        self.goal = goal

    def update_grid(self, new_grid: Grid, changed_cells: List[Cell]) -> None:
        self.grid = [row[:] for row in new_grid]

    def plan(self, grid: Grid, start: Cell, goal: Cell):
        self.initialize(grid, start, goal)
        self.last_result = self.replan()
        return self.last_result["path"]

    def replan(self) -> Dict:
        if self.grid is None or self.start is None or self.goal is None:
            raise ValueError("Planner must be initialized before replanning.")

        t0 = time.perf_counter_ns()
        path, expanded = self._astar(self.grid, self.start, self.goal)
        elapsed_us = (time.perf_counter_ns() - t0) / 1000.0

        return {
            "path": path,
            "path_length": max(0, len(path) - 1) if path else float("inf"),
            "planning_time_us": elapsed_us,
            "expanded": expanded,
        }

    def _astar(self, grid: Grid, start: Cell, goal: Cell):
        if not self._is_free(grid, start) or not self._is_free(grid, goal):
            return [], 0

        open_heap = []
        heapq.heappush(open_heap, (self._heuristic(start, goal), 0.0, start))

        g: Dict[Cell, float] = {start: 0.0}
        parent: Dict[Cell, Cell] = {}
        closed = set()
        expanded = 0

        while open_heap:
            f, current_g, current = heapq.heappop(open_heap)

            if current in closed:
                continue

            closed.add(current)
            expanded += 1

            if current == goal:
                return self._reconstruct_path(parent, goal), expanded

            for nxt in self._neighbors(grid, current):
                tentative_g = g[current] + 1.0

                if tentative_g < g.get(nxt, float("inf")):
                    g[nxt] = tentative_g
                    parent[nxt] = current
                    heapq.heappush(
                        open_heap,
                        (tentative_g + self._heuristic(nxt, goal), tentative_g, nxt),
                    )

        return [], expanded

    def _neighbors(self, grid: Grid, cell: Cell) -> List[Cell]:
        r, c = cell
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        if self.allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        out = []
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]) and grid[nr][nc] == 0:
                out.append((nr, nc))
        return out

    def _heuristic(self, a: Cell, b: Cell) -> float:
        ar, ac = a
        br, bc = b

        if self.allow_diagonal:
            return max(abs(ar - br), abs(ac - bc))
        return abs(ar - br) + abs(ac - bc)

    def _is_free(self, grid: Grid, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < len(grid) and 0 <= c < len(grid[0]) and grid[r][c] == 0

    def _reconstruct_path(self, parent: Dict[Cell, Cell], goal: Cell) -> List[Cell]:
        path = [goal]
        current = goal

        while current in parent:
            current = parent[current]
            path.append(current)

        path.reverse()
        return path