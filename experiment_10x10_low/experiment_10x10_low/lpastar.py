import heapq
import math
import time
from typing import Dict, List, Optional, Tuple

Grid = List[List[int]]
Cell = Tuple[int, int]


class LPAStarPlanner:
    """Repair planner: reuses previous search state after grid changes.

    This is a practical grid-based LPA* for fixed start/goal experiments.
    """

    def __init__(self, allow_diagonal: bool = False):
        self.allow_diagonal = allow_diagonal
        self.grid: Optional[Grid] = None
        self.start: Optional[Cell] = None
        self.goal: Optional[Cell] = None
        self.g: Dict[Cell, float] = {}
        self.rhs: Dict[Cell, float] = {}
        self.open_heap: List[Tuple[float, float, Cell]] = []
        self.open_best_key: Dict[Cell, Tuple[float, float]] = {}
        self.expanded_last = 0

    def initialize(self, grid: Grid, start: Cell, goal: Cell) -> None:
        self.grid = [row[:] for row in grid]
        self.start = start
        self.goal = goal
        self.g.clear()
        self.rhs.clear()
        self.open_heap.clear()
        self.open_best_key.clear()

        for r in range(len(self.grid)):
            for c in range(len(self.grid[0])):
                self.g[(r, c)] = math.inf
                self.rhs[(r, c)] = math.inf

        self.rhs[start] = 0.0
        self._push_or_update(start)

    def update_grid(self, new_grid: Grid, changed_cells: List[Cell]) -> None:
        if self.grid is None:
            raise ValueError("Planner must be initialized before grid update.")

        self.grid = [row[:] for row in new_grid]

        affected = set()
        for cell in changed_cells:
            affected.add(cell)
            for nb in self._all_neighbors(cell):
                affected.add(nb)

        for cell in affected:
            self._update_vertex(cell)

    def replan(self) -> Dict:
        if self.grid is None or self.start is None or self.goal is None:
            raise ValueError("Planner must be initialized before replanning.")

        t0 = time.perf_counter_ns()
        self.expanded_last = self._compute_shortest_path()
        path = self._extract_path()
        elapsed_us = (time.perf_counter_ns() - t0) / 1000.0

        return {
            "path": path,
            "path_length": max(0, len(path) - 1) if path else float("inf"),
            "planning_time_us": elapsed_us,
            "expanded": self.expanded_last,
        }

    def _compute_shortest_path(self) -> int:
        expanded = 0
        while self._top_key() < self._calculate_key(self.goal) or self.rhs[self.goal] != self.g[self.goal]:
            k_old, u = self._pop_valid()
            if u is None:
                break

            k_new = self._calculate_key(u)
            if k_old < k_new:
                self._push_with_key(u, k_new)
                continue

            expanded += 1
            if self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self._successors(u):
                    self._update_vertex(s)
            else:
                self.g[u] = math.inf
                self._update_vertex(u)
                for s in self._successors(u):
                    self._update_vertex(s)
        return expanded

    def _extract_path(self) -> List[Cell]:
        if self.grid is None or self.start is None or self.goal is None:
            return []
        if self.g[self.goal] == math.inf:
            return []

        current = self.goal
        path = [current]
        guard = len(self.grid) * len(self.grid[0]) + 5

        while current != self.start and guard > 0:
            guard -= 1
            preds = self._predecessors(current)
            if not preds:
                return []
            current = min(preds, key=lambda s: self.g[s] + self._cost(s, current))
            if self.g[current] == math.inf:
                return []
            path.append(current)

        path.reverse()
        return path

    def _update_vertex(self, u: Cell) -> None:
        if self.start is None:
            return
        if not self._is_free(u):
            self.g[u] = math.inf
            self.rhs[u] = math.inf
            self.open_best_key.pop(u, None)
            return

        if u != self.start:
            preds = self._predecessors(u)
            best = math.inf
            for s in preds:
                best = min(best, self.g[s] + self._cost(s, u))
            self.rhs[u] = best

        self.open_best_key.pop(u, None)
        if self.g[u] != self.rhs[u]:
            self._push_or_update(u)

    def _calculate_key(self, s: Cell) -> Tuple[float, float]:
        base = min(self.g[s], self.rhs[s])
        return (base + self._heuristic(s, self.goal), base)

    def _push_or_update(self, s: Cell) -> None:
        self._push_with_key(s, self._calculate_key(s))

    def _push_with_key(self, s: Cell, key: Tuple[float, float]) -> None:
        self.open_best_key[s] = key
        heapq.heappush(self.open_heap, (key[0], key[1], s))

    def _top_key(self) -> Tuple[float, float]:
        while self.open_heap:
            k1, k2, s = self.open_heap[0]
            if self.open_best_key.get(s) == (k1, k2):
                return (k1, k2)
            heapq.heappop(self.open_heap)
        return (math.inf, math.inf)

    def _pop_valid(self):
        while self.open_heap:
            k1, k2, s = heapq.heappop(self.open_heap)
            if self.open_best_key.get(s) == (k1, k2):
                self.open_best_key.pop(s, None)
                return (k1, k2), s
        return (math.inf, math.inf), None

    def _all_neighbors(self, cell: Cell) -> List[Cell]:
        r, c = cell
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        out = []
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if self.grid is not None and 0 <= nr < len(self.grid) and 0 <= nc < len(self.grid[0]):
                out.append((nr, nc))
        return out

    def _successors(self, cell: Cell) -> List[Cell]:
        return [n for n in self._all_neighbors(cell) if self._is_free(n)]

    def _predecessors(self, cell: Cell) -> List[Cell]:
        return [n for n in self._all_neighbors(cell) if self._is_free(n)]

    def _cost(self, a: Cell, b: Cell) -> float:
        return 1.0 if self._is_free(a) and self._is_free(b) else math.inf

    def _heuristic(self, a: Cell, b: Cell) -> float:
        ar, ac = a
        br, bc = b
        if self.allow_diagonal:
            return max(abs(ar - br), abs(ac - bc))
        return abs(ar - br) + abs(ac - bc)

    def _is_free(self, cell: Cell) -> bool:
        if self.grid is None:
            return False
        r, c = cell
        return 0 <= r < len(self.grid) and 0 <= c < len(self.grid[0]) and self.grid[r][c] == 0
