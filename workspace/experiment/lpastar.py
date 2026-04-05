import heapq
from collections import defaultdict
from itertools import count
from math import inf


class LPAStarPlanner:
    def __init__(self):
        self.grid = None
        self.width = 0
        self.height = 0
        self.start = None
        self.goal = None

        self.g = defaultdict(lambda: inf)
        self.rhs = defaultdict(lambda: inf)

        self.open_heap = []
        self.open_entries = {}
        self.counter = count()

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def adjacent_cells(self, node):
        x, y = node
        neighbors = []

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                neighbors.append((nx, ny))

        return neighbors

    def is_free(self, node):
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height and self.grid[y][x] == 0

    def transition_cost(self, a, b):
        if not self.is_free(a) or not self.is_free(b):
            return inf
        return 1.0

    def calculate_key(self, node):
        best = min(self.g[node], self.rhs[node])
        return (best + self.heuristic(node, self.goal), best)

    def remove_from_open(self, node):
        if node in self.open_entries:
            del self.open_entries[node]

    def push_to_open(self, node):
        key = self.calculate_key(node)
        self.open_entries[node] = key
        heapq.heappush(self.open_heap, (key[0], key[1], next(self.counter), node))

    def top_key(self):
        while self.open_heap:
            k1, k2, _, node = self.open_heap[0]
            current_key = self.open_entries.get(node)

            if current_key is None or current_key != (k1, k2):
                heapq.heappop(self.open_heap)
                continue

            return (k1, k2)

        return (inf, inf)

    def pop_smallest(self):
        while self.open_heap:
            k1, k2, _, node = heapq.heappop(self.open_heap)
            current_key = self.open_entries.get(node)

            if current_key is None or current_key != (k1, k2):
                continue

            del self.open_entries[node]
            return node

        return None

    def reset(self, grid, start, goal):
        self.grid = [row[:] for row in grid]
        self.height = len(grid)
        self.width = len(grid[0]) if self.height > 0 else 0
        self.start = start
        self.goal = goal

        self.g = defaultdict(lambda: inf)
        self.rhs = defaultdict(lambda: inf)

        self.open_heap = []
        self.open_entries = {}
        self.counter = count()

        self.rhs[self.start] = 0.0
        self.push_to_open(self.start)

    def update_vertex(self, node):
        if node != self.start:
            if not self.is_free(node):
                self.rhs[node] = inf
            else:
                best_rhs = inf
                for pred in self.adjacent_cells(node):
                    cost = self.transition_cost(pred, node)
                    if cost < inf:
                        best_rhs = min(best_rhs, self.g[pred] + cost)
                self.rhs[node] = best_rhs

        self.remove_from_open(node)

        if self.g[node] != self.rhs[node]:
            self.push_to_open(node)

    def compute_shortest_path(self):
        while (
            self.top_key() < self.calculate_key(self.goal)
            or self.rhs[self.goal] != self.g[self.goal]
        ):
            node = self.pop_smallest()
            if node is None:
                break

            if self.g[node] > self.rhs[node]:
                self.g[node] = self.rhs[node]
                for succ in self.adjacent_cells(node):
                    self.update_vertex(succ)
            else:
                self.g[node] = inf
                self.update_vertex(node)
                for succ in self.adjacent_cells(node):
                    self.update_vertex(succ)

    def detect_changed_cells(self, new_grid):
        changed = []
        for y in range(self.height):
            for x in range(self.width):
                if self.grid[y][x] != new_grid[y][x]:
                    changed.append((x, y))
        return changed

    def apply_grid_changes(self, new_grid, changed_cells):
        self.grid = [row[:] for row in new_grid]

        affected = set()
        for cell in changed_cells:
            affected.add(cell)
            for neighbor in self.adjacent_cells(cell):
                affected.add(neighbor)

        for node in affected:
            self.update_vertex(node)

    def extract_path(self):
        if self.g[self.goal] == inf and self.rhs[self.goal] == inf:
            return []

        current = self.goal
        reverse_path = [current]
        safety_limit = self.width * self.height + 5

        while current != self.start and safety_limit > 0:
            candidates = []

            for pred in self.adjacent_cells(current):
                cost = self.transition_cost(pred, current)
                if cost < inf:
                    candidates.append((self.g[pred] + cost, pred))

            if not candidates:
                return []

            best_cost, best_pred = min(
                candidates,
                key=lambda item: (item[0], item[1][1], item[1][0])
            )

            if best_cost == inf:
                return []

            current = best_pred
            reverse_path.append(current)
            safety_limit -= 1

        if current != self.start:
            return []

        reverse_path.reverse()
        return reverse_path

    def plan(self, grid, start, goal):
        if not grid or not grid[0]:
            return []

        needs_reset = (
            self.grid is None
            or self.start != start
            or self.goal != goal
            or len(grid) != self.height
            or len(grid[0]) != self.width
        )

        if needs_reset:
            self.reset(grid, start, goal)
        else:
            changed_cells = self.detect_changed_cells(grid)
            if changed_cells:
                self.apply_grid_changes(grid, changed_cells)

        self.compute_shortest_path()
        return self.extract_path()
