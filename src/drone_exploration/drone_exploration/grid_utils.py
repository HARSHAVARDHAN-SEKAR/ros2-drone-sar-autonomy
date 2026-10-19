"""Pure occupancy-grid utilities used by the aerial frontier explorer."""

from __future__ import annotations

import heapq
import math
from collections import deque
from typing import Iterable

import numpy as np

UNKNOWN = -1
FREE_MAX = 20
OCCUPIED_MIN = 65


def world_to_grid(x: float, y: float, origin_x: float, origin_y: float, resolution: float):
    if resolution <= 0.0:
        raise ValueError('resolution must be positive')
    return int(math.floor((x - origin_x) / resolution)), int(math.floor((y - origin_y) / resolution))


def grid_to_world(gx: int, gy: int, origin_x: float, origin_y: float, resolution: float):
    return (
        origin_x + (gx + 0.5) * resolution,
        origin_y + (gy + 0.5) * resolution,
    )


def in_bounds(grid: np.ndarray, cell: tuple[int, int]) -> bool:
    x, y = cell
    return 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]


def inflate_obstacles(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    """Return a boolean blocked mask with occupied/uncertain cells inflated."""
    occupied = grid >= OCCUPIED_MIN
    if radius_cells <= 0:
        return occupied
    blocked = occupied.copy()
    ys, xs = np.nonzero(occupied)
    r2 = radius_cells * radius_cells
    for y, x in zip(ys.tolist(), xs.tolist()):
        y0, y1 = max(0, y - radius_cells), min(grid.shape[0], y + radius_cells + 1)
        x0, x1 = max(0, x - radius_cells), min(grid.shape[1], x + radius_cells + 1)
        for yy in range(y0, y1):
            dy2 = (yy - y) ** 2
            for xx in range(x0, x1):
                if (xx - x) ** 2 + dy2 <= r2:
                    blocked[yy, xx] = True
    return blocked


def frontier_mask(grid: np.ndarray) -> np.ndarray:
    """Free cells that touch at least one unknown 4-neighbour."""
    free = (grid >= 0) & (grid <= FREE_MAX)
    unknown = grid == UNKNOWN
    adjacent_unknown = np.zeros_like(free, dtype=bool)
    adjacent_unknown[1:, :] |= unknown[:-1, :]
    adjacent_unknown[:-1, :] |= unknown[1:, :]
    adjacent_unknown[:, 1:] |= unknown[:, :-1]
    adjacent_unknown[:, :-1] |= unknown[:, 1:]
    return free & adjacent_unknown


def cluster_mask(mask: np.ndarray, min_size: int = 4) -> list[list[tuple[int, int]]]:
    seen = np.zeros_like(mask, dtype=bool)
    clusters: list[list[tuple[int, int]]] = []
    h, w = mask.shape
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or seen[y, x]:
                continue
            q = deque([(x, y)])
            seen[y, x] = True
            cluster: list[tuple[int, int]] = []
            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for nx, ny in ((cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)):
                    if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] and not seen[ny, nx]:
                        seen[ny, nx] = True
                        q.append((nx, ny))
            if len(cluster) >= max(1, min_size):
                clusters.append(cluster)
    return clusters


def nearest_free_cell(grid: np.ndarray, start: tuple[int, int], blocked: np.ndarray | None = None):
    """Breadth-first search for a traversable known-free cell."""
    if not in_bounds(grid, start):
        return None
    if blocked is None:
        blocked = np.zeros_like(grid, dtype=bool)
    q = deque([start])
    seen = {start}
    while q:
        x, y = q.popleft()
        if 0 <= grid[y, x] <= FREE_MAX and not blocked[y, x]:
            return (x, y)
        for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if in_bounds(grid, n) and n not in seen:
                seen.add(n)
                q.append(n)
    return None


def astar(
    grid: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    blocked: np.ndarray | None = None,
    max_expansions: int = 80000,
) -> list[tuple[int, int]]:
    """8-connected A* through known free cells. Unknown space is never traversed."""
    if blocked is None:
        blocked = np.zeros_like(grid, dtype=bool)
    if not in_bounds(grid, start) or not in_bounds(grid, goal):
        return []
    if blocked[start[1], start[0]] or blocked[goal[1], goal[0]]:
        return []
    if not (0 <= grid[start[1], start[0]] <= FREE_MAX):
        return []
    if not (0 <= grid[goal[1], goal[0]] <= FREE_MAX):
        return []

    def heuristic(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    neighbours = [
        (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
    ]
    open_heap = [(heuristic(start, goal), 0.0, start)]
    g_cost = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    closed = set()
    expansions = 0

    while open_heap and expansions < max_expansions:
        _, current_g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in parent:
                current = parent[current]
                path.append(current)
            path.reverse()
            return path
        closed.add(current)
        expansions += 1
        cx, cy = current
        for dx, dy, step in neighbours:
            nxt = (cx + dx, cy + dy)
            if not in_bounds(grid, nxt):
                continue
            nx, ny = nxt
            if blocked[ny, nx] or not (0 <= grid[ny, nx] <= FREE_MAX):
                continue
            if dx and dy:
                # No diagonal corner-cutting through inflated obstacles.
                if blocked[cy, nx] or blocked[ny, cx]:
                    continue
            tentative = current_g + step
            if tentative < g_cost.get(nxt, float('inf')):
                g_cost[nxt] = tentative
                parent[nxt] = current
                heapq.heappush(open_heap, (tentative + heuristic(nxt, goal), tentative, nxt))
    return []


def choose_frontier_goal(
    clusters: Iterable[list[tuple[int, int]]],
    start: tuple[int, int],
    min_distance_cells: float = 3.0,
):
    """Prefer large frontiers while mildly penalizing travel distance."""
    best = None
    best_score = -float('inf')
    for cluster in clusters:
        if not cluster:
            continue
        # Pick the cluster cell closest to its centroid, not an unknown centroid coordinate.
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        candidate = min(cluster, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        distance = math.hypot(candidate[0] - start[0], candidate[1] - start[1])
        if distance < min_distance_cells:
            continue
        score = len(cluster) / (1.0 + 0.35 * distance)
        if score > best_score:
            best_score = score
            best = candidate
    return best
