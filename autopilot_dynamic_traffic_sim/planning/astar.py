from __future__ import annotations

import heapq
from typing import Dict, List, Optional, Tuple

from planning.grid_map import GridMap

GridCell = Tuple[int, int]


def _manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start: GridCell, goal: GridCell, grid_map: GridMap) -> List[GridCell]:
    if start == goal:
        return [start]

    if not grid_map.is_free(*start) or not grid_map.is_free(*goal):
        return []

    open_heap: List[Tuple[int, int, GridCell]] = []
    heapq.heappush(open_heap, (_manhattan(start, goal), 0, start))

    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, int] = {start: 0}

    closed: set[GridCell] = set()

    while open_heap:
        _, current_g, current = heapq.heappop(open_heap)

        if current in closed:
            continue
        closed.add(current)

        if current == goal:
            return _reconstruct_path(came_from, current)

        i, j = current
        for ni, nj in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1)):
            neighbor = (ni, nj)
            if not grid_map.is_free(ni, nj):
                continue
            if neighbor in closed:
                continue

            tentative_g = current_g + 1
            prev_g = g_score.get(neighbor)
            if prev_g is None or tentative_g < prev_g:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + _manhattan(neighbor, goal)
                heapq.heappush(open_heap, (f, tentative_g, neighbor))

    return []


def _reconstruct_path(came_from: Dict[GridCell, GridCell], current: GridCell) -> List[GridCell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path
