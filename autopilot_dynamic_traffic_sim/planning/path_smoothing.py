from __future__ import annotations

from typing import List, Tuple

from planning.grid_map import GridMap

GridCell = Tuple[int, int]


def _bresenham_cells(a: GridCell, b: GridCell) -> List[GridCell]:
    x0, y0 = a[1], a[0]
    x1, y1 = b[1], b[0]

    cells: List[GridCell] = []

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    x, y = x0, y0
    while True:
        cells.append((y, x))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy

    return cells


def has_line_of_sight(a: GridCell, b: GridCell, grid_map: GridMap) -> bool:
    for i, j in _bresenham_cells(a, b):
        if (i, j) == a or (i, j) == b:
            continue
        if not grid_map.is_free(i, j):
            return False
    return True


def smooth_path(path: List[GridCell], grid_map: GridMap, max_skip: int = 40) -> List[GridCell]:
    if len(path) <= 2:
        return path

    out: List[GridCell] = [path[0]]
    i = 0
    n = len(path)
    while i < n - 1:
        best = i + 1
        limit = min(n - 1, i + max_skip)
        for j in range(limit, i, -1):
            if has_line_of_sight(path[i], path[j], grid_map):
                best = j
                break
        out.append(path[best])
        i = best

    return out
