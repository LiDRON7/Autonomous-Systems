"""A* planning and path simplification on a rolling occupancy grid."""

import math
from heapq import heappop, heappush

from .grid import Cell

NEIGHBORS = tuple(
    (dx, dy)
    for dx in (-1, 0, 1)
    for dy in (-1, 0, 1)
    if (dx, dy) != (0, 0)
)

SEARCH_DIRECTIONS = (
    (1, 0),
    (0, 1),
    (-1, 0),
    (0, -1),
    (1, 1),
    (-1, 1),
    (-1, -1),
    (1, -1),
)


def landing_search_offset(index: int, step_m: float) -> tuple[float, float]:
    """Return successive landing-search offsets in expanding rings."""
    if index < 0 or step_m <= 0.0:
        raise ValueError("index must be non-negative and step_m must be positive")
    direction = SEARCH_DIRECTIONS[index % len(SEARCH_DIRECTIONS)]
    ring = 1 + index // len(SEARCH_DIRECTIONS)
    return direction[0] * step_m * ring, direction[1] * step_m * ring


def astar(start: Cell, goal: Cell, occupied: set[Cell], width: int) -> list[Cell]:
    """Return a lowest-cost 8-connected route or an empty list."""
    if (
        width <= 0
        or any(not (0 <= x < width and 0 <= y < width) for x, y in (start, goal))
        or start in occupied
        or goal in occupied
    ):
        return []
    frontier: list[tuple[float, Cell]] = [(0.0, start)]
    came_from: dict[Cell, Cell | None] = {start: None}
    cost = {start: 0.0}
    while frontier:
        _, current = heappop(frontier)
        if current == goal:
            break
        for dx, dy in NEIGHBORS:
            nxt = current[0] + dx, current[1] + dy
            if not (0 <= nxt[0] < width and 0 <= nxt[1] < width):
                continue
            if nxt in occupied or (
                dx and dy and (
                    (current[0] + dx, current[1]) in occupied
                    or (current[0], current[1] + dy) in occupied
                )
            ):
                continue
            step = math.sqrt(2.0) if dx and dy else 1.0
            candidate = cost[current] + step
            if candidate >= cost.get(nxt, float("inf")):
                continue
            cost[nxt] = candidate
            priority = candidate + math.dist(nxt, goal)
            heappush(frontier, (priority, nxt))
            came_from[nxt] = current
    if goal not in came_from:
        return []
    route = []
    node: Cell | None = goal
    while node is not None:
        route.append(node)
        node = came_from[node]
    return list(reversed(route))


def _line_clear(a: Cell, b: Cell, occupied: set[Cell]) -> bool:
    # Supercover traversal: check every cell touched, including both sides
    # when a segment passes exactly through a grid corner.
    x, y = a
    dx, dy = b[0] - x, b[1] - y
    nx, ny = abs(dx), abs(dy)
    sx = 1 if dx > 0 else -1
    sy = 1 if dy > 0 else -1
    ix = iy = 0
    if a in occupied:
        return False
    while ix < nx or iy < ny:
        decision = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx
        if decision == 0:
            if (x + sx, y) in occupied or (x, y + sy) in occupied:
                return False
            x += sx
            y += sy
            ix += 1
            iy += 1
        elif decision < 0:
            x += sx
            ix += 1
        else:
            y += sy
            iy += 1
        if (x, y) in occupied:
            return False
    return True


def simplify(route: list[Cell], occupied: set[Cell]) -> list[Cell]:
    """Remove intermediate cells while retaining collision-free segments."""
    if any(not _line_clear(a, b, occupied) for a, b in zip(route, route[1:])):
        return []
    if len(route) < 3:
        return [] if route and route[0] in occupied else route
    result = [route[0]]
    anchor = 0
    while anchor < len(route) - 1:
        candidate = len(route) - 1
        while candidate > anchor + 1 and not _line_clear(
            route[anchor], route[candidate], occupied
        ):
            candidate -= 1
        result.append(route[candidate])
        anchor = candidate
    return result
