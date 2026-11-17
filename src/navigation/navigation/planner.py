"""A* planning and path simplification on a rolling occupancy grid."""

import math
from collections.abc import Iterable
from heapq import heappop, heappush

from .grid import Cell, Point, RollingGrid

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


def escape_leg_clear(
    start: Point, target: Point, obstacles: Iterable[Point], grid: RollingGrid,
) -> bool:
    """Check the actual segment against measured obstacles before leaving inflation."""
    dx, dy = target[0] - start[0], target[1] - start[1]
    length_squared = dx * dx + dy * dy
    cell_radius = grid.resolution / math.sqrt(2.0)
    exit_clearance = max(grid.inflation_m, cell_radius)
    for obstacle in obstacles:
        clearance = math.dist(start, obstacle)
        if clearance < cell_radius or math.dist(target, obstacle) < exit_clearance:
            return False
        if length_squared:
            ox, oy = obstacle[0] - start[0], obstacle[1] - start[1]
            fraction = max(0.0, min(1.0, (ox * dx + oy * dy) / length_squared))
            closest = start[0] + fraction * dx, start[1] + fraction * dy
            if math.dist(closest, obstacle) < min(clearance, exit_clearance):
                return False
    return True


def find_escape_target(start: Point, origin: Point, grid: RollingGrid) -> Point | None:
    """Find a nearby free cell reachable by a straight segment away from obstacles."""
    start_x, start_y = grid.to_cell(start, origin)
    if any(grid.to_cell(point, origin) == (start_x, start_y) for point in grid.observations):
        return None
    radius = math.ceil(grid.inflation_m / grid.resolution) + 1
    candidates = [
        grid.to_world((x, y), origin)
        for x in range(start_x - radius, start_x + radius + 1)
        for y in range(start_y - radius, start_y + radius + 1)
        if grid.in_bounds((x, y)) and (x, y) not in grid.occupied
    ]
    return next(
        (
            target for target in sorted(candidates, key=lambda point: math.dist(start, point))
            if escape_leg_clear(start, target, grid.observations, grid)
        ),
        None,
    )


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
