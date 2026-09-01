"""Rolling occupancy grid built from forward depth points."""

from dataclasses import dataclass, field
import math


Cell = tuple[int, int]
Point = tuple[float, float]


@dataclass
class RollingGrid:
    resolution: float = 0.5
    size_m: float = 30.0
    inflation_m: float = 1.0
    expiry_s: float = 2.0
    occupied: dict[Cell, float] = field(default_factory=dict)

    @property
    def width(self) -> int:
        return max(3, int(math.ceil(self.size_m / self.resolution)))

    def to_cell(self, point: Point, origin: Point) -> Cell:
        return (
            int(math.floor((point[0] - origin[0]) / self.resolution)),
            int(math.floor((point[1] - origin[1]) / self.resolution)),
        )

    def to_world(self, cell: Cell, origin: Point) -> Point:
        return (
            origin[0] + (cell[0] + 0.5) * self.resolution,
            origin[1] + (cell[1] + 0.5) * self.resolution,
        )

    def origin_around(self, center: Point) -> Point:
        half = self.width * self.resolution / 2.0
        return center[0] - half, center[1] - half

    def observe(self, points: list[Point], origin: Point, now_s: float) -> None:
        radius = int(math.ceil(self.inflation_m / self.resolution))
        for point in points:
            cell = self.to_cell(point, origin)
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if math.hypot(dx, dy) * self.resolution <= self.inflation_m:
                        inflated = cell[0] + dx, cell[1] + dy
                        if self.in_bounds(inflated):
                            self.occupied[inflated] = now_s
        self.expire(now_s)

    def expire(self, now_s: float) -> None:
        self.occupied = {
            cell: stamp
            for cell, stamp in self.occupied.items()
            if now_s - stamp <= self.expiry_s
        }

    def in_bounds(self, cell: Cell) -> bool:
        return 0 <= cell[0] < self.width and 0 <= cell[1] < self.width
