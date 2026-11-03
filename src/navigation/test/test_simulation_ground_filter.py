from pathlib import Path

import numpy as np

from navigation.grid import RollingGrid
from navigation.mapping import depth_points_to_world


def simulation_height_bounds():
    config = (
        Path(__file__).parents[2]
        / "bringup"
        / "config"
        / "simulation.yaml"
    )
    values = {}

    for line in config.read_text().splitlines():
        stripped = line.strip()
        for name in ("map.min_height_m", "map.max_height_m"):
            if stripped.startswith(f"{name}:"):
                values[name] = float(stripped.split(":", 1)[1])

    return values["map.min_height_m"], values["map.max_height_m"]


def occupied_from(raw):
    minimum, maximum = simulation_height_bounds()
    world, valid_count = depth_points_to_world(
        np.asarray(raw),
        position=(0.0, 0.0, 0.0),
        yaw=0.0,
        min_height=minimum,
        max_height=maximum,
        sensor_translation=(0.13233, 0.0, 0.26078),
    )

    grid = RollingGrid(
        resolution=0.5,
        size_m=30.0,
        inflation_m=1.0,
        expiry_s=2.0,
    )
    origin = grid.origin_around((0.0, 0.0))
    grid.observe(world, origin, now_s=0.0)
    return valid_count, world, grid.occupied


def test_simulation_ground_plane_does_not_create_occupied_cells():
    ground = [
        [3.0, -1.0, -0.24778],
        [3.0, 0.0, -0.24778],
        [3.0, 1.0, -0.24778],
    ]

    valid_count, world, occupied = occupied_from(ground)

    assert valid_count == 3
    assert world == []
    assert occupied == {}


def test_raised_obstacle_remains_in_occupancy_grid():
    ground_and_obstacle = [
        [3.0, -1.0, -0.24778],
        [3.0, 0.0, -0.24778],
        [3.0, 1.0, -0.24778],
        [3.0, 0.0, 0.0],
        [3.0, 0.0, 0.74726],
    ]

    valid_count, world, occupied = occupied_from(ground_and_obstacle)

    assert valid_count == 5
    assert len(world) == 2
    assert occupied