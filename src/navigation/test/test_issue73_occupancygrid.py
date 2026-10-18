#!/usr/bin/env python3
"""Issue #73: non-invasive, simulation-only OAK-D / occupancy-grid diagnostics.

Run in the sourced lidron_autonomy container with /autonomy/src/navigation available.
Does not move/arm the drone or modify navigation parameters.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

# Import the actual repository implementation, not a reimplementation of RollingGrid.
sys.path.insert(0, '/autonomy/src/navigation')
try:
    from navigation.grid import RollingGrid
    from navigation.planner import astar
except ImportError as exc:
    raise SystemExit(f'Cannot import navigation package: {exc}. Check build/mount path.')


def isolated_tests():
    checks = {}
    g = RollingGrid(resolution=0.5, size_m=30.0, inflation_m=1.0, expiry_s=2.0)
    origin = (0.0, 0.0)
    g.observe([(5.0, 5.0)], origin, 10.0)
    center = g.to_cell((5.0, 5.0), origin)
    checks['grid_insertion'] = center in g.occupied
    # A 1 m Euclidean-radius inflation at 0.5 m resolution has 13 cell offsets.
    checks['inflation'] = len(g.occupied) == 13
    initial = set(g.occupied)
    g.observe([], origin, 11.5)
    checks['persist_before_expiry'] = set(g.occupied) == initial
    g.observe([], origin, 12.1)
    checks['expire_after_timeout'] = not g.occupied
    g.observe([(5.0, 5.0)], origin, 20.0)
    g.observe([(5.0, 5.0)], origin, 21.0)
    g.observe([], origin, 22.5)
    checks['refresh_extends_lifetime'] = bool(g.occupied)
    # Check that the real A* function treats occupied cells as blocked.
    obstacles = {(6, y) for y in range(3, 9)}
    try:
        route = astar((2, 5), (11, 5), obstacles, 20)
        checks['astar_uses_blocked_cells'] = bool(route) and not any(
            tuple(c) in obstacles for c in route
        )
    except Exception as exc:
        checks['astar_uses_blocked_cells'] = False
        checks['astar_exception'] = str(exc)
    return checks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', default='/tmp/issue73_validation.json')
    parser.add_argument('--min-height', type=float, default=-1.5)
    parser.add_argument('--max-height', type=float, default=1.5)
    args = parser.parse_args()
    checks = isolated_tests()
    print('=== Deterministic checks (actual RollingGrid + A*) ===', flush=True)
    for key, result in checks.items():
        print(f'{key}: {result}', flush=True)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from nav_msgs.msg import OccupancyGrid
    from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
    from std_msgs.msg import Bool

    class Collector(Node):
        def __init__(self):
            super().__init__('issue73_validation')
            self.cloud_count = 0
            self.grid_count = 0
            self.obstacle_true = 0
            self.obstacle_false = 0
            self.position = None
            self.yaw = None
            self.cloud_sample = None
            self.grid_sample = None
            self.frame_ids = set()
            self.last_cloud_wall = None
            self.cloud_intervals = []
            self.last_grid_wall = None
            self.grid_intervals = []
            self.grid_occupied_counts = []
            self.grid_data_valid = True
            self.create_subscription(PointCloud2, '/oakd/depth/points', self.on_cloud, 10)
            self.create_subscription(OccupancyGrid, '/autonomy/occupancy_grid', self.on_grid, 10)
            self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_position, 10)            
            self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude, 10)
            self.create_subscription(Bool, '/perception/obstacle_detected', self.on_obstacle, 10)

        def on_cloud(self, msg):
            wall = time.monotonic()
            if self.last_cloud_wall is not None:
                self.cloud_intervals.append(wall - self.last_cloud_wall)
            self.last_cloud_wall = wall
            self.cloud_count += 1
            self.frame_ids.add(msg.header.frame_id)
            # Retain only most recent sample, avoid decoding each very large cloud.
            self.cloud_sample = msg

        def on_grid(self, msg):
            wall = time.monotonic()
            if self.last_grid_wall is not None:
                self.grid_intervals.append(wall - self.last_grid_wall)
            self.last_grid_wall = wall
            self.grid_count += 1
            self.grid_data_valid &= len(msg.data) == msg.info.width * msg.info.height
            count = sum(v > 0 for v in msg.data)
            self.grid_occupied_counts.append(count)
            self.grid_sample = msg

        def on_position(self, msg):
            if bool(msg.xy_valid) and bool(msg.z_valid):
                self.position = [float(msg.x), float(msg.y), float(msg.z)]

        def on_attitude(self, msg):
            w, x, y, z = (float(v) for v in msg.q)
            self.yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

        def on_obstacle(self, msg):
            if msg.data:
                self.obstacle_true += 1
            else:
                self.obstacle_false += 1

    rclpy.init()
    node = Collector()
    print(f'\n=== Live ROS sample ({args.seconds:g}s) ===', flush=True)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.3)
    except KeyboardInterrupt:
        print('Interrupted; saving partial results.', flush=True)

    report = {
        'unit_checks': checks,
        'live': {
            'cloud_messages': node.cloud_count,
            'grid_messages': node.grid_count,
            'cloud_frame_ids': sorted(node.frame_ids),
            'cloud_max_interval_s': max(node.cloud_intervals, default=None),
            'grid_max_interval_s': max(node.grid_intervals, default=None),
            'obstacle_detected_true_messages': node.obstacle_true,
            'obstacle_detected_false_messages': node.obstacle_false,
            'px4_position_valid_sample': node.position,
            'px4_yaw_rad_sample': node.yaw,
            'grid_dimensions_valid': node.grid_data_valid,
            'occupied_cells_min': min(node.grid_occupied_counts, default=None),
            'occupied_cells_max': max(node.grid_occupied_counts, default=None),
            'occupied_cells_latest': node.grid_occupied_counts[-1] if node.grid_occupied_counts else None,
            'occupied_cell_counts_by_message': node.grid_occupied_counts,
        },
        'limitations': [
            'Point ranges alone cannot prove FLU sensor-frame convention or mounting transformation.',
            'Nonzero grid cells alone do not prove alignment with the physical obstacle.',
            'An independent sampler cannot prove MissionNode callback execution or that all points reach observe().',
            'Repeated occupied-cell counts do not prove the same cells refresh; deterministic expiry is tested separately.',
            'Synthetic A* test does not prove active-mission replanning.',
            'No RViz verification is performed.',
        ],
    }
    if node.cloud_sample is not None:
        raw = point_cloud2.read_points_numpy(node.cloud_sample, field_names=('x','y','z'))
        pts = np.asarray(raw, dtype=float).reshape((-1, 3))
        pts = pts[np.isfinite(pts).all(axis=1)]
        original = ((pts[:,1] >= args.min_height) & (pts[:,1] <= args.max_height) & (pts[:,2] > 0.2))
        proposed = ((pts[:,2] >= args.min_height) & (pts[:,2] <= args.max_height) & (pts[:,0] > 0.2))
        report['live']['cloud_sample'] = {
            'valid_xyz_points': len(pts),
            'original_filter_retained': int(original.sum()),
            'proposed_flu_filter_retained': int(proposed.sum()),
            'xyz_min': pts.min(axis=0).tolist() if len(pts) else None,
            'xyz_max': pts.max(axis=0).tolist() if len(pts) else None,
        }
    if node.grid_sample is not None:
        msg = node.grid_sample
        report['live']['grid'] = {
            'frame_id': msg.header.frame_id,
            'resolution_m': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin_xy': [msg.info.origin.position.x, msg.info.origin.position.y],
            'occupied_cell_coordinates': [
                [i % msg.info.width, i // msg.info.width]
                for i, val in enumerate(msg.data) if val > 0
            ],
        }
    node.destroy_node()
    rclpy.shutdown()
    Path(args.output).write_text(json.dumps(report, indent=2))
    print(json.dumps({'unit_checks': checks, 'live': {k:v for k,v in report['live'].items() if k not in ('occupied_cell_counts_by_message','grid')}}, indent=2), flush=True)
    print(f'\nFull evidence saved: {args.output}', flush=True)
    if node.cloud_count == 0 or node.grid_count == 0:
        print('WARNING: missing live topic messages; do not claim live integration passed.', flush=True)

if __name__ == '__main__':
    main()
