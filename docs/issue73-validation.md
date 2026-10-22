# Issue 73: contribution boundary and validation plan

## Attribution

This branch starts at PR #79 commit `64dcb3f77c47e939ca4621f9a78a733c9068e7ac`, authored by the original contributor. No original commits are rewritten. Compare new contributions with:

```bash
git log --oneline 64dcb3f..HEAD
git diff 64dcb3f...HEAD
```

Original work: proposed FLU interpretation, height/direction axis correction, horizontal sign correction, diagnostic logging and six isolated grid/A* checks plus live collector. Follow-up work: extract the same processing into a tested function; configurable mount translation; callback/planning/timer regression tests; refresh grid before planning and from timer; valid occupancy-origin quaternion; debug diagnostics; split original checks from live collection; collector QoS/exit status; this runbook. The six original checks are preserved and attributed in their test module.

## Coordinate contract and limits

Incoming XYZ is provisionally FLU (forward/left/up), meters. Sensor and body axes must align; yaw is PX4 local-NED heading. With body FLU point `(f,l,u)`, output is `north = pn + cos(yaw)*f + sin(yaw)*l`, `east = pe + sin(yaw)*f - cos(yaw)*l`. Height band remains **sensor-relative**, inclusive, and forward cutoff remains strictly greater than 0.2 m. It is not ground segmentation.

`map.sensor_translation_flu_m` is the sensor origin relative to vehicle body FLU. Default is zero; hardware mounting must be measured. Simulation uses `[0.13233, 0, 0.26078]`: local x500_depth mount `[0.12,0.03,0.242]` plus internal sensor pose `[0.01233,-0.03,0.01878]`, all rotations zero. Source: Simulation `afdaadb`, PX4 v1.15.0 Gazebo-model submodule `d754381a1cecdd7f17050acd72bf5bf1327bced6`, `models/OakD-Lite/model.sdf`.

The mapper still uses latest pose and yaw only. Roll/pitch, cloud-time pose alignment and mounting rotation are unsupported. Grid/path `map` XY remains local north/east; a Gazebo ENU overlay needs a verified transform. Do not relabel untransformed data or assume a generic world/map TF is correct. None of these assumptions is live-validated yet.

## Offline tests

In the sourced autonomy container, or any Python environment with numpy and pytest:

```bash
PYTHONPATH=src/navigation python3 -m pytest -q src/navigation/test
```

New tests exercise the production mapping function and execute actual MissionNode callback/planning/timer methods with ROS transport fakes. These tests do not instantiate a real ROS node, validate message decoding/QoS at runtime, or prove cloud axes. Original PR checks are now collected by pytest. The live sampler lives in `scripts/issue73_collect.py` and performs no flight commands.

## Simulation preparation

Use the supported Ubuntu environment and adjacent Autonomous-Systems / Simulation checkouts. Record both exact commits, PR parent and follow-up HEAD, image IDs, ROS/Gazebo versions and parameter dump. Keep separate before/after result folders. Stop the autonomy process before changing revisions; do not run two mission controllers. Preserve local work and use separate checkouts for comparisons. Never reset or overwrite contributor branches.

```bash
git rev-parse HEAD
git -C ../Simulation rev-parse HEAD
./scripts/simulation.sh check
./scripts/simulation.sh up
```

This starts the stack but `auto_start` is false. Leave it disarmed for stationary checks. After workspace build, open a shell in `lidron_autonomy` and source:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/px4_ws/install/setup.bash
source /autonomy/install/setup.bash
ros2 param dump /mission > /tmp/issue73-mission.yaml
ros2 topic info /oakd/depth/points --verbose
ros2 topic info /fmu/out/vehicle_local_position --verbose
ros2 service call /autonomy/preflight_check std_srvs/srv/Trigger '{}'
```

Compare main `f6110e1`, original PR `64dcb3f`, and follow-up HEAD under identical obstacle/vehicle poses and parameters. Separate axis-only comparison (translation zero) from mounting-correction comparison (configured translation). Record all differences. Main/PR do not declare the new translation parameter.

## Stationary scene and expected cells

Use one tall box intersecting the camera's +/-1.5 m vertical band, 3 m ahead of the sensor, then a left and right target. Record its world pose and size numerically. Default 1 m crates are unsuitable as the sole target at 5 m cruise altitude. Establish the transform from Gazebo world coordinates to PX4 local NED using a measured pose/heading; do not assume their origins or axes coincide.

For deterministic reference: vehicle local NED `(0,0,-5)`, yaw 0, sensor point FLU `(3,0,0)` and configured mount predict world XY `(3.13233,0)`. At yaw pi/2 predict `(0,3.13233)`. A left point yields negative east at yaw 0. Actual grid quantizes observations and cells; compare against `RollingGrid.to_cell()` and allow resolution/obstacle-surface uncertainty rather than demanding an exact obstacle-center cell.

Confirm actual published frame ID and X/Y/Z values with a controlled object; compare original/new filter retention. Capture callback debug counts, cell positions, and source/pose timestamps. The independent sampler alone does not prove MissionNode received a cloud.

Enable callback logs if needed with the ROS logger service when available, or start the mission node with `--ros-args --log-level mission:=debug` in an isolated launch that replaces the existing node. Do not start a second controller.

Run sampler inside the sourced container from `/autonomy`, supplying actual mission height limits:

```bash
python3 scripts/issue73_collect.py --seconds 30 --min-height=-1.5 --max-height=1.5 --output /tmp/issue73-live.json
```

Exit 0 means required cloud/grid/pose samples and grid dimensions were available; it does **not** mean geometric alignment or avoidance passed. Empty occupancy can be valid in a clear scene. Preserve JSON plus logs and screenshots outside ephemeral container storage.

## Required runtime checks

1. Cloud publishes and MissionNode processes it while valid position is available. Known obstacle survives filtering and occupies expected cells; clear scene does not incorrectly map ground. Repeat at varied height and yaw.
2. Inflation expands the expected obstacle footprint. Observe same cells refreshing beyond 2 seconds while obstacle remains visible.
3. Interrupt only the cloud source under controlled disarmed conditions while clock/position continue. On follow-up branch, occupancy expires after 2 seconds plus one timer period; depth freshness must remain stale. Record HOLD separately for active missions.
4. Translate/rotate vehicle with stationary obstacle. Compare obstacle location before/after grid recentering and before planning between clouds; verify A* uses those cells.
5. RViz: fixed frame `map`, OccupancyGrid `/autonomy/occupancy_grid`, Path `/autonomy/path`; verify frame alignment before overlaying sensor data. Save before/after screenshots with obstacle position and revision recorded.
6. Only after stationary checks, perform the team's controlled simulated mission procedure. Validate trigger -> hold -> replan -> usable route -> progress, including persistent detection. Synthetic A* checks do not prove this. Full attitude/timestamp limitations may require further changes based on evidence.

## Completion evidence

Record confirmed root cause (or explicitly unresolved), source revisions, full parameters, before/after finite/retained point counts and occupied cells, expected/observed obstacle coordinates, inflation and expiry timing, moving-grid results, RViz images and mission-state/path evidence. Repeat relevant checks on merged revision. No simulation or flight acceptance is claimed by offline tests.

## Offline results (October 22, 2026)

Navigation suite: 28 passed in temporary Python 3.13 environment (numpy 2.5.4, pytest 9.1.1). No ROS/Gazebo runtime test performed. Both recenter-before-plan and timer-expiry regressions fail against original PR mission_node at 64dcb3f: occupied index remains (36,30) instead of (34,30) after a 1 m movement; cells remain after 2.01 s without a cloud. The same tests pass with follow-up changes. Synthetic FLU level obstacle (3,0,0) is rejected by main's forward-Z test but retained by the proposed forward-X processing; this only establishes behavior under the FLU assumption.
