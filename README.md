# LiDRON Autonomous Systems

This code flies the drone on its own. It maps obstacles with the OAK-D camera, plans a path with A*, sends the flight commands to PX4 and checks the landing spot with the LiDAR. The same container runs in simulation and on the drone's Raspberry Pi 5 (ROS 2 Jazzy, PX4 v1.15).

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/architecture/data-flow-dark.png">
  <img alt="How data moves from the sensors to PX4" src="docs/architecture/data-flow-light.png">
</picture>

## Run it on the drone

1. Start the sensor containers from [Sensor-Control-Data-Gathering](https://github.com/LiDRON7/Sensor-Control-Data-Gathering).
2. Tell the Pi which drone it is, once: `echo DRONE=widow > integration/.env` (or `DRONE=huntsman`). It refuses to start without it.
3. Start autonomy:

   ```bash
   docker compose -f integration/compose.hardware.yml up -d --build
   ```

It loads `src/bringup/config/hardware.yaml`, then that drone's own numbers (`widow.yaml` or `huntsman.yaml`: camera position, LiDAR mount), and waits. Nothing flies until you start a mission. The first build on the Pi is slow because it compiles the PX4 messages.

`hardware.yaml` sets `require_lidar: false`, so the drone lands at the destination without checking the ground. Widow has no LiDAR, so it stays that way. For Huntsman, change it to `true` once the LiDAR is mounted and its cloud looks right in RViz.

## Run it in simulation

Clone [Simulation](https://github.com/LiDRON7/Simulation) next to this repo, then:

```bash
./scripts/simulation.sh up
```

`./scripts/sim_scenarios.sh` then flies every way a mission can end (A to B, around a tree, abort, pause, pilot takeover, a bad landing spot, crashes) and says PASS or FAIL; results are in the Simulation README. The simulator has no RC transmitter, so `simulation.yaml` sets `require_rc: false` and the preflight check skips the RC link there. On the drone it stays on. `./scripts/simulation.sh down` stops it. `DRONE=` in `Simulation/.env` picks the drone for both the simulator and the autonomy config. Machine setup (Linux, macOS, Windows) is in the Simulation README.

## Fly a mission

```bash
docker exec -it lidron_autonomy bash
source /autonomy/install/setup.bash
```

1. Check input freshness, PX4 readiness, disarmed state, battery, failsafe flags and GPS. Only fly if it says `success: true`.
   `ros2 service call /autonomy/preflight_check std_srvs/srv/Trigger {}`
2. Set where to go, here 8 m north:
   `ros2 service call /autonomy/set_local_destination interfaces/srv/SetLocalDestination '{north: 8.0, east: 0.0, down: -5.0}'`
   The drone flies at `cruise_altitude_m` above where it took off. `down` (negative is up) only counts when it asks for more height than that, so a destination at ground level does not fly the drone into the ground.
3. Start within 30 seconds of a passing check (repeat step 1 if it expired):
   `ros2 service call /autonomy/enable std_srvs/srv/SetBool '{data: true}'`
4. Watch it:
   `ros2 topic echo /autonomy/state`
5. Stop and land if anything looks wrong:
   `ros2 service call /autonomy/abort std_srvs/srv/Trigger {}`

The pilot can take over at any time with the RC transmitter, or switch to Land, Hold or RTL in QGroundControl. Any mode change away from Offboard stops the autonomy, and so does a PX4 failsafe: it sends nothing more until the drone is disarmed and you start a new mission.

## How it decides

It only starts after a preflight check that passed in the last 30 seconds: every input fresh, PX4's own checks passed, disarmed, battery fine, no failsafe flags, and a GPS 3D fix with at least 8 satellites. Then it climbs to 5 m above the pad and flies toward the goal on an A* path through the camera's obstacle map, replanning when something shows up. At the goal Huntsman checks the ground with the LiDAR (flat, smooth, nothing sticking up, no drop-off like a crate edge). If the spot is unsafe it tries up to 8 nearby spots, and if none is safe it lands where it is. If any input goes stale it hovers, and after the hold timeout it lands.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/architecture/algorithm-logic-dark.png">
  <img alt="Mission decision loop and landing check" src="docs/architecture/algorithm-logic-light.png">
</picture>

## In the simulator

These come from the runs on my Ubuntu PC (RTX 3060 Ti, real time, camera and LiDAR on). Every scenario and its result is in the [Simulation README](https://github.com/LiDRON7/Simulation#mission-tests).

| | |
| --- | --- |
| <img src="docs/img/114-takeover-path.png" width="420" alt="Flight path when the pilot takes over mid-mission"> | The pilot switches PX4 to Hold mid-mission. The autonomy stands down and doesn't take the drone back (#114). |
| <img src="docs/img/119-cruise-after.png" width="420" alt="Height over a mission with a ground-level destination"> | A destination at ground level: it still cruises 5 m above the pad and only comes down at the goal (#119). |
| <img src="docs/img/121-guards-after.png" width="420" alt="Obstacle map without Huntsman's own prop guards"> | Huntsman's camera sees its own prop guards. With `map.min_range_m` they stay out of the map, so the planner finds its way (#121). |
| <img src="docs/img/crate-after.png" width="420" alt="Huntsman refusing to land on a crate"> | Sent onto a 1 m crate, Huntsman refuses the lid and lands next to it (#124). |
| <img src="docs/img/avoid-rate.png" width="420" alt="How close each drone came to the tree over every run"> | How close each drone came to the tree over every avoid run (#126). |

## More

- [Connect to the Pi](https://github.com/LiDRON7/Sensor-Control-Data-Gathering/blob/main/docs/connect-to-pi.md): how to SSH into the drone's Pi, at the university or in the field.
- [Pi setup](https://github.com/LiDRON7/Sensor-Control-Data-Gathering/blob/main/docs/pi-setup.md): wire the drone's Pi, start the containers and check everything works.
- [Pi known issues](https://github.com/LiDRON7/Sensor-Control-Data-Gathering/blob/main/docs/pi-known-issues.md): every problem we hit on the Pi, why, and the fix.
- [docs/topics.md](docs/topics.md): every topic and service, who sends it and who uses it.
- [docs/reference.md](docs/reference.md): parameters, tests and troubleshooting.
- `docs/architecture/`: the diagrams. Open the `.excalidraw` files at [excalidraw.com](https://excalidraw.com) to edit them.
