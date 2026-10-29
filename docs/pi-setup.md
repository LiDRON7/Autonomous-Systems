# Set up the drone's Pi

How to wire everything to the Raspberry Pi 5 (`lidron-cranberry`), start the containers, and check each part works. Do the sections in order.

To get into the Pi, see [connect-to-pi.md](connect-to-pi.md).

If something that used to work stops working, read [pi-known-issues.md](https://github.com/LiDRON7/Sensor-Control-Data-Gathering/blob/flight-test/docs/pi-known-issues.md) first. It lists every problem we hit and how to fix it.

## 1. Connect the hardware

| Part | Plug it into | Used by |
| --- | --- | --- |
| Pixhawk USB-C | any Pi USB port | `airborne-bridge` (MAVLink to the radio) |
| Pixhawk TELEM2 | Pi GPIO pins 10, 8 and 6 (see below) | `px4-agent` (autonomy talks to PX4) |
| SiK radio | any Pi USB port | `airborne-bridge` |
| OAK-D Pro | a **blue** USB 3 port on the Pi | `oak-d` |
| H-RTK GPS (10-pin plug) | Pixhawk **GPS1**, not the Pi | PX4 |

TELEM2 uses 3 of its 6 wires. Count from pin 1, the red wire.

The data wires cross: the Pixhawk's TX goes to the Pi's RX, and the Pixhawk's RX goes to the Pi's TX. RX to RX gets no data at all. Turn the Pi off before you touch its pins.

| TELEM2 wire | Pi pin |
| --- | --- |
| 1 (5 V, red) | nothing, tape it off |
| 2 (Pixhawk TX, white) | 10 (Pi RXD) |
| 3 (Pixhawk RX, yellow) | 8 (Pi TXD) |
| 6 (GND, black) | 6 (GND) |

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/LiDRON7/Sensor-Control-Data-Gathering/flight-test/docs/architecture/telem2-wiring-dark.png">
  <img alt="TELEM2 plug wires 2, 3 and 6 going to Pi header pins 10, 8 and 6, drawn in the real orientation" src="https://raw.githubusercontent.com/LiDRON7/Sensor-Control-Data-Gathering/flight-test/docs/architecture/telem2-wiring-light.png">
</picture>

On the Pi, all three are in the outer row (along the board edge): the 3rd, 4th and 5th pin from the end with pin 1.

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="https://raw.githubusercontent.com/LiDRON7/Sensor-Control-Data-Gathering/flight-test/docs/architecture/connections-dark.png">
  <img alt="How every part connects, with the TELEM2 to Pi wiring at the bottom" src="https://raw.githubusercontent.com/LiDRON7/Sensor-Control-Data-Gathering/flight-test/docs/architecture/connections-light.png">
</picture>

## 2. Set up the Pi (once per Pi)

`lidron-cranberry` already has all of this. Only do it on a new Pi.

1. Let your user run Docker and use the serial ports, then log out and back in:

   ```bash
   sudo usermod -aG docker,dialout $USER
   ```

2. Let the camera work without root:

   ```bash
   echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

3. Check the UART exists: `ls /dev/ttyAMA0`. If it doesn't, add `dtparam=uart0=on` to `/boot/firmware/config.txt` and reboot.

4. Stop Linux from using the UART as a login console. Otherwise it swallows everything PX4 sends:

   ```bash
   sudo systemctl mask --now serial-getty@ttyAMA0.service
   sudo sed -i "s/console=serial0,115200 //" /boot/firmware/current/cmdline.txt
   sudo reboot
   ```

5. Get both repos on the `flight-test` branch, in `~/LiDRON/`.

## 3. Set up PX4 (once per Pixhawk)

1. In QGroundControl, flash **PX4 1.15.x**. Not 1.16 or newer: they rename topics and autonomy gets no data.
2. Calibrate the sensors, the radio and the flight modes.
3. Set these parameters, then reboot (Vehicle Setup → Parameters → Tools → Reboot Vehicle):

   | Parameter | Value |
   | --- | --- |
   | `UXRCE_DDS_CFG` | `102` (TELEM2) |
   | `SER_TEL2_BAUD` | `921600` |
   | `MAV_1_CONFIG` | `0` |

   Set them in QGroundControl, not with a pymavlink script. A script that sends the number the wrong way saves `UXRCE_DDS_CFG` as `1120665600`, even though reading it back over MAVLink still shows `102`. PX4 then never starts the DDS client on TELEM2.

4. After the reboot, check it in the MAVLink Console (Analyze Tools → MAVLink Console):

   ```text
   param show UXRCE_DDS_CFG
   uxrce_dds_client status
   ```

   The first must print exactly `102`, and the second `Running`. If it says `not running`, set `UXRCE_DDS_CFG` again and reboot.

## 4. Start everything

1. Find the USB names: `ls /dev/serial/by-id/`

2. Create `~/LiDRON/Sensor-Control-Data-Gathering/.env` with them:

   ```dotenv
   PIXHAWK_DEVICE=/dev/serial/by-id/usb-Auterion_PX4_FMU_v6X.x_0-if00
   RADIO_DEVICE=/dev/serial/by-id/usb-FTDI_FT231X_USB_UART_<your radio>-if00-port0
   PX4_AGENT_DEVICE=/dev/ttyAMA0
   SERIAL_GID=20
   ROS_DOMAIN_ID=0
   ```

3. Start the sensor containers:

   ```bash
   cd ~/LiDRON/Sensor-Control-Data-Gathering
   docker compose up -d --build airborne-bridge px4-agent oak-d
   ```

4. Start autonomy:

   ```bash
   cd ~/LiDRON/Autonomous-Systems
   docker compose -f integration/compose.hardware.yml up -d --build
   ```

The first build takes about an hour on the Pi. After that, leave out `--build` and they start in seconds. They also start by themselves after a reboot.

## 5. Check it works

Checks 3, 5 and 6 run inside the autonomy container. To save typing, paste this line into the Pi's terminal once (each time you log in). It prints nothing, it just creates a shortcut called `auto`:

```bash
auto() { docker exec lidron_autonomy bash -c "source /opt/ros/jazzy/setup.bash && source /autonomy/install/setup.bash && $*"; }
```

Then put `auto` in front of any `ros2` command:

```bash
auto ros2 topic list
auto ros2 topic hz /fmu/out/vehicle_status
```

Press `Ctrl+C` to stop commands that keep running, like `topic hz`. Don't run `auto` alone or the `docker exec ...` part by itself: it fails with `syntax error: unexpected end of file`.

To work inside the container instead, run `docker exec -it lidron_autonomy bash`, then `source /autonomy/install/setup.bash`, and type `ros2` commands without `auto`. Type `exit` to leave.

| # | Check | Command | Good result |
| --- | --- | --- | --- |
| 1 | Pixhawk and radio | `docker logs --tail 5 airborne-bridge` | `Opened pixhawk`, `Opened radio`, then `Forwarded ...` every 5 s |
| 2 | TELEM2 link | `docker logs px4-agent` | a `create_client` line |
| 3 | PX4 reaches autonomy | `auto ros2 topic hz /fmu/out/vehicle_status` | a steady rate |
| 4 | Camera reaches autonomy | `auto ros2 topic hz /oakd/depth/points` | about 13 Hz |
| 5 | Obstacle detection | `auto ros2 topic echo --once /perception/obstacle_distance` | your hand's distance in meters |
| 6 | Everything at once | `auto ros2 service call /autonomy/preflight_check std_srvs/srv/Trigger` | `success=True` |
| 7 | Ground station | change the flight mode in QGroundControl over the radio | the mode changes, and `uplinked` goes up in check 1 |

When all 7 pass, do the props-off dry run ([Autonomous-Systems #84](https://github.com/LiDRON7/Autonomous-Systems/issues/84)).

Indoors, the GPS has no fix, so check 6 lists `gps` as stale. Run it outside.

## If something fails

Each of these is explained in more detail in [pi-known-issues.md](https://github.com/LiDRON7/Sensor-Control-Data-Gathering/blob/flight-test/docs/pi-known-issues.md).

| Problem | Fix |
| --- | --- |
| `px4-agent` has no `create_client` | 1) `uxrce_dds_client status` in the MAVLink Console says `Running` (step 3.4). 2) `systemctl status serial-getty@ttyAMA0` says masked (step 2.4). 3) The TELEM2 wires are tight and TX/RX are crossed. With the agent stopped, `stty -F /dev/ttyAMA0 921600 raw; timeout 4 cat /dev/ttyAMA0 \| wc -c` must print more than 0. |
| `oak-d` keeps restarting with `No available devices` or `Insufficient permissions` | The udev rule from step 2 is missing. Add it, then unplug and replug the camera. |
| Camera topics exist in `oak-d` but not in autonomy | Every container needs `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` (it's in both compose files). |
| Camera rate is low | It's in a black USB 2 port. Move it to a blue one. |
| PX4 says no GPS, even with the LED on | Reseat the 10-pin plug in GPS1 and reboot the Pixhawk. Indoors it should still show as present, just with no fix. |
| `airborne-bridge` won't start | A path in `.env` is wrong. Compare it with `ls /dev/serial/by-id/`. |

For everyday commands (logs, restart, update), see the [containers walkthrough](https://github.com/LiDRON7/Sensor-Control-Data-Gathering/blob/flight-test/CONTAINERS-WALKTHROUGH.md).
