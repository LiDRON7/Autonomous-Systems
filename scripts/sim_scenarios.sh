#!/usr/bin/env bash
# Fly the autonomy through the simulator's test field and check every way a mission can end.
#
#   ./scripts/simulation.sh up          (another terminal; wait until /autonomy/state is published)
#   ./scripts/sim_scenarios.sh          (all of them, in order, about 45 minutes without a GPU)
#   ./scripts/sim_scenarios.sh abort    (just one)
#
# Positions are Gazebo's own (ground truth), east/north/up in metres. Mission destinations are PX4 local
# north/east from where the drone spawned. The world is Simulation/worlds/outdoor_field.sdf.
set -uo pipefail

autonomy_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
sim_root="${SIMULATION_ROOT:-$autonomy_root/../Simulation}"
drone="${DRONE:-$(grep -s '^DRONE=' "$sim_root/.env" | cut -d= -f2)}"
drone="${drone:-huntsman}"
# Half the motor-to-motor width plus a prop, so clearance is measured from the drone's edge.
case "$drone" in huntsman) radius=0.45 ;; *) radius=0.40 ;; esac

ros() { docker exec lidron_autonomy bash -c "source /autonomy/install/setup.bash && $*" 2>&1; }
# The mission logs every state change; reading the log is far cheaper than starting ros2 for each poll.
state() { docker logs lidron_autonomy 2>&1 | grep -oE "mission_node-[0-9]+\]: process started|Mission [A-Z_]+ -> [A-Z_]+" \
  | tail -1 | awk '/process started/{print "IDLE"; next} {print $NF}'; }
call() { ros "timeout 20 ros2 service call $*" | grep -E "success|accepted|message" | tr -s ' ' | tr '\n' ' '; echo; }
destination() { call /autonomy/set_local_destination interfaces/srv/SetLocalDestination "'{north: $1, east: $2, down: 0.0}'"; }
enable() { call /autonomy/enable std_srvs/srv/SetBool "'{data: $1}'"; }
px4() { docker exec -w /px4/build/px4_sitl_default drone_px4 timeout 10 ./bin/px4-"$1" "${@:2}" 2>&1; }
nav_state() { px4 listener vehicle_status -n 1 | awk '$1=="nav_state:"{print $2; exit}'; }
armed() { px4 listener vehicle_status -n 1 | grep -q "arming_state: 2"; }
# Gazebo pose: east north up roll pitch yaw
pose() { docker exec drone_sim_dev timeout 10 gz model -m "${drone}_0" -p 2>/dev/null | tr -d '[]' \
  | awk '/XYZ/{getline x; getline r} END{print x, r}'; }
calc() { awk "BEGIN{$1}" </dev/null; }
# All limits are simulator seconds (PX4 runs in lockstep), so a slow machine just takes longer.
sim_now() { px4 listener vehicle_status -n 1 | awk '$1=="timestamp:"{printf "%d", $2/1e6; exit}'; }
sim_sleep() { local end=$(( $(sim_now) + $1 )); while (( $(sim_now) < end )); do sleep 1; done; }

failures=()
fail() { echo "  FAIL: $1"; failures+=("$scenario: $1"); }
ok() { echo "  ok: $1"; }

# Closest the drone's edge came to tree_2 (trunk r 0.12 up to 3.6 m at east 5 north 15, canopy sphere r 1.2 at 4.5 m).
tree_min=99
track() {
  local p="$1"
  # TRACK=file.txt keeps every sample (scenario, sim time, state, pose) for plotting the flight later.
  [ -n "${TRACK:-}" ] && echo "${scenario:-} $(sim_now) ${cur_state:-} $p" >> "$TRACK"
  read -r e n u r pt _ <<<"$p"
  [ -z "${u:-}" ] && return 0
  [ -n "${pt:-}" ] && calc "r=($r); p=($pt); exit !(r*r>1.0 || p*p>1.0)" \
    && { fail "drone flipped (roll $r, pitch $pt)"; return 1; }
  tree_min=$(calc "dc=sqrt(($e-5)^2+($n-15)^2+($u-4.5)^2)-1.2-($radius);
    dt=sqrt(($e-5)^2+($n-15)^2)-0.12-($radius); if($u>3.6+($radius)) dt=99;
    m=(dc<dt?dc:dt); printf \"%.2f\", (m<($tree_min)?m:($tree_min))")
}

# Wait until the mission state is $1, for at most $2 seconds. Any further arguments are states that fail it.
wait_state() {
  local want=$1 limit=$2 start; start=$(sim_now); local last="" s p
  while (( $(sim_now) - start < limit )); do
    s="$(state)"; p="$(pose)"
    if [[ "$s" != "$last" ]]; then
      printf '  %-15s %s\n' "$s" "$(awk '{printf "E %5.1f  N %5.1f  up %4.1f", $1, $2, $3}' <<<"$p")"
      last="$s"
    fi
    cur_state="$s"
    track "$p" || return 1
    [[ "$s" == "$want" ]] && return 0
    for bad in "${@:3}"; do [[ "$s" == "$bad" ]] && { fail "went to $s, expected $want"; return 1; }; done
  done
  fail "still $last after ${limit} s, expected $want"
  return 1
}

wait_disarmed() {
  local start; start=$(sim_now)
  while (( $(sim_now) - start < $1 )); do armed || return 0; track "$(pose)" || true; sleep 1; done
  fail "still armed ${1} s later"
  return 1
}

# Where it ended up against where it should be: within $3 m of north $1 east $2 (PX4 local == Gazebo here).
landed_near() {
  local p; p="$(pose)"; read -r e n u _ <<<"$p"
  local d; d=$(calc "printf \"%.2f\", sqrt(($n-($1))^2+($e-($2))^2)")
  if calc "exit !($d<=($3) && ($u)<0.6)"; then ok "on the ground ${d} m from N $1 E $2"
  else fail "ended at E $e N $n up $u, ${d} m from N $1 E $2"; fi
}

# Leave the drone landed, disarmed and the mission idle, whatever the last scenario did.
settle() {
  if armed; then echo "  (landing the leftover flight)"; px4 commander land >/dev/null; wait_disarmed 180 || true; fi
  ros "timeout 20 ros2 param set /mission mission_timeout_s 300.0" >/dev/null
  ros "timeout 20 ros2 param set /mission cruise_altitude_m 5.0" >/dev/null
  px4 param set SIM_BAT_DRAIN 60 >/dev/null; px4 param set SIM_BAT_MIN_PCT 50 >/dev/null
}

fly() { tree_min=99; destination "$1" "$2"; enable true; }

scenario_a_to_b() {
  fly 5.0 5.0
  wait_state COMPLETE 400 ABORT PILOT_OVERRIDE && landed_near 5 5 1.5
}

scenario_avoid() {
  # Straight north from (N 5, E 5) goes through tree_2 at N 15. At the usual 5 m the drone clears the 5.7 m
  # canopy, so cruise at 3 m here: trunk and canopy are both in the way.
  ros "timeout 20 ros2 param set /mission cruise_altitude_m 3.0" >/dev/null
  fly 24.0 5.0
  wait_state COMPLETE 600 ABORT PILOT_OVERRIDE && landed_near 24 5 1.5
  if calc "exit !($tree_min>0.3)"; then ok "closest to the tree: ${tree_min} m"
  else fail "came within ${tree_min} m of the tree (hit it if below 0)"; fi
}

scenario_abort() {
  fly 24.0 12.0
  wait_state NAVIGATING 200 ABORT PILOT_OVERRIDE || return
  sim_sleep 4
  read -r e n _ <<<"$(pose)"
  echo "  abort: $(call /autonomy/abort std_srvs/srv/Trigger '{}')"
  wait_state ABORT 10 || return
  [[ "$(nav_state)" == 18 ]] && ok "PX4 is in Land (nav_state 18)" || fail "PX4 nav_state $(nav_state), expected 18 (Land)"
  wait_disarmed 180 && landed_near "$n" "$e" 3.0
  echo "  a second abort: $(call /autonomy/abort std_srvs/srv/Trigger '{}')"
}

# A destination $2 m west of wherever the drone is: open field, nothing in the way.
west() { read -r e n _ <<<"$(pose)"; dest_n=$(calc "printf \"%.1f\", ($n)"); dest_e=$(calc "printf \"%.1f\", ($e)-($1)"); }

scenario_hold() {
  west 30; fly "$dest_n" "$dest_e"
  wait_state NAVIGATING 200 ABORT PILOT_OVERRIDE || return
  sim_sleep 1
  echo "  pause: $(enable false)"
  wait_state HOLD 10 || return
  sim_sleep 2; read -r e1 n1 _ <<<"$(pose)"; sim_sleep 10; read -r e2 n2 _ <<<"$(pose)"
  local drift; drift=$(calc "printf \"%.2f\", sqrt(($e2-($e1))^2+($n2-($n1))^2)")
  calc "exit !($drift<1.0)" && ok "held still: moved ${drift} m in 10 s" || fail "moved ${drift} m in 10 s of HOLD"
  echo "  resume: $(enable true)"
  wait_state COMPLETE 400 ABORT PILOT_OVERRIDE && landed_near "$dest_n" "$dest_e" 1.5
}

scenario_takeover() {
  fly 8.0 12.0
  wait_state NAVIGATING 200 ABORT PILOT_OVERRIDE || return
  sim_sleep 3
  echo "  pilot switches PX4 to Hold"; px4 commander mode auto:loiter >/dev/null
  wait_state PILOT_OVERRIDE 15 || return
  echo "  abort while the pilot flies: $(call /autonomy/abort std_srvs/srv/Trigger '{}')"
  echo "  enable while airborne: $(enable true)"
  sim_sleep 5
  [[ "$(nav_state)" == 4 ]] && ok "PX4 still in Hold, autonomy did not take it back" \
    || fail "PX4 nav_state $(nav_state) after takeover, expected 4 (Hold)"
  [[ "$(state)" == PILOT_OVERRIDE ]] && ok "autonomy still stood down" || fail "autonomy left PILOT_OVERRIDE while armed"
  px4 commander land >/dev/null; wait_disarmed 180
}

scenario_bad_landing() {
  if [[ "$drone" != huntsman ]]; then
    echo "  SKIP: $drone has no LiDAR, so it lands wherever it is sent (see README)"; return
  fi
  # crate_1 (1 m cube) sits at east 10, north 2. The LiDAR check should refuse it and search nearby.
  fly 2.0 10.0
  wait_state COMPLETE 900 PILOT_OVERRIDE || [[ "$(state)" == ABORT ]] || return
  wait_disarmed 180
  read -r e n u _ <<<"$(pose)"
  local d; d=$(calc "printf \"%.2f\", sqrt(($n-2)^2+($e-10)^2)")
  calc "exit !($u<0.6 && ($d)>0.5+($radius)+0.2)" && ok "landed on open ground ${d} m from the crate's centre" \
    || fail "ended at up $u, ${d} m from the crate's centre (on it or touching)"
}

scenario_timeout() {
  ros "timeout 20 ros2 param set /mission mission_timeout_s 25.0" >/dev/null
  fly -15.0 -15.0
  wait_state ABORT 200 COMPLETE PILOT_OVERRIDE && ok "mission timeout aborted it"
  wait_disarmed 180
  ros "timeout 20 ros2 param set /mission mission_timeout_s 300.0" >/dev/null
}

scenario_battery() {
  echo "  PX4 low battery action (COM_LOW_BAT_ACT): $(px4 param show COM_LOW_BAT_ACT | awk '/COM_LOW_BAT_ACT/{print $NF}')"
  # The simulated battery empties about 8 s after arming, so PX4's failsafe fires during the climb or cruise.
  px4 param set SIM_BAT_MIN_PCT 5 >/dev/null; px4 param set SIM_BAT_DRAIN 8 >/dev/null
  west 25; fly "$dest_n" "$dest_e"
  wait_state PILOT_OVERRIDE 200 COMPLETE ABORT LANDING || return
  read -r _ _ u _ <<<"$(pose)"
  calc "exit !(($u)>1.0)" && ok "PX4 battery failsafe took over in the air at ${u} m (nav_state $(nav_state))" \
    || fail "PX4 only took over on the ground (${u} m)"
  wait_disarmed 300
  px4 param set SIM_BAT_DRAIN 60 >/dev/null; px4 param set SIM_BAT_MIN_PCT 50 >/dev/null
}

scenario_perception_crash() {
  fly 10.0 5.0
  wait_state NAVIGATING 200 ABORT PILOT_OVERRIDE || return
  echo "  killing perception_node"; ros "pkill -f perception_node" >/dev/null
  if [[ "$drone" == huntsman ]]; then
    # No landing verdicts means stale LiDAR data: HOLD, then a controlled landing after hold_timeout_s.
    wait_state HOLD 15 && wait_state ABORT 60 && ok "held, then landed on its own"
  else
    echo "  (Widow's health check does not include perception; the map still comes from the depth points)"
    wait_state COMPLETE 400 ABORT PILOT_OVERRIDE
  fi
  wait_disarmed 180
  echo "  restarting the autonomy container"; docker restart lidron_autonomy >/dev/null; sleep 30
}

scenario_autonomy_crash() {
  fly 5.0 5.0
  wait_state NAVIGATING 200 ABORT PILOT_OVERRIDE || return
  read -r e n _ <<<"$(pose)"
  echo "  killing mission_node mid-flight"; ros "pkill -f mission_node" >/dev/null
  sim_sleep 6
  local ns; ns="$(nav_state)"
  [[ "$ns" != 14 ]] && ok "PX4 left Offboard on its own (nav_state $ns)" || fail "PX4 still in Offboard 6 s after the stream stopped"
  sim_sleep 10
  read -r e2 n2 u2 _ <<<"$(pose)"
  local d; d=$(calc "printf \"%.2f\", sqrt(($n2-($n))^2+($e2-($e))^2)")
  calc "exit !($d<4.0)" && ok "stayed within ${d} m of where the autonomy died" || fail "drifted ${d} m after the autonomy died"
  armed && px4 commander land >/dev/null
  wait_disarmed 180
  echo "  restarting the autonomy container"; docker restart lidron_autonomy >/dev/null; sleep 30
}

all=(a_to_b avoid abort hold takeover bad_landing timeout battery perception_crash autonomy_crash)
picked=("${@:-${all[@]}}")

echo "drone: $drone. Waiting for PX4 and the autonomy..."
for _ in $(seq 36); do
  px4 commander check | grep -q "Preflight check: OK" && [[ -n "$(state)" ]] && ready=1 && break
  sleep 5
done
[ "${ready:-}" ] || { echo "FAIL: PX4 or /autonomy/state not up. Run ./scripts/simulation.sh up first."; exit 1; }

for scenario in "${picked[@]}"; do
  echo; echo "== $scenario"
  settle
  "scenario_$scenario"
done
settle

echo
if (( ${#failures[@]} )); then printf 'FAILED:\n'; printf '  %s\n' "${failures[@]}"; exit 1; fi
echo "PASS: ${picked[*]}"
