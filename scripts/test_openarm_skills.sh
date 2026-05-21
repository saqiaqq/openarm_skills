#!/usr/bin/env bash
# Smoke-test openarm_skills ROS interfaces.
# Prerequisite: ros2 launch openarm_skills skills.launch.py  (or use_demo:=true)
set -euo pipefail

WS="${1:-$HOME/code/openArm}"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"

TIMEOUT=120
echo "== Waiting for skill_server (max ${TIMEOUT}s) =="
deadline=$((SECONDS + TIMEOUT))
while (( SECONDS < deadline )); do
  if ros2 service list 2>/dev/null | grep -q '^/openarm/gripper$'; then
    echo "skill_server is up."
    break
  fi
  sleep 2
done
if ! ros2 service list 2>/dev/null | grep -q '^/openarm/gripper$'; then
  echo "ERROR: /openarm/gripper not available. Start skills first."
  exit 1
fi

run_svc() {
  local name="$1"
  shift
  echo ""
  echo ">>> ${name}"
  ros2 service call "$@" --timeout-sec 60
}

# --- 1. gripper: open / half_close / grasp / close ---
run_svc "gripper open (left)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'left', action: 'open', position: 0.0, force: 0.0}"

run_svc "gripper half_close (left)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'left', action: 'half_close', position: 0.0, force: 0.0}"

run_svc "gripper grasp (left)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'left', action: 'grasp', position: 0.0, force: 0.0}"

run_svc "gripper close (right)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'right', action: 'close', position: 0.0, force: 0.0}"

run_svc "gripper open (right)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'right', action: 'open', position: 0.0, force: 0.0}"

# --- 2. goto_home ---
run_svc "goto_home both" /openarm/goto_home openarm_skills/srv/GotoHome \
  "{arm: 'both', speed_scale: 0.15}"

# --- 3. stop (no-op if idle) ---
run_svc "stop" /openarm/stop openarm_skills/srv/Stop \
  "{cmd_id: 'test-stop-001'}"

# --- 4. pick_place (short timeout; adjust poses for your cell) ---
echo ""
echo ">>> pick_place (right arm, upper_computer poses — edit script if needed)"
ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace \
  "{cmd_id: 'test-pp-001', arm: 'right', pose_source: 'upper_computer',
    grasp_pose: {position: {x: 0.42, y: -0.10, z: 0.20},
                 orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}},
    place_pose: {position: {x: 0.30, y: -0.20, z: 0.20},
                 orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}},
    approach_offset_m: 0.05, retreat_offset_m: 0.05,
    speed_scale: 0.10, timeout_s: 60.0}" --feedback || true

echo ""
echo "== Done. Check success:true in each response; pick_place may fail if poses are unreachable. =="
