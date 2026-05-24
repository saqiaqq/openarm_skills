#!/usr/bin/env bash
# Remote RViz viewer for openArm simulation / real robot.
#
# Run this script on a SECOND Ubuntu 22 machine (viewer) that is on the
# same LAN as the robot machine. Both machines must share the same
# ROS_DOMAIN_ID.
#
# Usage (on the VIEWER machine):
#   bash sim_viewer.sh [ROBOT_IP] [ROS_DOMAIN_ID]
#
# Defaults:
#   ROBOT_IP     -- auto (no unicast needed if multicast works)
#   DOMAIN_ID    -- 0
#
# Prerequisites on the viewer machine:
#   sudo apt install ros-humble-desktop ros-humble-moveit
#   # also install the openarm description package or copy the URDF/meshes:
#   #   scp -r ROBOT_IP:~/code/openArm/install/openarm_description \
#   #              ~/code/openArm_viewer/install/
#
set -eo pipefail

ROBOT_IP="${1:-}"
DOMAIN_ID="${2:-0}"
WS="${3:-$HOME/code/openArm}"

source /opt/ros/humble/setup.bash
if [[ -f "${WS}/install/setup.bash" ]]; then
  source "${WS}/install/setup.bash"
fi

export ROS_DOMAIN_ID="${DOMAIN_ID}"

# Optional: force unicast to the robot if multicast is blocked on the LAN.
if [[ -n "${ROBOT_IP}" ]]; then
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  FASTDDS_CFG="/tmp/openarm_fastdds_viewer.xml"
  cat > "${FASTDDS_CFG}" <<XML
<?xml version="1.0" encoding="utf-8"?>
<dds>
  <profiles>
    <participant profile_name="viewer_participant" is_default_profile="true">
      <rtps>
        <builtin>
          <initialPeersList>
            <locator>
              <udpv4>
                <address>${ROBOT_IP}</address>
              </udpv4>
            </locator>
          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
XML
  export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTDDS_CFG}"
  echo "[sim_viewer] Unicast peer set to ${ROBOT_IP}"
fi

echo "[sim_viewer] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "[sim_viewer] Waiting for /joint_states from robot ..."

# Try to find a moveit.rviz config to load.
RVIZ_CFG=""
for candidate in \
    "${WS}/install/openarm_bimanual_moveit_config/share/openarm_bimanual_moveit_config/config/moveit.rviz" \
    "${WS}/src/openarm_ros2/openarm_bimanual_moveit_config/config/moveit.rviz"
do
  if [[ -f "${candidate}" ]]; then
    RVIZ_CFG="${candidate}"
    break
  fi
done

if [[ -n "${RVIZ_CFG}" ]]; then
  echo "[sim_viewer] Using RViz config: ${RVIZ_CFG}"
  exec ros2 run rviz2 rviz2 -d "${RVIZ_CFG}"
else
  echo "[sim_viewer] No moveit.rviz found, starting RViz without config."
  echo "             Tip: Add a RobotModel display (topic: /robot_description)"
  echo "                  and a JointState display (topic: /joint_states)"
  exec ros2 run rviz2 rviz2
fi
