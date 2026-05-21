# openarm_skills

Skill (technique) layer for the OpenArm bimanual robot. Implements PRD tasks
1 / 2 / 3 / 5: Cartesian linear motion, `pick` / `place` / `pick_and_place`
flows with status feedback, plus a basic exception-handling envelope.

## Provided ROS2 interfaces

| Kind     | Name                              | Type                                    |
|----------|-----------------------------------|-----------------------------------------|
| Action   | `/openarm/pick_place`             | `openarm_skills/action/PickPlace`       |
| Service  | `/openarm/stop`                   | `openarm_skills/srv/Stop`               |
| Service  | `/openarm/goto_home`              | `openarm_skills/srv/GotoHome`           |
| Service  | `/openarm/gripper`                | `openarm_skills/srv/Gripper`            |
| Client   | `/openarm/detect_grasp_pose`      | `openarm_skills/srv/DetectGraspPose`    |

The same package also defines the generic JSON envelope srv used by
`openarm_api`: `openarm_skills/srv/StringCommand`.

## Quick start

```bash
# One-shot: demo (MoveIt + controllers + RViz) + skill_server (default)
ros2 launch openarm_skills skills.launch.py

# Headless / no RViz
ros2 launch openarm_skills skills.launch.py use_rviz:=false

# Skills only (demo already running in another terminal)
ros2 launch openarm_skills skills.launch.py use_demo:=false

# (optional) perception stub for pose_source=camera
ros2 launch openarm_perception perception.launch.py

# Automated smoke tests (after stack is up)
bash src/openarm_skills/scripts/test_openarm_skills.sh ~/code/openArm
```

## Interface test cheat sheet

| # | Interface | Type | Test command |
|---|-----------|------|----------------|
| 1 | `/openarm/gripper` | Service | `ros2 service call /openarm/gripper openarm_skills/srv/Gripper "{arm: 'left', action: 'open', position: 0.0, force: 0.0}"` |
| 2 | `/openarm/gripper` | Service | `action: 'half_close'` — same as above, change action |
| 3 | `/openarm/gripper` | Service | `action: 'close'` |
| 4 | `/openarm/gripper` | Service | `action: 'grasp'` — compliant close / hold (needs object for full test) |
| 5 | `/openarm/goto_home` | Service | `ros2 service call /openarm/goto_home openarm_skills/srv/GotoHome "{arm: 'both', speed_scale: 0.15}"` |
| 6 | `/openarm/stop` | Service | `ros2 service call /openarm/stop openarm_skills/srv/Stop "{cmd_id: 'stop-1'}"` |
| 7 | `/openarm/pick_place` | Action | see § Quick start item 4 below |
| 8 | `/openarm/detect_grasp_pose` | Client (in skill_server) | start `openarm_perception`; use `pose_source: camera` in pick_place |

```bash
# 4. drive a pick & place from the CLI (upper_computer mode)
# Poses are in the `world` frame (see skills.yaml base_frame). Adjust x/y/z for your cell.
ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace "{
  cmd_id: 'cli-1', arm: 'right', pose_source: 'upper_computer',
  grasp_pose: {position: {x: 0.42, y: -0.10, z: 0.20},
               orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}},
  place_pose: {position: {x: 0.30, y: -0.30, z: 0.20},
               orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}},
  approach_offset_m: 0.05, retreat_offset_m: 0.05,
  speed_scale: 0.10, timeout_s: 30.0
}" --feedback
```

## Error codes (single source of truth)

These integers are emitted in the action `result.result_code` and propagated
to the JSON gateway. The full table lives in
[`include/openarm_skills/error_codes.hpp`](include/openarm_skills/error_codes.hpp)
and is mirrored in `openarm_api/openarm_api/schemas/error_codes.json`.

| code | name                | meaning                                                         |
|-----:|---------------------|-----------------------------------------------------------------|
|    0 | `OK`                | success                                                         |
| 1001 | `PLAN_FAILED`       | MoveIt plan failed / Cartesian fraction below threshold (after retries) |
| 1002 | `ACTION_TIMEOUT`    | step exceeded `step_timeout_s`                                  |
| 1003 | `GRIP_NOT_HELD`     | finger position indicates empty grasp after retry               |
| 1004 | `EXECUTE_FAILED`    | controller reported execution failure                           |
| 2001 | `CAMERA_NO_CLOUD`   | perception service unreachable / no point cloud                 |
| 2002 | `CAMERA_NO_TARGET`  | target object / region not detected                             |
| 2003 | `CAMERA_OUT_OF_RANGE` | computed pose outside workspace bounds                        |
| 2004 | `PERCEPTION_TIMEOUT`| perception call exceeded `perception_timeout_s`                 |
| 3001 | `BAD_REQUEST`       | argument validation failed (bad arm, pose, JSON schema)         |
| 3002 | `UNSUPPORTED_CMD`   | unknown `cmd_type` in JSON envelope                             |
| 9001 | `STOPPED_BY_USER`   | aborted by `/openarm/stop` or action cancel                     |
| 9002 | `INTERNAL_ERROR`    | unexpected exception                                            |

## Tunables (`config/skills.yaml`)

`default_speed_scale`, `transport_speed_scale`, `approach_offset_m`,
`retreat_offset_m`, `cartesian_min_fraction`, `plan_retry_count`,
`grasp_retry_count`, workspace bounds, gripper finger targets, perception
service name and timeout. See the YAML for inline comments.
