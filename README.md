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
# 1. start MoveIt + controllers + RViz (terminal A)
ros2 launch openarm_bimanual_moveit_config demo.launch.py

# 2. start the skill server (terminal B)
ros2 launch openarm_skills skills.launch.py

# 3. (optional) start the perception stub if you want pose_source=camera
ros2 launch openarm_perception perception.launch.py

# 4. drive a pick & place from the CLI (upper_computer mode)
ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace "{
  cmd_id: 'cli-1', arm: 'right', pose_source: 'upper_computer',
  grasp_pose: {position: {x: 0.42, y: 0.10, z: 0.20},
               orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}},
  place_pose: {position: {x: 0.30, y: -0.20, z: 0.20},
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
