#!/usr/bin/env bash
# Smoke-test openarm_skills ROS interfaces.
# Prerequisite: ros2 launch openarm_skills skills.launch.py  (or use_demo:=true)
set -eo pipefail

WS="${1:-$HOME/code/openArm}"
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

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
  timeout 60s ros2 service call "$@"
}

'''
启动方式：
demo（默认）：  ros2 launch openarm_skills skills.launch.py
重力补偿： ros2 launch openarm_skills skills.launch.py use_demo:=false use_gravity_comp:=true
重力补偿+远程RViz（板端假硬件、不启本地RViz）：
  板卡：ros2 launch openarm_skills skills.launch.py use_demo:=false use_gravity_comp:=true remote_rviz:=true
  远程Ubuntu：ros2 launch openarm_skills remote_viewer.launch.py use_fake_hardware:=true
仅skills：  ros2 launch openarm_skills skills.launch.py use_demo:=false
'''


# --- 1. gripper: open / half_close / grasp / close ---
'''
案例：
ros2 service call /openarm/gripper openarm_skills/srv/Gripper   "{arm: 'left', action: 'grasp', position: 0.00, force: 0.1, speed: 0.05}"
'''


: <<'GRIPPER_NOTES'
注释说明爪子相关功能的各参数的意义：
arm: 选择夹爪，left 或 right
action: 夹爪动作；open=张开到 0.040m，half_close=半闭合到 0.020m，close=闭合到 0.0m，grasp=闭合抓取并检测是否夹住
position: 可选夹爪指尖目标位置，单位米；>0 时覆盖 action 的默认位置，0 表示按 action 默认值执行
force: 可选夹持力/保持强度；0 表示默认值，500g 塑料水瓶可从 6~10N 调试
speed: 可选夹爪速度比例，0 表示默认值；0.1 较慢，0.9 较快

GRIPPER_NOTES
run_svc "gripper open (left)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'left', action: 'open', position: 0.0, force: 0.0, speed: 0.5}"

run_svc "gripper half_close (left)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'left', action: 'half_close', position: 0.0, force: 0.0, speed: 0.5}"

run_svc "gripper grasp (left)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'left', action: 'grasp', position: 0.0, force: 8.0, speed: 0.5}"

run_svc "gripper close (right)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'right', action: 'close', position: 0.0, force: 0.0, speed: 0.5}"

run_svc "gripper open (right)" /openarm/gripper openarm_skills/srv/Gripper \
  "{arm: 'right', action: 'open', position: 0.0, force: 0.0, speed: 0.5}"

# --- 2. goto_home ---
'''
案例：
ros2 service call /openarm/goto_home openarm_skills/srv/GotoHome "{arm: 'both', speed_scale: 0.15}"
'''
run_svc "goto_home both" /openarm/goto_home openarm_skills/srv/GotoHome \
  "{arm: 'both', speed_scale: 0.15}"

# --- 2b. carry_action ---
# width: 双臂 TCP 横向间距，单位 m；0.0 使用默认宽度 0.307m。
# gripper: close | open | half_close。
'''
命令行测试用例：
ros2 service call /openarm/carry_action openarm_skills/srv/CarryAction "{width: 0.318, gripper: 'open', speed_scale: 0.15}"

width: 单位 m；0.0 表示默认宽度 0.307m，最大0.384，最小0.20，超过范围会默认用0.307或者报错
gripper: 只能是 close、open、half_close
speed_scale: 0.0~1.0，0.0 用默认速度


'''
run_svc "carry_action default width open gripper" /openarm/carry_action openarm_skills/srv/CarryAction \
  "{width: 0.0, gripper: 'open', speed_scale: 0.15}"

# --- 3. stop (no-op if idle) ---
run_svc "stop" /openarm/stop openarm_skills/srv/Stop \
  "{cmd_id: 'test-stop-001'}"

# --- 4. pick_place (short timeout; adjust poses for your cell) ---

: <<'PICK_PLACE_NOTES'
cmd_id: 本次命令 ID，用于区分请求，例如 test-pp-001
arm: 使用哪只手臂，left 或 right
pose_source: 位姿来源，upper_computer 表示直接使用命令里的位姿，camera 表示由相机识别
target_name: pose_source: camera 时使用的目标名称，例如 cup
target_index: 同名目标有多个时用于选择第几个
grasp_pose: 抓取位姿，upper_computer 模式下有效
place_pose: 放置位姿，upper_computer 模式下有效
position.x/y/z: 目标位置，单位米，基于机器人 base frame
orientation.x/y/z/w: 四元数姿态
approach_offset_m: 抓取/放置前的接近偏移距离，单位米
retreat_offset_m: 抓取后/放置后的抬升退出距离，单位米
speed_scale: 速度比例，范围通常 0.0 ~ 1.0
timeout_s: 超时时间，单位秒；0 表示使用服务默认值

常用四元素对照表（openarm 零位时 TCP 的「approach 轴」= 本体 +Z，即朝上）：

四元数 (x,y,z,w) 通过绕 Y 轴旋转 θ，把 TCP approach 轴从 +Z 旋到目标方向：

姿态                                                    x       y       z       w         approach 指向     palm 朝向
默认 / identity                                          0       0       0       1.0       +Z（朝上）         +X
绕Y轴 90°（水平向前，背朝上）                            0       0.7071  0       0.7071    +X（水平向前）     -Z（朝下）
★ 水平向前 + palm 朝上（90°绕Y + 180°滚转 approach 轴）  0.7071  0       0.7071  0         +X（水平向前）     +Z（朝上）✅
绕Y轴 180°（垂直向下，背朝 +X）                          0       1       0       0         -Z（朝下）         +X 或 -X（看滚转）
绕Y轴 -90°（水平向后）                                   0      -0.7071  0       0.7071    -X（水平向后）     多在身后才有意义
绕Z轴 90°（沿 +Z 旋转，配合俯仰使用）                    0       0       0.7071  0.7071    approach 轴不变   一般与上面组合

提示：同一 approach 方向上还可以"沿 approach 轴滚转 180°"得到等价 IK 解（手腕 joint7 ±π），
靠四元数选择 palm 朝向哪边。先决定 approach 方向（绕 Y 角度），再决定 palm 朝向
（是否在 X、Z 两个分量上加同号 0.7071）。

注意（已经过实测验证）：identity (0,0,0,1) 并不是「TCP 垂直向下」。openarm 在 home
位姿（所有关节=0）时 TCP 的 approach 方向沿本体 +Z 朝上。要做桌面物体的顶部抓取，
用 ★ 行的 (0,1,0,0)。先前 (0,0.7071,0,0.7071) 给出的是「水平向前」抓取（如下图 1
所示），不是垂直向下；本表已更正。

            案例1（推荐：垂直向下抓取，TCP 朝下 = 绕Y轴 180°）：
            ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace \
            "{cmd_id: 'test-pp-003', arm: 'right', pose_source: 'upper_computer',
              target_name: '', target_index: 0,
              grasp_pose: {position: {x: 0.28, y: -0.18, z: 0.28},
                          orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}},
              place_pose: {position: {x: 0.28, y: -0.28, z: 0.28},
                          orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}},
              approach_offset_m: 0.05, retreat_offset_m: 0.05,
              speed_scale: 0.10, timeout_s: 120.0}" --feedback

            案例1b（水平向前抓取，approach 沿 +X，背部朝上 —— 一般不推荐，因为手抓"背朝天"）：
            ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace \
            "{cmd_id: 'test-pp-003', arm: 'right', pose_source: 'upper_computer',
              target_name: '', target_index: 0,
              grasp_pose: {position: {x: 0.28, y: -0.18, z: 0.28},
                          orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}},
              place_pose: {position: {x: 0.28, y: -0.28, z: 0.28},
                          orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}},
              approach_offset_m: 0.05, retreat_offset_m: 0.05,
              speed_scale: 0.10, timeout_s: 120.0}" --feedback

案例1c（水平向前抓取，approach 沿 +X，★ palm 朝上，更接近"人手手心向上"姿态）：
  说明：这是在案例1b基础上沿 approach 轴再滚转 180°（让夹爪的 palm 面翻上来）。
  四元数 = (0,0.7071,0,0.7071) ⊗ 绕局部Z 180°的滚转  = (0.7071, 0, 0.7071, 0)
ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace \
"{cmd_id: 'test-pp-003', arm: 'right', pose_source: 'upper_computer',
  target_name: '', target_index: 0,
  grasp_pose: {position: {x: 0.28, y: -0.18, z: 0.28},
               orientation: {x: 0.7071, y: 0.0, z: 0.7071, w: 0.0}},
  place_pose: {position: {x: 0.28, y: -0.28, z: 0.28},
               orientation: {x: 0.7071, y: 0.0, z: 0.7071, w: 0.0}},
  approach_offset_m: 0.05, retreat_offset_m: 0.05,
  speed_scale: 0.10, timeout_s: 120.0}" --feedback

案例1d（推荐：垂直向下抓取，TCP 朝下 = 绕Y轴 180°）：
ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace \
"{cmd_id: 'test-pp-003', arm: 'right', pose_source: 'upper_computer',
  target_name: '', target_index: 0,
  grasp_pose: {position: {x: 0.28, y: -0.18, z: 0.28},
              orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}},
  place_pose: {position: {x: 0.28, y: -0.28, z: 0.28},
              orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}},
  approach_offset_m: 0.05, retreat_offset_m: 0.05,
  speed_scale: 0.10, timeout_s: 120.0}" --feedback

案例1e（左臂：身后抓取 → 身前放置，已仿真验证）：
  要点：
  - grasp 在身后（-X, y>0 为左臂侧），place 在身前（+X）；四元数用水平 palm 朝上，勿用 (1,0,0,0) 垂直朝下（身后点 IK 易失败）
  - target_radius：pick.approach 绕抓取点球体；transport/place.approach 不避障；place.retreat 绕放置点球体
  - place.z=0.68 可达；若需更稳可改为 z=0.52（见下方注释）
ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace "{cmd_id: 'test-pp-1e', arm: 'left', pose_source: 'upper_computer',
  target_name: '', target_index: 0,
  grasp_pose: {position: {x: -0.25, y: 0.08, z: 0.28},
              orientation: {x: -0.7071, y: 0.0, z: 0.7071, w: 0.0}},
  place_pose: {position: {x: 0.48, y: 0.18, z: 0.68},
              orientation: {x: 0.7071, y: 0.0, z: 0.7071, w: 0.0}},
  approach_offset_m: 0.05, retreat_offset_m: 0.05,
  target_radius: 0.04, gripper_force: 8.0, gripper_speed: 0.3,
  speed_scale: 0.10, timeout_s: 120.0}" --feedback
# 备选（身前放置略低、更稳）：place_pose z=0.52 → position: {x: 0.36, y: 0.17, z: 0.52}


Feedback: status: perceiving / phase: pick.detect          ← 解析抓取位姿（upper_computer模式直接跳过）
Feedback: status: grasping  / phase: pick.open_gripper     ← 前往目标前先张开夹爪；已张开则跳过
Feedback: status: grasping  / phase: pick.approach         ← 夹爪已张开后运动到悬停点（grasp+approach_offset）
Feedback: status: grasping  / phase: pick.descend          ← 笛卡尔直线下降到grasp点（爪子已张开）
Feedback: status: grasping  / phase: pick.close_gripper    ← 闭合夹爪抓取物体
Feedback: status: grasping  / phase: pick.fake_grasp        ← RViz调试开关打开时：无物体也假定已抓住
Feedback: status: transporting / phase: transport            ← 前往放置区（无避障球体）
Feedback: status: placing   / phase: place.approach         ← 到放置悬停点（无避障球体）
Feedback: status: placing   / phase: place.descend           ← 下降到 place 点
Feedback: status: placing   / phase: place.open_gripper     ← 张开夹爪释放
Feedback: status: placing   / phase: place.retreat           ← 撤离时绕放置点 target_radius 球体
Feedback: status: returning / phase: return.start           ← 放置后回到本次动作开始时的手臂关节位
Feedback: status: returning / phase: return.home            ← 如果没有起始关节位或 return.start 失败，则回 home
Feedback: status: returning / phase: return.restore_gripper ← 恢复动作开始时的夹爪状态；读不到时默认闭合

真实抓取时请将 skills.yaml 里的 debug_assume_grasp_success 改为 false；此时如果夹爪关闭但
检测到没夹住物体，会返回 success:false / result_code:1003 (GRIP_NOT_HELD)。

常见错误码 (result_code) 与 message 示例：
  1001 PLAN_FAILED   "pick failed at pick.approach: IK unreachable (pose not solvable)"
                    → grasp_pose 不可达，常因姿态四元数错误（如用 identity 而非 ★ 行）
  1001 PLAN_FAILED   "pick failed at pick.descend: cartesian fraction below threshold"
                    → 笛卡尔下降被几何/奇异性阻挡
  1003 GRIP_NOT_HELD "pick failed at pick.close_gripper"
                    → 夹爪关闭但没夹到物体
  1004 EXECUTE_FAILED → 控制器执行失败（运动学没问题，硬件层面）
  2003 CAMERA_OUT_OF_RANGE "grasp_pose outside workspace"
                    → 位置在 workspace 球外（半径 1.2m）或 z 越界


PICK_PLACE_NOTES
echo ""
echo ">>> pick_place (right arm, upper_computer poses — edit script if needed)"
ros2 action send_goal /openarm/pick_place openarm_skills/action/PickPlace \
  "{cmd_id: 'test-pp-001', arm: 'right', pose_source: 'upper_computer',
    grasp_pose: {position: {x: 0.42, y: -0.10, z: 0.20},
                 orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}},
    place_pose: {position: {x: 0.30, y: -0.20, z: 0.20},
                 orientation: {x: 0.0, y: 0.7071, z: 0.0, w: 0.7071}},
    approach_offset_m: 0.05, retreat_offset_m: 0.05,
    target_radius: 0.04, gripper_force: 8.0, gripper_speed: 0.3,
    speed_scale: 0.10, timeout_s: 60.0}" --feedback || true

echo ""
echo "== Done. Check success:true in each response; pick_place may fail if poses are unreachable. =="
