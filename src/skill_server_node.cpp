// SPDX-License-Identifier: Apache-2.0
//
// openarm_skills::SkillServerNode
// --------------------------------
// Implements PRD tasks 1 / 2 / 3 / 5: Cartesian linear motion, pick + place
// flows, the combined pick_and_place action, and basic exception handling /
// status feedback.  External clients should normally talk to this node via
// the openarm_api JSON bridge -- the ROS2 interfaces below are kept simple
// and stable so the bridge layer can stay thin.
//
// Exposed interfaces:
//   Action  : /openarm/pick_place                 [openarm_skills/action/PickPlace]
//   Service : /openarm/stop                       [openarm_skills/srv/Stop]
//   Service : /openarm/goto_home                  [openarm_skills/srv/GotoHome]
//   Service : /openarm/gripper                    [openarm_skills/srv/Gripper]
//   Client  : /openarm/detect_grasp_pose          [openarm_skills/srv/DetectGraspPose]

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "openarm_skills/action/pick_place.hpp"
#include "openarm_skills/srv/stop.hpp"
#include "openarm_skills/srv/goto_home.hpp"
#include "openarm_skills/srv/gripper.hpp"
#include "openarm_skills/srv/detect_grasp_pose.hpp"
#include "openarm_skills/error_codes.hpp"

namespace openarm_skills
{

using PickPlaceAction = openarm_skills::action::PickPlace;
using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlaceAction>;
using moveit::planning_interface::MoveGroupInterface;
// Humble 版 MoveIt2 的 MoveGroupInterface 没有 SharedPtr typedef，手动补充
using MoveGroupInterfacePtr = std::shared_ptr<MoveGroupInterface>;

class SkillServerNode : public rclcpp::Node
{
public:
  SkillServerNode()
  : rclcpp::Node("skill_server",
                 rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // ---- parameters --------------------------------------------------------
    auto get = [this](const std::string & n, auto def) {
      using T = decltype(def);
      T v = def;
      this->get_parameter_or(n, v, def);
      return v;
    };

    left_arm_group_         = get("left_arm_group",   std::string("left_arm"));
    right_arm_group_        = get("right_arm_group",  std::string("right_arm"));
    left_grip_group_        = get("left_gripper_group",  std::string("left_gripper"));
    right_grip_group_       = get("right_gripper_group", std::string("right_gripper"));
    base_frame_             = get("base_frame",       std::string("world"));
    left_eef_link_          = get("left_eef_link",    std::string("openarm_left_hand_tcp"));
    right_eef_link_         = get("right_eef_link",   std::string("openarm_right_hand_tcp"));
    goal_pos_tol_           = get("goal_position_tolerance_m", 0.01);
    goal_ori_tol_           = get("goal_orientation_tolerance_rad", 0.1);

    default_speed_scale_    = get("default_speed_scale",   0.10);
    transport_speed_scale_  = get("transport_speed_scale", 0.30);
    cartesian_eef_step_     = get("cartesian_eef_step_m",  0.005);
    // 5mm step + well-conditioned arm → adjacent IK solutions differ by < 0.1 rad.
    // A threshold of 1.0 rad per step catches IK branch flips (~2+ rad) while
    // tolerating mild near-singularity variations.  The old 5.0 rad value
    // allowed a full branch flip to go undetected, causing the Cartesian
    // trajectory to silently include huge intermediate joint excursions.
    cartesian_jump_thresh_  = get("cartesian_jump_threshold", 1.0);
    cartesian_min_fraction_ = get("cartesian_min_fraction", 0.95);
    approach_offset_m_      = get("approach_offset_m", 0.05);
    retreat_offset_m_       = get("retreat_offset_m",  0.05);
    step_timeout_s_         = get("step_timeout_s",    10.0);
    planning_time_s_        = get("planning_time_s",   15.0);
    plan_retry_count_       = get("plan_retry_count",  2);
    grasp_retry_count_      = get("grasp_retry_count", 1);

    gripper_open_pos_       = get("gripper_open_pos",   0.040);
    gripper_half_pos_       = get("gripper_half_pos",   0.020);
    gripper_closed_pos_     = get("gripper_closed_pos", 0.0);
    gripper_close_thresh_   = get("gripper_close_threshold_m", 0.001);

    workspace_radius_       = get("workspace_radius_m", 1.20);
    min_z_                  = get("min_z_m", -0.10);
    max_z_                  = get("max_z_m",  1.50);

    perception_srv_name_    = get("perception_service",
                                  std::string("/openarm/detect_grasp_pose"));
    perception_timeout_s_   = get("perception_timeout_s", 5.0);

    // ---- service clients ---------------------------------------------------
    perception_client_ = this->create_client<openarm_skills::srv::DetectGraspPose>(
      perception_srv_name_);

    // ---- service servers ---------------------------------------------------
    stop_srv_ = this->create_service<openarm_skills::srv::Stop>(
      "/openarm/stop",
      std::bind(&SkillServerNode::handleStop, this,
                std::placeholders::_1, std::placeholders::_2));

    home_srv_ = this->create_service<openarm_skills::srv::GotoHome>(
      "/openarm/goto_home",
      std::bind(&SkillServerNode::handleHome, this,
                std::placeholders::_1, std::placeholders::_2));

    gripper_srv_ = this->create_service<openarm_skills::srv::Gripper>(
      "/openarm/gripper",
      std::bind(&SkillServerNode::handleGripper, this,
                std::placeholders::_1, std::placeholders::_2));

    // ---- action server -----------------------------------------------------
    using namespace std::placeholders;
    pick_place_server_ = rclcpp_action::create_server<PickPlaceAction>(
      this, "/openarm/pick_place",
      std::bind(&SkillServerNode::handleGoal,   this, _1, _2),
      std::bind(&SkillServerNode::handleCancel, this, _1),
      std::bind(&SkillServerNode::handleAccept, this, _1));

    RCLCPP_INFO(get_logger(), "openarm_skills skill_server ready.");
  }

  // MoveGroupInterface needs a *separate* node; lazily build them so we can
  // construct the SkillServerNode first and call rclcpp::spin() on it.
  void initMoveGroups()
  {
    auto opts = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    mg_node_ = rclcpp::Node::make_shared("skill_server_mgi", opts);

    left_arm_   = std::make_shared<MoveGroupInterface>(mg_node_, left_arm_group_);
    right_arm_  = std::make_shared<MoveGroupInterface>(mg_node_, right_arm_group_);
    left_grip_  = std::make_shared<MoveGroupInterface>(mg_node_, left_grip_group_);
    right_grip_ = std::make_shared<MoveGroupInterface>(mg_node_, right_grip_group_);

    configureArmMoveGroup(left_arm_, left_eef_link_, "left");
    configureArmMoveGroup(right_arm_, right_eef_link_, "right");

    RCLCPP_INFO(get_logger(),
                "MoveGroup frame='%s' EEF: left=%s right=%s (planning_time=%.1fs)",
                base_frame_.c_str(), left_eef_link_.c_str(), right_eef_link_.c_str(),
                planning_time_s_);
  }

  void configureArmMoveGroup(MoveGroupInterfacePtr arm,
                              const std::string & eef_link,
                              const std::string & label)
  {
    arm->setEndEffectorLink(eef_link);
    arm->setPoseReferenceFrame(base_frame_);
    arm->setPlanningTime(planning_time_s_);
    arm->setNumPlanningAttempts(5);
    arm->setGoalPositionTolerance(goal_pos_tol_);
    arm->setGoalOrientationTolerance(goal_ori_tol_);
    arm->setMaxVelocityScalingFactor(default_speed_scale_);
    arm->setMaxAccelerationScalingFactor(default_speed_scale_);
    logCurrentEef(arm, label);
  }

  void logCurrentEef(MoveGroupInterfacePtr arm, const std::string & label) const
  {
    try {
      const auto pose = arm->getCurrentPose(arm->getEndEffectorLink());
      const auto & p = pose.pose.position;
      RCLCPP_INFO(get_logger(),
                  "[%s] current TCP in '%s': x=%.3f y=%.3f z=%.3f (use RViz to tune goals)",
                  label.c_str(), arm->getPlanningFrame().c_str(), p.x, p.y, p.z);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "[%s] cannot read current TCP: %s",
                  label.c_str(), e.what());
    }
  }

  static void logTargetPose(rclcpp::Logger logger,
                            const std::string & phase,
                            const geometry_msgs::msg::Pose & p)
  {
    RCLCPP_ERROR(logger,
                 "[%s] target in planning frame: pos [%.3f, %.3f, %.3f] "
                 "ori [%.3f, %.3f, %.3f, %.3f] — check reachability / y sign "
                 "(right arm y<0, left arm y>0)",
                 phase.c_str(), p.position.x, p.position.y, p.position.z,
                 p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  }

  // Normalize a quaternion in-place; return true if correction was needed.
  // An un-normalized quaternion passed to IK produces a garbled rotation
  // target and causes wild, unpredictable arm motions.
  static bool normalizeQuat(geometry_msgs::msg::Pose & pose, const std::string & tag,
                             rclcpp::Logger logger)
  {
    auto & q = pose.orientation;
    const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    if (n < 1e-6) {
      RCLCPP_WARN(logger, "[%s] quaternion magnitude ~0, reset to identity", tag.c_str());
      q.x = 0; q.y = 0; q.z = 0; q.w = 1;
      return true;
    }
    if (std::abs(n - 1.0) > 1e-3) {
      RCLCPP_WARN(logger,
                  "[%s] quaternion not unit (|q|=%.4f), normalizing "
                  "[%.4f, %.4f, %.4f, %.4f] → "
                  "[%.4f, %.4f, %.4f, %.4f]",
                  tag.c_str(), n,
                  q.x, q.y, q.z, q.w,
                  q.x/n, q.y/n, q.z/n, q.w/n);
      q.x /= n; q.y /= n; q.z /= n; q.w /= n;
      return true;
    }
    return false;
  }

  // Apply ±2π wrap to each joint in `sol` so that travel from `current` is
  // minimised.  Works on a solution vector in-place; returns true if any joint
  // was changed.  Only accepts a wrap if the candidate stays within joint bounds.
  bool wrapSolution(const moveit::core::JointModelGroup * jmg,
                    const std::vector<double> & current,
                    std::vector<double> & sol) const
  {
    if (current.size() != sol.size()) return false;
    bool changed = false;
    const auto & joints = jmg->getActiveJointModels();
    for (size_t i = 0; i < sol.size(); ++i) {
      double diff = sol[i] - current[i];
      double candidate = 0.0;
      if (diff > M_PI)        candidate = sol[i] - 2 * M_PI;
      else if (diff < -M_PI)  candidate = sol[i] + 2 * M_PI;
      else continue;

      const auto & bounds = joints[i]->getVariableBounds();
      if (bounds.empty() || !bounds[0].position_bounded_ ||
          (candidate >= bounds[0].min_position_ &&
           candidate <= bounds[0].max_position_)) {
        sol[i] = candidate;
        changed = true;
      }
    }
    return changed;
  }

  // Solve IK for `target_pose` using multiple seeds and return the joint
  // solution with minimum total travel from `current_joints`.
  //
  // Why: KDL IK is seed-sensitive and from some configurations (e.g. home =
  // all-zeros) it converges to the wrong IK branch (elbow-up vs elbow-down).
  // Trying several seeds (current state first, then random) and selecting the
  // minimum-travel solution reliably finds the kinematically nearest branch.
  //
  // Returns true and populates `best_joints` if at least one seed succeeded.
  bool solveIkBestBranch(MoveGroupInterfacePtr arm,
                          const geometry_msgs::msg::Pose & target_pose,
                          const std::vector<double> & current_joints,
                          int n_seeds,
                          std::vector<double> & best_joints,
                          const std::string & phase)
  {
    const auto * jmg =
      arm->getRobotModel()->getJointModelGroup(arm->getName());
    if (!jmg) return false;

    const std::string eef = arm->getEndEffectorLink();
    auto robot_model = arm->getRobotModel();

    double best_travel = std::numeric_limits<double>::max();
    int n_ok = 0;

    for (int attempt = 0; attempt < n_seeds; ++attempt) {
      moveit::core::RobotState rs(robot_model);
      if (attempt == 0) {
        // Seed from the actual current configuration: most likely to land on
        // the nearest IK branch.
        if (current_joints.size() == jmg->getVariableCount())
          rs.setJointGroupPositions(jmg, current_joints);
        else
          rs.setToDefaultValues();
      } else {
        rs.setToRandomPositions(jmg);
      }
      rs.update();

      if (!rs.setFromIK(jmg, target_pose, eef, 0.05 /* s per seed */)) continue;
      ++n_ok;

      std::vector<double> sol(jmg->getVariableCount());
      rs.copyJointGroupPositions(jmg, sol);

      // Apply ±2π wrap before computing travel.
      wrapSolution(jmg, current_joints, sol);

      double travel = 0.0;
      for (size_t j = 0; j < sol.size() && j < current_joints.size(); ++j)
        travel += std::abs(sol[j] - current_joints[j]);

      if (travel < best_travel) {
        best_travel = travel;
        best_joints = sol;
      }
      // Early exit: travel < 1 rad/joint is clearly the nearest branch.
      if (best_travel < static_cast<double>(jmg->getVariableCount())) break;
    }

    if (n_ok > 0) {
      RCLCPP_INFO(get_logger(),
                  "[%s] IK: %d/%d seeds succeeded, best branch Δjoints=%.3f rad",
                  phase.c_str(), n_ok, n_seeds, best_travel);
    } else {
      RCLCPP_WARN(get_logger(), "[%s] IK: all %d seeds failed", phase.c_str(), n_seeds);
    }
    return n_ok > 0;
  }

  // Log the joint names/values at the end-point of a planned trajectory.
  void logIkResult(const moveit::planning_interface::MoveGroupInterface::Plan & plan,
                   const std::string & phase, const std::string & planner)
  {
    const auto & jt = plan.trajectory_.joint_trajectory;
    if (jt.points.empty()) return;

    const auto & last = jt.points.back();
    const double dur =
      last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;

    std::string jvals;
    for (size_t i = 0; i < jt.joint_names.size() && i < last.positions.size(); ++i) {
      char buf[48];
      snprintf(buf, sizeof(buf), "%s:%.3f",
               jt.joint_names[i].c_str(), last.positions[i]);
      jvals += buf;
      if (i + 1 < jt.joint_names.size()) jvals += ' ';
    }
    RCLCPP_INFO(get_logger(),
                "[%s][%s] IK goal joints (dur=%.2fs): [%s]",
                phase.c_str(), planner.c_str(), dur, jvals.c_str());
  }

  rclcpp::Node::SharedPtr mgNode() { return mg_node_; }

private:
  // ===========================================================================
  // Helpers
  // ===========================================================================
  MoveGroupInterfacePtr armGroup(const std::string & arm) const
  {
    if (arm == "left")  return left_arm_;
    if (arm == "right") return right_arm_;
    return nullptr;
  }
  MoveGroupInterfacePtr gripGroup(const std::string & arm) const
  {
    if (arm == "left")  return left_grip_;
    if (arm == "right") return right_grip_;
    return nullptr;
  }

  bool inWorkspace(const geometry_msgs::msg::Pose & p) const
  {
    const double r = std::sqrt(p.position.x * p.position.x +
                               p.position.y * p.position.y);
    return r < workspace_radius_ &&
           p.position.z >= min_z_ &&
           p.position.z <= max_z_;
  }

  // Cartesian linear motion. Returns err::OK on success.
  int linearMoveTo(MoveGroupInterfacePtr arm,
                   const geometry_msgs::msg::Pose & target,
                   double speed_scale,
                   const std::string & phase)
  {
    if (stop_requested_.load()) return err::STOPPED_BY_USER;
    if (!inWorkspace(target))   return err::CAMERA_OUT_OF_RANGE;

    arm->setMaxVelocityScalingFactor(speed_scale);
    arm->setMaxAccelerationScalingFactor(speed_scale);
    arm->setPlanningTime(planning_time_s_);
    // Sync start state so computeCartesianPath seeds IK from the actual
    // post-execution joint configuration, not a cached pre-motion state.
    // Without this, after stop()+goto_home() the Cartesian IK may diverge and
    // trigger the jump-threshold, causing erratic descent behaviour.
    arm->setStartStateToCurrentState();

    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory traj;

    int attempts = plan_retry_count_ + 1;
    double fraction = 0.0;
    while (attempts-- > 0 && !stop_requested_.load()) {
      fraction = arm->computeCartesianPath(
        waypoints, cartesian_eef_step_, cartesian_jump_thresh_, traj);
      if (fraction >= cartesian_min_fraction_) break;
      RCLCPP_WARN(get_logger(),
                  "[%s] cartesian plan fraction=%.2f (jump_thresh=%.1f), retrying...",
                  phase.c_str(), fraction, cartesian_jump_thresh_);
    }
    if (fraction < cartesian_min_fraction_) {
      RCLCPP_WARN(get_logger(),
                  "[%s] cartesian FAILED (frac=%.2f, jump_thresh=%.1f) → "
                  "falling back to joint-space",
                  phase.c_str(), fraction, cartesian_jump_thresh_);
      last_failure_phase_ = phase;
      last_failure_reason_ = "cartesian fraction below threshold";
      return err::PLAN_FAILED;
    }
    RCLCPP_INFO(get_logger(), "[%s] cartesian OK (frac=%.2f, %zu pts)",
                phase.c_str(), fraction,
                traj.joint_trajectory.points.size());

    // Diagnostic: scan the planned trajectory for any large joint jump between
    // adjacent waypoints.  computeCartesianPath only checks jump_threshold on
    // the *raw* IK waypoints; after time-parameterization the trajectory still
    // contains those same joint values (interpolated), so a real discontinuity
    // will show up here as a large per-step delta.
    {
      const auto & pts = traj.joint_trajectory.points;
      double max_jump = 0.0;
      size_t max_jump_idx = 0;
      int max_jump_joint = -1;
      for (size_t i = 1; i < pts.size(); ++i) {
        for (size_t j = 0; j < pts[i].positions.size() &&
                            j < pts[i - 1].positions.size(); ++j) {
          double d = std::abs(pts[i].positions[j] - pts[i - 1].positions[j]);
          if (d > max_jump) {
            max_jump = d;
            max_jump_idx = i;
            max_jump_joint = static_cast<int>(j);
          }
        }
      }
      RCLCPP_INFO(get_logger(),
                  "[%s] cartesian max joint jump=%.4f rad (joint %d at pt %zu/%zu)",
                  phase.c_str(), max_jump, max_jump_joint,
                  max_jump_idx, pts.size());
      // If we somehow still passed planning but a > 0.5 rad jump exists between
      // adjacent (sub-mm-spaced) waypoints, refuse to execute — that path
      // would whip the arm at full controller speed and is unsafe.
      if (max_jump > 0.5) {
        RCLCPP_ERROR(get_logger(),
                     "[%s] REFUSING execute: %.3f rad joint jump between adjacent "
                     "trajectory points indicates IK discontinuity",
                     phase.c_str(), max_jump);
        last_failure_phase_ = phase;
        last_failure_reason_ = "IK discontinuity in cartesian trajectory";
        return err::PLAN_FAILED;
      }
    }

    auto rc = arm->execute(traj);
    if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[%s] execute FAILED (code=%d)",
                   phase.c_str(), rc.val);
      return err::EXECUTE_FAILED;
    }
    return err::OK;
  }

  // Joint-space motion to a pose (used for the longer transport segment where
  // a straight Cartesian line is unnecessary and may be infeasible).
  int jointMoveTo(MoveGroupInterfacePtr arm,
                  const geometry_msgs::msg::Pose & target,
                  double speed_scale,
                  const std::string & phase)
  {
    if (stop_requested_.load()) return err::STOPPED_BY_USER;
    if (!inWorkspace(target))   return err::CAMERA_OUT_OF_RANGE;

    arm->setMaxVelocityScalingFactor(speed_scale);
    arm->setMaxAccelerationScalingFactor(speed_scale);
    arm->setPlanningTime(planning_time_s_);
    // Sync start state so planning starts from the actual robot position.
    arm->setStartStateToCurrentState();
    const std::vector<double> seed = arm->getCurrentJointValues();

    // Multi-seed IK: try `seed` (current state) first, then random seeds, and
    // pick the solution with minimum joint travel.  This prevents KDL from
    // landing on the wrong IK branch (e.g. elbow-up vs elbow-down) when the
    // robot is at a configuration (like home = all-zeros) that biases toward a
    // distant branch.  A single-seed call via setJointValueTarget(pose, eef)
    // is unreliable because it always seeds from the last stored goal state.
    std::vector<double> ik_joints;
    const int kIkSeeds = 10;
    if (!solveIkBestBranch(arm, target, seed, kIkSeeds, ik_joints, phase)) {
      // Fail fast: if 10 seeded IK attempts all fail, the target pose is
      // unreachable.  Falling back to setPoseTarget() would only defer the
      // same IK failure to the planner, wasting ~45 s of PTP+RRTConnect
      // retries before giving the user the same answer.
      RCLCPP_ERROR(get_logger(),
                   "[%s] IK UNREACHABLE: all %d seeds failed — "
                   "pose is not solvable by this arm.",
                   phase.c_str(), kIkSeeds);
      logTargetPose(get_logger(), phase, target);
      last_failure_phase_ = phase;
      last_failure_reason_ = "IK unreachable (pose not solvable)";
      return err::PLAN_FAILED;
    }
    arm->setJointValueTarget(ik_joints);

    // Try Pilz PTP first: produces shortest joint-space path, avoids wild
    // rotations caused by OMPL picking a distant IK solution.  Fall back to
    // OMPL RRTConnect if Pilz cannot solve (e.g. singularity at start state).
    static const std::vector<std::pair<std::string, std::string>> kPipelines = {
      {"pilz_industrial_motion_planner", "PTP"},
      {"ompl",                           "RRTConnect"},
    };
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    for (const auto & [pipeline, planner] : kPipelines) {
      arm->setPlanningPipelineId(pipeline);
      arm->setPlannerId(planner);
      int attempts = plan_retry_count_ + 1;
      while (attempts-- > 0 && !stop_requested_.load()) {
        if (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          logIkResult(plan, phase, planner);
          if (arm->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            return err::OK;
          }
          RCLCPP_WARN(get_logger(), "[%s][%s] execute failed, retrying",
                      phase.c_str(), planner.c_str());
          continue;
        }
        RCLCPP_WARN(get_logger(), "[%s][%s] plan failed, retrying",
                    phase.c_str(), planner.c_str());
      }
      RCLCPP_WARN(get_logger(), "[%s] %s exhausted, trying next planner",
                  phase.c_str(), planner.c_str());
    }
    last_failure_phase_ = phase;
    last_failure_reason_ = "all planners exhausted (PTP + RRTConnect)";
    return err::PLAN_FAILED;
  }

  // Drive a finger joint to `target` metres.  Uses the gripper MoveGroup
  // (which is wired to the gripper controller through MoveIt).
  // compliance_grasp: close until contact; success if finger stalls with object held
  // (hardware layer switches to low-KP hold in openarm_hardware).
  int setGripper(MoveGroupInterfacePtr grip,
                 double target_m, double speed_scale,
                 bool compliance_grasp = false)
  {
    if (stop_requested_.load()) return err::STOPPED_BY_USER;
    grip->setMaxVelocityScalingFactor(speed_scale);
    grip->setMaxAccelerationScalingFactor(speed_scale);

    auto names = grip->getJointNames();
    std::vector<double> values(names.size(), target_m);
    grip->setJointValueTarget(values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (grip->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return err::PLAN_FAILED;
    }
    const auto exec_code = grip->execute(plan);
    if (exec_code == moveit::core::MoveItErrorCode::SUCCESS) {
      return err::OK;
    }
    if (compliance_grasp && isGripped(grip)) {
      return err::OK;
    }
    return err::EXECUTE_FAILED;
  }

  // True if any finger is still further away than `gripper_close_thresh_`
  // from fully closed -> object is held.
  bool isGripped(MoveGroupInterfacePtr grip) const
  {
    auto cur = grip->getCurrentJointValues();
    for (auto v : cur) {
      if (v > gripper_close_thresh_) return true;
    }
    return false;
  }

  // True if every finger is within `tolerance` of the configured open position.
  // Used by doPick to decide whether the (slow) open-gripper step at the hover
  // point can be skipped — if the gripper is already open we move straight to
  // the descend, saving ~0.5 s on a normal pick.
  bool isGripperOpen(MoveGroupInterfacePtr grip,
                     double tolerance_m = 0.005) const
  {
    auto cur = grip->getCurrentJointValues();
    if (cur.empty()) return false;
    const double thresh = gripper_open_pos_ - tolerance_m;
    for (auto v : cur) {
      if (v < thresh) return false;
    }
    return true;
  }

  geometry_msgs::msg::Pose offsetZ(const geometry_msgs::msg::Pose & in, double dz) const
  {
    auto out = in;
    out.position.z += dz;
    return out;
  }

  // Block-call the perception service.  Returns err::OK on success.
  int callPerception(const std::string & mode,
                     const std::string & target_name,
                     int target_index,
                     geometry_msgs::msg::Pose & out_pose)
  {
    if (!perception_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "perception service '%s' not available",
                  perception_srv_name_.c_str());
      return err::CAMERA_NO_CLOUD;
    }
    auto req = std::make_shared<openarm_skills::srv::DetectGraspPose::Request>();
    req->mode = mode;
    req->target_name = target_name;
    req->target_index = target_index;

    auto fut = perception_client_->async_send_request(req);
    if (fut.wait_for(std::chrono::duration<double>(perception_timeout_s_)) !=
        std::future_status::ready)
    {
      return err::PERCEPTION_TIMEOUT;
    }
    auto resp = fut.get();
    if (!resp->success) {
      return resp->result_code != 0 ? resp->result_code : err::CAMERA_NO_TARGET;
    }
    out_pose = resp->pose;
    if (!inWorkspace(out_pose)) return err::CAMERA_OUT_OF_RANGE;
    return err::OK;
  }

  // ===========================================================================
  // Pick / Place primitives
  // ===========================================================================
  int doPick(const std::string & arm,
             const geometry_msgs::msg::Pose & grasp,
             double approach, double retreat, double speed,
             const std::shared_ptr<PickPlaceGoalHandle> & gh)
  {
    auto a = armGroup(arm); auto g = gripGroup(arm);
    if (!a || !g) return err::BAD_REQUEST;

    // Approach uses the user's speed_scale capped at transport_speed_scale_ so
    // that a very slow speed_scale (e.g. 0.01) is visible, while a very high
    // one does not exceed the configured transport limit.
    const double approach_speed = std::min(speed, transport_speed_scale_);
    RCLCPP_INFO(get_logger(),
                "pick: speed_scale=%.3f  approach_speed=%.3f  transport_limit=%.3f",
                speed, approach_speed, transport_speed_scale_);

    // Move to the hover point first (no gripper change here): the user
    // requested that the gripper not be operated during transit to the hover
    // pose.  This keeps the approach trajectory deterministic regardless of
    // the gripper's prior state.
    publishFb(gh, "grasping", "pick.approach", 0.30, "moving above object");
    int rc = jointMoveTo(a, offsetZ(grasp, approach),
                          approach_speed, "pick.approach");
    if (rc) return rc;

    // At the hover point: only open the gripper if it is currently closed or
    // half-closed.  When it is already open we skip this step to save ~0.5 s
    // and to avoid a needless controller command.  This guarantees that the
    // subsequent descend always happens with the fingers spread, so the
    // gripper does not push the object away before contact.
    if (!isGripperOpen(g)) {
      publishFb(gh, "grasping", "pick.open_gripper", 0.40,
                "opening gripper at hover");
      const int og = setGripper(g, gripper_open_pos_, speed);
      if (og) {
        RCLCPP_WARN(get_logger(),
                    "pick.open_gripper failed (rc=%d), continuing anyway", og);
      }
    } else {
      RCLCPP_INFO(get_logger(),
                  "pick.open_gripper: gripper already open, skipping");
    }

    publishFb(gh, "grasping", "pick.descend", 0.45, "descending to grasp");
    rc = linearMoveTo(a, grasp, speed, "pick.descend");
    if (rc) {
      // Cartesian straight-line descent failed (IK fraction below threshold).
      // Fall back to joint-space planning which is less strict about linearity.
      RCLCPP_WARN(get_logger(),
                  "pick.descend cartesian failed, falling back to joint-space");
      rc = jointMoveTo(a, grasp, speed, "pick.descend.jnt");
      if (!rc) {
        // Joint fallback succeeded — clear the cartesian failure context so a
        // later phase (e.g. pick.close_gripper) is not mis-reported as descend.
        last_failure_phase_.clear();
        last_failure_reason_.clear();
      }
    }
    if (rc) return rc;

    publishFb(gh, "grasping", "pick.close_gripper", 0.55, "grasping object");
    rc = setGripper(g, gripper_closed_pos_, speed, true);
    if (rc) return rc;

    if (!isGripped(g)) {
      // With the new pick.open_gripper-first step the retry should rarely
      // trigger — if it still does, the object is genuinely outside grasp
      // range, so a lift/open/descend cycle is the right recovery.
      RCLCPP_WARN(get_logger(),
                  "pick: gripper closed empty after first attempt, retrying "
                  "lift→open→descend×%d", grasp_retry_count_);
      for (int i = 0; i < grasp_retry_count_ && !isGripped(g); ++i) {
        rc = linearMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retry_lift");
        if (rc) jointMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retry_lift.jnt");
        setGripper(g, gripper_open_pos_, speed);
        rc = linearMoveTo(a, grasp, speed, "pick.retry_descend");
        if (rc) jointMoveTo(a, grasp, speed, "pick.retry_descend.jnt");
        setGripper(g, gripper_closed_pos_, speed, true);
      }
      if (!isGripped(g)) return err::GRIP_NOT_HELD;
    }

    publishFb(gh, "grasping", "pick.retreat", 0.65, "lifting object");
    rc = linearMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retreat");
    if (rc) {
      RCLCPP_WARN(get_logger(),
                  "pick.retreat cartesian failed, falling back to joint-space");
      rc = jointMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retreat.jnt");
      if (!rc) {
        last_failure_phase_.clear();
        last_failure_reason_.clear();
      }
    }
    return rc;
  }

  int doPlace(const std::string & arm,
              const geometry_msgs::msg::Pose & place,
              double approach, double retreat, double speed,
              const std::shared_ptr<PickPlaceGoalHandle> & gh)
  {
    auto a = armGroup(arm); auto g = gripGroup(arm);
    if (!a || !g) return err::BAD_REQUEST;

    const double approach_speed = std::min(speed, transport_speed_scale_);
    RCLCPP_INFO(get_logger(),
                "place: speed_scale=%.3f  approach_speed=%.3f  transport_limit=%.3f",
                speed, approach_speed, transport_speed_scale_);

    publishFb(gh, "placing", "place.approach", 0.78, "moving above target");
    int rc = jointMoveTo(a, offsetZ(place, approach),
                          approach_speed, "place.approach");
    if (rc) return rc;

    publishFb(gh, "placing", "place.descend", 0.85, "descending to place");
    rc = linearMoveTo(a, place, speed, "place.descend");
    if (rc) {
      RCLCPP_WARN(get_logger(),
                  "place.descend cartesian failed, falling back to joint-space");
      rc = jointMoveTo(a, place, speed, "place.descend.jnt");
    }
    if (rc) return rc;

    publishFb(gh, "placing", "place.open_gripper", 0.92, "opening gripper");
    rc = setGripper(g, gripper_open_pos_, speed);
    if (rc) return rc;

    publishFb(gh, "placing", "place.retreat", 0.97, "lifting away");
    rc = linearMoveTo(a, offsetZ(place, retreat), speed, "place.retreat");
    if (rc) {
      RCLCPP_WARN(get_logger(),
                  "place.retreat cartesian failed, falling back to joint-space");
      rc = jointMoveTo(a, offsetZ(place, retreat), speed, "place.retreat.jnt");
    }
    return rc;
  }

  // ===========================================================================
  // Action handlers
  // ===========================================================================
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const PickPlaceAction::Goal> g)
  {
    if (g->arm != "left" && g->arm != "right") {
      RCLCPP_ERROR(get_logger(), "reject: arm must be 'left' or 'right'");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (g->pose_source != "camera" && g->pose_source != "upper_computer") {
      RCLCPP_ERROR(get_logger(), "reject: pose_source must be camera|upper_computer");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "reject: already busy with another goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<PickPlaceGoalHandle>)
  {
    stop_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccept(const std::shared_ptr<PickPlaceGoalHandle> gh)
  {
    std::thread([this, gh]() { execute(gh); }).detach();
  }

  void publishFb(const std::shared_ptr<PickPlaceGoalHandle> & gh,
                 const std::string & status,
                 const std::string & phase,
                 float progress,
                 const std::string & msg)
  {
    auto fb = std::make_shared<PickPlaceAction::Feedback>();
    fb->status = status;
    fb->phase  = phase;
    fb->progress = progress;
    fb->message = msg;
    gh->publish_feedback(fb);
  }

  void execute(const std::shared_ptr<PickPlaceGoalHandle> gh)
  {
    auto goal = gh->get_goal();
    auto result = std::make_shared<PickPlaceAction::Result>();

    stop_requested_.store(false);
    last_failure_phase_.clear();
    last_failure_reason_.clear();
    const double approach = goal->approach_offset_m > 0 ? goal->approach_offset_m
                                                        : approach_offset_m_;
    const double retreat  = goal->retreat_offset_m  > 0 ? goal->retreat_offset_m
                                                        : retreat_offset_m_;
    const double speed    = goal->speed_scale > 0 ? goal->speed_scale
                                                  : default_speed_scale_;

    geometry_msgs::msg::Pose grasp = goal->grasp_pose;
    geometry_msgs::msg::Pose place = goal->place_pose;

    // Normalize quaternions early; an un-normalized orientation fed to KDL IK
    // produces a corrupted rotation target and causes unpredictable arm motion.
    normalizeQuat(grasp, "grasp_pose", get_logger());
    normalizeQuat(place, "place_pose", get_logger());

    RCLCPP_INFO(get_logger(),
                "execute: grasp pos=[%.3f,%.3f,%.3f] ori=[%.4f,%.4f,%.4f,%.4f]  "
                "place pos=[%.3f,%.3f,%.3f] ori=[%.4f,%.4f,%.4f,%.4f]  "
                "speed=%.3f approach=%.3f retreat=%.3f",
                grasp.position.x, grasp.position.y, grasp.position.z,
                grasp.orientation.x, grasp.orientation.y,
                grasp.orientation.z, grasp.orientation.w,
                place.position.x, place.position.y, place.position.z,
                place.orientation.x, place.orientation.y,
                place.orientation.z, place.orientation.w,
                speed, approach, retreat);

    publishFb(gh, "perceiving", "pick.detect", 0.05, "resolving grasp pose");
    if (goal->pose_source == "camera") {
      int rc = callPerception("grasp", goal->target_name, goal->target_index, grasp);
      if (rc) return finish(gh, result, rc, "grasp perception failed");
      result->perceived_grasp_pose = grasp;
    } else if (!inWorkspace(grasp)) {
      return finish(gh, result, err::BAD_REQUEST, "grasp_pose outside workspace");
    }

    int rc = doPick(goal->arm, grasp, approach, retreat, speed, gh);
    if (rc) return finish(gh, result, rc, "pick failed");

    publishFb(gh, "transporting", "transport", 0.72, "moving to place region");
    if (goal->pose_source == "camera") {
      int prc = callPerception("place", goal->target_name, goal->target_index, place);
      if (prc) return finish(gh, result, prc, "place perception failed");
      result->perceived_place_pose = place;
    } else if (!inWorkspace(place)) {
      return finish(gh, result, err::BAD_REQUEST, "place_pose outside workspace");
    }

    rc = doPlace(goal->arm, place, approach, retreat, speed, gh);
    if (rc) return finish(gh, result, rc, "place failed");

    return finish(gh, result, err::OK, "pick_and_place done");
  }

  void finish(const std::shared_ptr<PickPlaceGoalHandle> & gh,
              std::shared_ptr<PickPlaceAction::Result> result,
              int code, const std::string & msg)
  {
    result->result_code = code;
    // Compose a descriptive message: base reason + the specific phase/cause
    // captured by the motion primitives.  This converts opaque "pick failed"
    // / "place failed" into actionable feedback like:
    //   "pick failed at pick.approach: IK unreachable (pose not solvable)"
    std::string full = msg;
    if (code != err::OK && !last_failure_phase_.empty()) {
      full += " at " + last_failure_phase_;
      if (!last_failure_reason_.empty()) {
        full += ": " + last_failure_reason_;
      }
    }
    result->message = full;
    if (code == err::OK) {
      result->success = true;
      result->status = "done";
      gh->succeed(result);
    } else if (code == err::STOPPED_BY_USER) {
      result->success = false;
      result->status = "stopped";
      gh->canceled(result);
    } else {
      result->success = false;
      result->status = "error";
      gh->abort(result);
    }
    last_failure_phase_.clear();
    last_failure_reason_.clear();
    busy_.store(false);
    stop_requested_.store(false);
  }

  // ===========================================================================
  // Service handlers
  // ===========================================================================
  void handleStop(const std::shared_ptr<openarm_skills::srv::Stop::Request>,
                  std::shared_ptr<openarm_skills::srv::Stop::Response> resp)
  {
    stop_requested_.store(true);
    if (left_arm_)   left_arm_->stop();
    if (right_arm_)  right_arm_->stop();
    if (left_grip_)  left_grip_->stop();
    if (right_grip_) right_grip_->stop();
    resp->success = true;
    resp->message = "stop signal sent";
  }

  void handleHome(const std::shared_ptr<openarm_skills::srv::GotoHome::Request> req,
                  std::shared_ptr<openarm_skills::srv::GotoHome::Response> resp)
  {
    const double speed = req->speed_scale > 0 ? req->speed_scale : transport_speed_scale_;

    auto run_one = [&](MoveGroupInterfacePtr arm) -> int {
      arm->setMaxVelocityScalingFactor(speed);
      arm->setMaxAccelerationScalingFactor(speed);
      arm->setStartStateToCurrentState();
      arm->setNamedTarget("home");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arm->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) return err::PLAN_FAILED;
      if (arm->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return err::EXECUTE_FAILED;
      // Sync state after execution so the next operation starts from the
      // correct (home) configuration rather than any cached pre-motion state.
      arm->setStartStateToCurrentState();
      return err::OK;
    };

    int rc = err::OK;
    if (req->arm == "left" || req->arm == "both")  rc = run_one(left_arm_);
    if (rc == err::OK && (req->arm == "right" || req->arm == "both")) rc = run_one(right_arm_);

    // Clear the stop flag set by handleStop so that subsequent service calls
    // (e.g. gripper) are not immediately rejected with STOPPED_BY_USER.
    // The next pick_place action also resets this flag at the start of execute().
    if (rc == err::OK) stop_requested_.store(false);

    resp->result_code = rc;
    resp->success = (rc == err::OK);
    resp->message = (rc == err::OK) ? "home reached" : "go-home failed";
  }

  void handleGripper(const std::shared_ptr<openarm_skills::srv::Gripper::Request> req,
                     std::shared_ptr<openarm_skills::srv::Gripper::Response> resp)
  {
    // If stop was called but no pick_place is currently active (busy_==false),
    // clear the flag so the standalone gripper command is not rejected.
    if (stop_requested_.load() && !busy_.load()) {
      stop_requested_.store(false);
    }
    auto g = gripGroup(req->arm);
    if (!g) {
      resp->success = false;
      resp->result_code = err::BAD_REQUEST;
      resp->message = "arm must be left|right";
      return;
    }
    double target = gripper_open_pos_;
    bool compliance_grasp = false;
    if (req->position > 0.0) target = req->position;
    else if (req->action == "close")       target = gripper_closed_pos_;
    else if (req->action == "half_close")  target = gripper_half_pos_;
    else if (req->action == "grasp") {
      target = gripper_closed_pos_;
      compliance_grasp = true;
    } else if (req->action == "open")        target = gripper_open_pos_;

    int rc = setGripper(g, target, default_speed_scale_, compliance_grasp);
    resp->result_code = rc;
    resp->success = (rc == err::OK);
    resp->message = (rc == err::OK) ? "gripper command done" : "gripper failed";
  }

  // ===========================================================================
  // Members
  // ===========================================================================
  std::string left_arm_group_, right_arm_group_;
  std::string left_grip_group_, right_grip_group_;
  std::string base_frame_;
  std::string left_eef_link_, right_eef_link_;
  std::string perception_srv_name_;
  double goal_pos_tol_, goal_ori_tol_;

  double default_speed_scale_, transport_speed_scale_;
  double cartesian_eef_step_, cartesian_jump_thresh_, cartesian_min_fraction_;
  double approach_offset_m_, retreat_offset_m_, step_timeout_s_, planning_time_s_;
  int    plan_retry_count_, grasp_retry_count_;
  double gripper_open_pos_, gripper_half_pos_, gripper_closed_pos_;
  double gripper_close_thresh_;
  double workspace_radius_, min_z_, max_z_;
  double perception_timeout_s_;

  rclcpp::Node::SharedPtr mg_node_;
  MoveGroupInterfacePtr left_arm_, right_arm_, left_grip_, right_grip_;

  rclcpp_action::Server<PickPlaceAction>::SharedPtr pick_place_server_;
  rclcpp::Service<openarm_skills::srv::Stop>::SharedPtr stop_srv_;
  rclcpp::Service<openarm_skills::srv::GotoHome>::SharedPtr home_srv_;
  rclcpp::Service<openarm_skills::srv::Gripper>::SharedPtr gripper_srv_;
  rclcpp::Client<openarm_skills::srv::DetectGraspPose>::SharedPtr perception_client_;

  std::atomic<bool> busy_{false};
  std::atomic<bool> stop_requested_{false};

  // Most-recent failure context captured by linearMoveTo/jointMoveTo so that
  // finish() can compose a phase-aware action-result message (e.g.
  // "pick failed at pick.approach: IK unreachable").  Cleared by finish().
  // Only accessed from the single-threaded action executor.
  std::string last_failure_phase_;
  std::string last_failure_reason_;
};

}  // namespace openarm_skills

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<openarm_skills::SkillServerNode>();
  node->initMoveGroups();

  // Spin both the skill node and the MGI helper node together, so MoveIt's
  // internal callbacks (joint_states / scene) keep flowing.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(node->mgNode());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
