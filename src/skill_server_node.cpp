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
//   Service : /openarm/carry_action               [openarm_skills/srv/CarryAction]
//   Service : /openarm/gripper                    [openarm_skills/srv/Gripper]
//   Client  : /openarm/detect_grasp_pose          [openarm_skills/srv/DetectGraspPose]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "openarm_skills/action/pick_place.hpp"
#include "openarm_skills/srv/stop.hpp"
#include "openarm_skills/srv/goto_home.hpp"
#include "openarm_skills/srv/carry_action.hpp"
#include "openarm_skills/srv/gripper.hpp"
#include "openarm_skills/srv/detect_grasp_pose.hpp"
#include "openarm_skills/error_codes.hpp"

namespace openarm_skills
{

using PickPlaceAction = openarm_skills::action::PickPlace;
using PickPlaceGoalHandle = rclcpp_action::ServerGoalHandle<PickPlaceAction>;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperCommand = control_msgs::action::GripperCommand;
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
    debug_assume_grasp_success_ = get("debug_assume_grasp_success", false);

    gripper_open_pos_       = get("gripper_open_pos",   0.040);
    gripper_half_pos_       = get("gripper_half_pos",   0.020);
    gripper_closed_pos_     = get("gripper_closed_pos", 0.0);
    gripper_close_thresh_   = get("gripper_close_threshold_m", 0.001);
    gripper_max_force_      = get("gripper_max_force_n", 40.0);
    gripper_default_speed_  = get("gripper_default_speed", 0.5);
    eef_obstacle_margin_m_  = get("eef_obstacle_margin_m", 0.02);

    workspace_radius_       = get("workspace_radius_m", 1.20);
    min_z_                  = get("min_z_m", -0.10);
    max_z_                  = get("max_z_m",  1.50);

    perception_srv_name_    = get("perception_service",
                                  std::string("/openarm/detect_grasp_pose"));
    perception_timeout_s_   = get("perception_timeout_s", 5.0);

    carry_joint4_rad_        = get("carry_joint4_rad", 1.6057);
    carry_joint6_rad_        = get("carry_joint6_rad", 0.0);
    carry_joint7_rad_        = get("carry_joint7_rad", 0.0);
    carry_default_width_m_   = get("carry_default_width_m", 0.307);

    // ---- service clients ---------------------------------------------------
    perception_client_ = this->create_client<openarm_skills::srv::DetectGraspPose>(
      perception_srv_name_);
    traj_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    left_arm_traj_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/left_joint_trajectory_controller/follow_joint_trajectory",
      traj_callback_group_);
    right_arm_traj_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/right_joint_trajectory_controller/follow_joint_trajectory",
      traj_callback_group_);
    left_gripper_cmd_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/left_gripper_controller/gripper_cmd", traj_callback_group_);
    right_gripper_cmd_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/right_gripper_controller/gripper_cmd", traj_callback_group_);
    left_gripper_aux_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/left_gripper_aux_controller/commands", 10);
    right_gripper_aux_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/right_gripper_aux_controller/commands", 10);

    // ---- service servers ---------------------------------------------------
    stop_srv_ = this->create_service<openarm_skills::srv::Stop>(
      "/openarm/stop",
      std::bind(&SkillServerNode::handleStop, this,
                std::placeholders::_1, std::placeholders::_2));

    home_srv_ = this->create_service<openarm_skills::srv::GotoHome>(
      "/openarm/goto_home",
      std::bind(&SkillServerNode::handleHome, this,
                std::placeholders::_1, std::placeholders::_2));

    carry_srv_ = this->create_service<openarm_skills::srv::CarryAction>(
      "/openarm/carry_action",
      std::bind(&SkillServerNode::handleCarry, this,
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

    double fk_default_width = 0.0;
    if (computeCarryWidth(0.0, fk_default_width)) {
      carry_default_width_m_ = fk_default_width;
    }
    computeCarryWidthRange(carry_min_width_m_, carry_max_width_m_);

    RCLCPP_INFO(get_logger(),
                "MoveGroup frame='%s' EEF: left=%s right=%s (planning_time=%.1fs)",
                base_frame_.c_str(), left_eef_link_.c_str(), right_eef_link_.c_str(),
                planning_time_s_);
    RCLCPP_INFO(get_logger(),
                "carry_action: default_width=%.3fm allowed_width=[%.3f, %.3f]m "
                "(joint4=%.4frad)",
                carry_default_width_m_, carry_min_width_m_, carry_max_width_m_,
                carry_joint4_rad_);
  }

  void configureArmMoveGroup(MoveGroupInterfacePtr arm,
                              const std::string & eef_link,
                              [[maybe_unused]] const std::string & label)
  {
    arm->setEndEffectorLink(eef_link);
    arm->setPoseReferenceFrame(base_frame_);
    arm->setPlanningTime(planning_time_s_);
    arm->setNumPlanningAttempts(5);
    arm->setGoalPositionTolerance(goal_pos_tol_);
    arm->setGoalOrientationTolerance(goal_ori_tol_);
    arm->setMaxVelocityScalingFactor(default_speed_scale_);
    arm->setMaxAccelerationScalingFactor(default_speed_scale_);
    // Avoid querying current state during early startup: some ros2_control
    // joint_state sources publish JointState with zero timestamp, which makes
    // MoveIt treat the state as "not recent" and log errors. We only need
    // current TCP for convenience, so skip it here.
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

  static double pointDistance(const geometry_msgs::msg::Point & a,
                              const geometry_msgs::msg::Point & b)
  {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static bool segmentIntersectsSphere(
      const geometry_msgs::msg::Point & a,
      const geometry_msgs::msg::Point & b,
      const geometry_msgs::msg::Point & center,
      double radius)
  {
    if (radius <= 0.0) return false;
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double abz = b.z - a.z;
    const double ab_len_sq = abx * abx + aby * aby + abz * abz;
    geometry_msgs::msg::Point closest = a;
    if (ab_len_sq > 1e-12) {
      double t = ((center.x - a.x) * abx +
                    (center.y - a.y) * aby +
                    (center.z - a.z) * abz) / ab_len_sq;
      t = std::max(0.0, std::min(1.0, t));
      closest.x = a.x + t * abx;
      closest.y = a.y + t * aby;
      closest.z = a.z + t * abz;
    }
    return pointDistance(closest, center) < radius;
  }

  geometry_msgs::msg::Pose makeAvoidanceViaPose(
      const geometry_msgs::msg::Pose & from,
      const geometry_msgs::msg::Pose & to,
      const geometry_msgs::msg::Point & center,
      double radius,
      double margin) const
  {
    geometry_msgs::msg::Pose via = to;
    via.orientation = from.orientation;
    const double shell = radius + margin;
    via.position.x = 0.5 * (from.position.x + to.position.x);
    via.position.y = 0.5 * (from.position.y + to.position.y);
    via.position.z = std::max({from.position.z, to.position.z, center.z + shell});

    const double dx = via.position.x - center.x;
    const double dy = via.position.y - center.y;
    const double horiz = std::sqrt(dx * dx + dy * dy);
    if (horiz < shell) {
      if (horiz > 1e-6) {
        const double scale = shell / horiz;
        via.position.x = center.x + dx * scale;
        via.position.y = center.y + dy * scale;
      } else {
        via.position.x = center.x + shell;
        via.position.y = center.y;
      }
    }
    return via;
  }

  double effectiveApproachOffset(double approach, double target_radius) const
  {
    if (target_radius <= 0.0) return approach;
    const double min_hover_dist = target_radius + eef_obstacle_margin_m_;
    const double hover_dist = std::max(approach, min_hover_dist);
    if (hover_dist > approach) {
      RCLCPP_INFO(get_logger(),
                  "pick: raising approach_offset %.3f -> %.3f m "
                  "(target_radius=%.3f + margin=%.3f)",
                  approach, hover_dist, target_radius, eef_obstacle_margin_m_);
    }
    return hover_dist;
  }

  // Joint-space motion that inserts a via pose when the straight EEF segment
  // would pass through the forbidden sphere around the grasp target.
  int jointMoveToAvoid(MoveGroupInterfacePtr arm,
                       const geometry_msgs::msg::Pose & target,
                       double speed_scale,
                       const std::string & phase,
                       const geometry_msgs::msg::Point & obstacle_center,
                       double obstacle_radius)
  {
    if (obstacle_radius <= 0.0) {
      return jointMoveTo(arm, target, speed_scale, phase);
    }

    const double shell = obstacle_radius + eef_obstacle_margin_m_;
    if (pointDistance(target.position, obstacle_center) < shell) {
      RCLCPP_WARN(get_logger(),
                  "[%s] target inside obstacle shell (dist=%.3f < %.3f), "
                  "planning without via",
                  phase.c_str(),
                  pointDistance(target.position, obstacle_center), shell);
      return jointMoveTo(arm, target, speed_scale, phase);
    }

    const auto current = arm->getCurrentPose(arm->getEndEffectorLink());
    if (!segmentIntersectsSphere(current.pose.position, target.position,
                                 obstacle_center, shell)) {
      return jointMoveTo(arm, target, speed_scale, phase);
    }

    const auto via = makeAvoidanceViaPose(
      current.pose, target, obstacle_center, obstacle_radius, eef_obstacle_margin_m_);
    RCLCPP_INFO(get_logger(),
                "[%s] EEF path crosses target sphere (r=%.3f), inserting via "
                "pos=[%.3f,%.3f,%.3f]",
                phase.c_str(), obstacle_radius,
                via.position.x, via.position.y, via.position.z);
    int rc = jointMoveTo(arm, via, speed_scale, phase + ".via");
    if (rc) {
      RCLCPP_WARN(get_logger(),
                  "[%s] via pose unreachable, falling back to direct motion",
                  phase.c_str());
      return jointMoveTo(arm, target, speed_scale, phase);
    }
    return jointMoveTo(arm, target, speed_scale, phase);
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

  rclcpp_action::Client<GripperCommand>::SharedPtr gripperCommandClient(
      const std::string & arm) const
  {
    if (arm == "left") return left_gripper_cmd_client_;
    if (arm == "right") return right_gripper_cmd_client_;
    return nullptr;
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripperAuxPublisher(
      const std::string & arm) const
  {
    if (arm == "left") return left_gripper_aux_pub_;
    if (arm == "right") return right_gripper_aux_pub_;
    return nullptr;
  }

  double normalizedGripperSpeed(double speed) const
  {
    const double requested = (std::isfinite(speed) && speed > 0.0)
      ? speed
      : gripper_default_speed_;
    return std::max(0.05, std::min(1.0, requested));
  }

  double normalizedGripperForce(double force) const
  {
    if (!std::isfinite(force) || force <= 0.0) return 0.0;
    return std::min(force, gripper_max_force_);
  }

  void publishGripperAuxCommand(const std::string & arm,
                                double speed,
                                double force)
  {
    auto pub = gripperAuxPublisher(arm);
    if (!pub) return;
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {normalizedGripperSpeed(speed), normalizedGripperForce(force)};
    pub->publish(msg);
    RCLCPP_INFO(get_logger(), "gripper(%s): aux speed=%.2f force=%.2f",
                arm.c_str(), msg.data[0], msg.data[1]);
  }

  int sendGripperCommand(const std::string & arm,
                         double target_m,
                         double max_effort,
                         double speed,
                         bool compliance_grasp,
                         MoveGroupInterfacePtr grip)
  {
    auto client = gripperCommandClient(arm);
    if (!client) return err::BAD_REQUEST;
    if (!client->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "gripper(%s): action server unavailable", arm.c_str());
      return err::ACTION_TIMEOUT;
    }

    GripperCommand::Goal goal;
    goal.command.position = target_m;
    goal.command.max_effort = max_effort;
    publishGripperAuxCommand(arm, speed, max_effort);

    RCLCPP_INFO(get_logger(),
                "gripper(%s): target=%.4fm max_effort=%.2f speed=%.2f",
                arm.c_str(), target_m, max_effort, normalizedGripperSpeed(speed));
    auto goal_future = client->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "gripper(%s): timed out waiting for goal acceptance",
                   arm.c_str());
      return err::ACTION_TIMEOUT;
    }

    auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "gripper(%s): goal rejected", arm.c_str());
      return err::EXECUTE_FAILED;
    }

    auto result_future = client->async_get_result(goal_handle);
    const auto timeout = std::chrono::duration<double>(std::max(1.0, step_timeout_s_));
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "gripper(%s): timed out waiting for result", arm.c_str());
      return err::ACTION_TIMEOUT;
    }

    const auto wrapped = result_future.get();
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result) {
      const bool ok = wrapped.result->reached_goal || wrapped.result->stalled ||
                      (compliance_grasp && isGripped(grip));
      if (ok) return err::OK;
    }
    if (compliance_grasp && isGripped(grip)) {
      return err::OK;
    }

    RCLCPP_ERROR(get_logger(), "gripper(%s): controller reported failure", arm.c_str());
    return err::EXECUTE_FAILED;
  }

  // Drive a finger joint to `target` metres.  With force > 0, send the
  // controller's GripperCommand action directly so max_effort is preserved.
  // Otherwise use the existing MoveIt position path.
  // compliance_grasp: close until contact; success if finger stalls with object held
  // (hardware layer switches to low-KP hold in openarm_hardware).
  int setGripper(MoveGroupInterfacePtr grip,
                 const std::string & arm,
                 double target_m, double speed_scale,
                 bool compliance_grasp = false,
                 double force = 0.0,
                 double gripper_speed = 0.0)
  {
    if (stop_requested_.load()) return err::STOPPED_BY_USER;
    const double max_effort = normalizedGripperForce(force);
    (void)speed_scale;
    const double speed = normalizedGripperSpeed(gripper_speed);
    if (max_effort > 0.0 || gripper_speed > 0.0) {
      return sendGripperCommand(arm, target_m, max_effort, speed, compliance_grasp, grip);
    }

    publishGripperAuxCommand(arm, speed, max_effort);
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

  // Drive both grippers to the same target in parallel so left/right fingers
  // open or close together (used by carry_action after synchronized arm motion).
  int setBothGrippers(double target_m, double speed_scale,
                      bool compliance_grasp = false,
                      double force = 0.0,
                      double gripper_speed = 0.0)
  {
    if (stop_requested_.load()) return err::STOPPED_BY_USER;
    const double max_effort = normalizedGripperForce(force);
    const double speed = normalizedGripperSpeed(gripper_speed);

    if (max_effort <= 0.0 && gripper_speed <= 0.0) {
      int rc = setGripper(left_grip_, "left", target_m, speed_scale,
                          compliance_grasp, force, gripper_speed);
      if (rc != err::OK) return rc;
      return setGripper(right_grip_, "right", target_m, speed_scale,
                        compliance_grasp, force, gripper_speed);
    }

    auto left_client = gripperCommandClient("left");
    auto right_client = gripperCommandClient("right");
    if (!left_client || !right_client) return err::BAD_REQUEST;
    if (!left_client->wait_for_action_server(std::chrono::seconds(2)) ||
        !right_client->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "carry_action: gripper action server unavailable");
      return err::ACTION_TIMEOUT;
    }

    publishGripperAuxCommand("left", speed, max_effort);
    publishGripperAuxCommand("right", speed, max_effort);

    GripperCommand::Goal left_goal;
    GripperCommand::Goal right_goal;
    left_goal.command.position = target_m;
    left_goal.command.max_effort = max_effort;
    right_goal.command.position = target_m;
    right_goal.command.max_effort = max_effort;

    RCLCPP_INFO(get_logger(),
                "carry_action: sending synchronized gripper goals "
                "(target=%.4fm max_effort=%.2f speed=%.2f)",
                target_m, max_effort, speed);

    auto left_goal_future = left_client->async_send_goal(left_goal);
    auto right_goal_future = right_client->async_send_goal(right_goal);

    if (left_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
        right_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(),
                   "carry_action: timed out waiting for gripper goal acceptance");
      return err::ACTION_TIMEOUT;
    }

    auto left_handle = left_goal_future.get();
    auto right_handle = right_goal_future.get();
    if (!left_handle || !right_handle) {
      RCLCPP_ERROR(get_logger(), "carry_action: gripper goal rejected");
      return err::EXECUTE_FAILED;
    }

    auto left_result_future = left_client->async_get_result(left_handle);
    auto right_result_future = right_client->async_get_result(right_handle);
    const auto timeout = std::chrono::duration<double>(std::max(1.0, step_timeout_s_));
    if (left_result_future.wait_for(timeout) != std::future_status::ready ||
        right_result_future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(),
                   "carry_action: timed out waiting for gripper result");
      return err::ACTION_TIMEOUT;
    }

    const auto left_wrapped = left_result_future.get();
    const auto right_wrapped = right_result_future.get();

    auto gripper_ok = [&](const auto & wrapped, MoveGroupInterfacePtr grip) -> bool {
      if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result) {
        return wrapped.result->reached_goal || wrapped.result->stalled ||
               (compliance_grasp && isGripped(grip));
      }
      return compliance_grasp && isGripped(grip);
    };

    if (!gripper_ok(left_wrapped, left_grip_) || !gripper_ok(right_wrapped, right_grip_)) {
      RCLCPP_ERROR(get_logger(), "carry_action: gripper controller reported failure");
      return err::EXECUTE_FAILED;
    }
    return err::OK;
  }

  double currentGripperTargetOrClosed(MoveGroupInterfacePtr grip) const
  {
    if (!grip) return gripper_closed_pos_;

    auto cur = grip->getCurrentJointValues();
    if (cur.empty()) return gripper_closed_pos_;

    double sum = 0.0;
    for (auto v : cur) sum += v;
    double target = sum / static_cast<double>(cur.size());
    target = std::max(gripper_closed_pos_, std::min(gripper_open_pos_, target));
    return target;
  }

  int moveArmToJoints(MoveGroupInterfacePtr arm,
                      const std::vector<double> & joints,
                      double speed_scale,
                      const std::string & phase)
  {
    if (stop_requested_.load()) return err::STOPPED_BY_USER;
    if (!arm || joints.empty()) return err::BAD_REQUEST;

    arm->setMaxVelocityScalingFactor(speed_scale);
    arm->setMaxAccelerationScalingFactor(speed_scale);
    arm->setPlanningTime(planning_time_s_);
    arm->setStartStateToCurrentState();
    arm->setJointValueTarget(joints);

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
            arm->setStartStateToCurrentState();
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
    last_failure_reason_ = "joint return planners exhausted (PTP + RRTConnect)";
    return err::PLAN_FAILED;
  }

  int moveArmHome(MoveGroupInterfacePtr arm, double speed_scale, const std::string & phase)
  {
    if (stop_requested_.load()) return err::STOPPED_BY_USER;
    if (!arm) return err::BAD_REQUEST;

    arm->setMaxVelocityScalingFactor(speed_scale);
    arm->setMaxAccelerationScalingFactor(speed_scale);
    arm->setPlanningTime(planning_time_s_);
    arm->setStartStateToCurrentState();
    arm->setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      last_failure_phase_ = phase;
      last_failure_reason_ = "home plan failed";
      return err::PLAN_FAILED;
    }
    if (arm->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      last_failure_phase_ = phase;
      last_failure_reason_ = "home execute failed";
      return err::EXECUTE_FAILED;
    }
    arm->setStartStateToCurrentState();
    return err::OK;
  }

  bool makeHomeTrajectoryGoal(MoveGroupInterfacePtr arm,
                              double speed_scale,
                              FollowJointTrajectory::Goal & goal,
                              double & duration_s,
                              const std::string & label)
  {
    if (!arm) return false;

    const auto * jmg =
      arm->getRobotModel()->getJointModelGroup(arm->getName());
    if (!jmg) return false;

    moveit::core::RobotState target_state(arm->getRobotModel());
    target_state.setToDefaultValues();
    target_state.setToDefaultValues(jmg, "home");

    std::vector<double> positions;
    target_state.copyJointGroupPositions(jmg, positions);
    const auto names = arm->getJointNames();
    const auto current = arm->getCurrentJointValues();
    if (positions.size() != names.size()) {
      RCLCPP_ERROR(get_logger(),
                   "goto_home(%s): home joint count mismatch (%zu positions vs %zu names)",
                   label.c_str(), positions.size(), names.size());
      return false;
    }
    if (current.size() != positions.size()) {
      RCLCPP_ERROR(get_logger(),
                   "goto_home(%s): current joint count mismatch (%zu current vs %zu target)",
                   label.c_str(), current.size(), positions.size());
      return false;
    }

    goal.trajectory.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities.assign(positions.size(), 0.0);

    const double clamped_speed = std::max(0.05, std::min(1.0, speed_scale));
    duration_s = 0.0;
    const auto & joint_models = jmg->getActiveJointModels();
    for (size_t i = 0; i < positions.size() && i < current.size(); ++i) {
      double max_vel = 1.0;
      if (i < joint_models.size()) {
        const auto & bounds = joint_models[i]->getVariableBounds();
        if (!bounds.empty() && bounds[0].velocity_bounded_ &&
            bounds[0].max_velocity_ > 1e-6) {
          max_vel = bounds[0].max_velocity_;
        }
      }
      duration_s = std::max(duration_s,
                            std::abs(positions[i] - current[i]) / (max_vel * clamped_speed));
    }
    duration_s = std::max(0.5, duration_s);
    point.time_from_start.sec = static_cast<int32_t>(duration_s);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((duration_s - point.time_from_start.sec) * 1e9);
    goal.trajectory.points.push_back(point);
    return true;
  }

  void setTrajectoryDuration(FollowJointTrajectory::Goal & goal, double duration_s)
  {
    if (goal.trajectory.points.empty()) return;
    auto & point = goal.trajectory.points.front();
    point.time_from_start.sec = static_cast<int32_t>(duration_s);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((duration_s - point.time_from_start.sec) * 1e9);
  }

  bool setNamedJoint(std::vector<double> & positions,
                     const std::vector<std::string> & names,
                     const std::string & joint_name,
                     double value) const
  {
    for (size_t i = 0; i < names.size() && i < positions.size(); ++i) {
      if (names[i] == joint_name) {
        positions[i] = value;
        return true;
      }
    }
    RCLCPP_ERROR(get_logger(), "carry_action: joint '%s' not found", joint_name.c_str());
    return false;
  }

  bool makeCarryPositions(MoveGroupInterfacePtr arm,
                          const std::string & side,
                          double theta,
                          std::vector<double> & positions) const
  {
    if (!arm) return false;
    const auto * jmg =
      arm->getRobotModel()->getJointModelGroup(arm->getName());
    if (!jmg) return false;

    moveit::core::RobotState target_state(arm->getRobotModel());
    target_state.setToDefaultValues();
    target_state.setToDefaultValues(jmg, "home");
    target_state.copyJointGroupPositions(jmg, positions);

    const auto names = arm->getJointNames();
    if (positions.size() != names.size()) {
      RCLCPP_ERROR(get_logger(),
                   "carry_action(%s): joint count mismatch (%zu positions vs %zu names)",
                   side.c_str(), positions.size(), names.size());
      return false;
    }

    const double side_theta = (side == "left") ? theta : -theta;
    const std::string prefix = "openarm_" + side + "_joint";
    return setNamedJoint(positions, names, prefix + "2", side_theta) &&
           setNamedJoint(positions, names, prefix + "4", carry_joint4_rad_) &&
           setNamedJoint(positions, names, prefix + "5", side_theta) &&
           setNamedJoint(positions, names, prefix + "6", carry_joint6_rad_) &&
           setNamedJoint(positions, names, prefix + "7", carry_joint7_rad_);
  }

  bool getJointBounds(MoveGroupInterfacePtr arm,
                      const std::string & joint_name,
                      double & lower,
                      double & upper) const
  {
    if (!arm) return false;
    const auto * jmg =
      arm->getRobotModel()->getJointModelGroup(arm->getName());
    if (!jmg) return false;

    for (const auto * joint_model : jmg->getActiveJointModels()) {
      if (joint_model->getName() != joint_name) continue;
      const auto & bounds = joint_model->getVariableBounds();
      if (bounds.empty() || !bounds[0].position_bounded_) return false;
      lower = bounds[0].min_position_;
      upper = bounds[0].max_position_;
      return true;
    }
    return false;
  }

  bool carryThetaLimits(double & lower, double & upper) const
  {
    lower = -std::numeric_limits<double>::infinity();
    upper = std::numeric_limits<double>::infinity();

    auto add_bound = [&](MoveGroupInterfacePtr arm,
                         const std::string & joint_name,
                         double sign) -> bool {
      double joint_lower = 0.0;
      double joint_upper = 0.0;
      if (!getJointBounds(arm, joint_name, joint_lower, joint_upper)) {
        RCLCPP_ERROR(get_logger(), "carry_action: missing bounds for %s",
                     joint_name.c_str());
        return false;
      }
      if (sign > 0.0) {
        lower = std::max(lower, joint_lower);
        upper = std::min(upper, joint_upper);
      } else {
        lower = std::max(lower, -joint_upper);
        upper = std::min(upper, -joint_lower);
      }
      return true;
    };

    if (!add_bound(left_arm_,  "openarm_left_joint2",  -1.0) ||
        !add_bound(right_arm_, "openarm_right_joint2",  1.0) ||
        !add_bound(left_arm_,  "openarm_left_joint5",  -1.0) ||
        !add_bound(right_arm_, "openarm_right_joint5",  1.0)) {
      return false;
    }
    return lower <= upper;
  }

  bool computeCarryWidth(double theta, double & width_m) const
  {
    if (!left_arm_ || !right_arm_) return false;

    std::vector<double> left_positions;
    std::vector<double> right_positions;
    if (!makeCarryPositions(left_arm_, "left", theta, left_positions) ||
        !makeCarryPositions(right_arm_, "right", theta, right_positions)) {
      return false;
    }

    auto model = left_arm_->getRobotModel();
    const auto * left_jmg = model->getJointModelGroup(left_arm_->getName());
    const auto * right_jmg = model->getJointModelGroup(right_arm_->getName());
    if (!left_jmg || !right_jmg) return false;

    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    state.setJointGroupPositions(left_jmg, left_positions);
    state.setJointGroupPositions(right_jmg, right_positions);
    state.update();

    const auto & left_tf = state.getGlobalLinkTransform(left_eef_link_);
    const auto & right_tf = state.getGlobalLinkTransform(right_eef_link_);
    width_m = std::abs(left_tf.translation().y() - right_tf.translation().y());
    return std::isfinite(width_m);
  }

  bool computeCarryWidthRange(double & min_width_m, double & max_width_m) const
  {
    double theta_lower = 0.0;
    double theta_upper = 0.0;
    if (!carryThetaLimits(theta_lower, theta_upper)) return false;

    min_width_m = std::numeric_limits<double>::infinity();
    max_width_m = 0.0;
    constexpr int kSamples = 200;
    for (int i = 0; i <= kSamples; ++i) {
      const double t = theta_lower +
        (theta_upper - theta_lower) * static_cast<double>(i) / kSamples;
      double width = 0.0;
      if (!computeCarryWidth(t, width)) return false;
      min_width_m = std::min(min_width_m, width);
      max_width_m = std::max(max_width_m, width);
    }
    return std::isfinite(min_width_m) && std::isfinite(max_width_m);
  }

  bool solveCarryThetaForWidth(double requested_width_m,
                               double & theta,
                               double & actual_width_m,
                               std::string & message) const
  {
    if (!std::isfinite(requested_width_m) || requested_width_m < 0.0) {
      message = "width must be finite and >= 0";
      return false;
    }

    if (requested_width_m == 0.0) {
      theta = 0.0;
      if (!computeCarryWidth(theta, actual_width_m)) {
        message = "failed to compute default carry width";
        return false;
      }
      return true;
    }

    double theta_lower = 0.0;
    double theta_upper = 0.0;
    if (!carryThetaLimits(theta_lower, theta_upper)) {
      message = "failed to compute carry joint limits";
      return false;
    }

    double min_width = 0.0;
    double max_width = 0.0;
    if (!computeCarryWidthRange(min_width, max_width)) {
      message = "failed to compute carry width range";
      return false;
    }

    constexpr double kWidthTolerance = 0.003;
    if (requested_width_m < min_width - kWidthTolerance ||
        requested_width_m > max_width + kWidthTolerance) {
      char buf[160];
      snprintf(buf, sizeof(buf),
               "width %.3fm out of range [%.3f, %.3f]m",
               requested_width_m, min_width, max_width);
      message = buf;
      return false;
    }

    double best_theta = theta_lower;
    double best_width = 0.0;
    double best_err = std::numeric_limits<double>::infinity();
    bool have_prev = false;
    double prev_theta = 0.0;
    double prev_width = 0.0;

    constexpr int kSamples = 200;
    for (int i = 0; i <= kSamples; ++i) {
      const double t = theta_lower +
        (theta_upper - theta_lower) * static_cast<double>(i) / kSamples;
      double width = 0.0;
      if (!computeCarryWidth(t, width)) {
        message = "failed to compute carry width";
        return false;
      }

      const double err = std::abs(width - requested_width_m);
      if (err < best_err) {
        best_err = err;
        best_theta = t;
        best_width = width;
      }

      if (have_prev &&
          (prev_width - requested_width_m) * (width - requested_width_m) <= 0.0) {
        double lo = prev_theta;
        double hi = t;
        double w_lo = prev_width;
        double w_hi = width;
        for (int iter = 0; iter < 40; ++iter) {
          const double mid = 0.5 * (lo + hi);
          double w_mid = 0.0;
          if (!computeCarryWidth(mid, w_mid)) break;
          if ((w_lo - requested_width_m) * (w_mid - requested_width_m) <= 0.0) {
            hi = mid;
            w_hi = w_mid;
          } else {
            lo = mid;
            w_lo = w_mid;
          }
        }
        const double candidate_theta = 0.5 * (lo + hi);
        const double candidate_width = 0.5 * (w_lo + w_hi);
        const double candidate_err = std::abs(candidate_width - requested_width_m);
        if (candidate_err < best_err) {
          best_err = candidate_err;
          best_theta = candidate_theta;
          best_width = candidate_width;
        }
      }

      have_prev = true;
      prev_theta = t;
      prev_width = width;
    }

    if (best_err > kWidthTolerance) {
      char buf[160];
      snprintf(buf, sizeof(buf),
               "width %.3fm cannot be solved within %.3fm tolerance",
               requested_width_m, kWidthTolerance);
      message = buf;
      return false;
    }

    theta = best_theta;
    actual_width_m = best_width;
    return true;
  }

  bool makeJointTrajectoryGoal(MoveGroupInterfacePtr arm,
                               const std::vector<double> & positions,
                               double speed_scale,
                               FollowJointTrajectory::Goal & goal,
                               double & duration_s,
                               const std::string & label,
                               const std::string & command_name)
  {
    if (!arm) return false;
    const auto * jmg =
      arm->getRobotModel()->getJointModelGroup(arm->getName());
    if (!jmg) return false;

    const auto names = arm->getJointNames();
    const auto current = arm->getCurrentJointValues();
    if (positions.size() != names.size() || current.size() != positions.size()) {
      RCLCPP_ERROR(get_logger(),
                   "%s(%s): joint count mismatch (%zu positions, %zu names, %zu current)",
                   command_name.c_str(), label.c_str(), positions.size(), names.size(),
                   current.size());
      return false;
    }

    goal.trajectory.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities.assign(positions.size(), 0.0);

    const double clamped_speed = std::max(0.05, std::min(1.0, speed_scale));
    duration_s = 0.0;
    const auto & joint_models = jmg->getActiveJointModels();
    for (size_t i = 0; i < positions.size() && i < current.size(); ++i) {
      double max_vel = 1.0;
      if (i < joint_models.size()) {
        const auto & bounds = joint_models[i]->getVariableBounds();
        if (!bounds.empty() && bounds[0].velocity_bounded_ &&
            bounds[0].max_velocity_ > 1e-6) {
          max_vel = bounds[0].max_velocity_;
        }
      }
      duration_s = std::max(duration_s,
                            std::abs(positions[i] - current[i]) / (max_vel * clamped_speed));
    }
    duration_s = std::max(0.5, duration_s);
    point.time_from_start.sec = static_cast<int32_t>(duration_s);
    point.time_from_start.nanosec =
      static_cast<uint32_t>((duration_s - point.time_from_start.sec) * 1e9);
    goal.trajectory.points.push_back(point);
    return true;
  }

  int executeBothHomeWithControllers(double speed_scale)
  {
    if (!left_arm_traj_client_->wait_for_action_server(std::chrono::seconds(2)) ||
        !right_arm_traj_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(), "goto_home(both): arm trajectory controller unavailable");
      return err::EXECUTE_FAILED;
    }

    FollowJointTrajectory::Goal left_goal;
    FollowJointTrajectory::Goal right_goal;
    double left_duration_s = 0.0;
    double right_duration_s = 0.0;
    if (!makeHomeTrajectoryGoal(left_arm_, speed_scale, left_goal, left_duration_s, "left") ||
        !makeHomeTrajectoryGoal(right_arm_, speed_scale, right_goal, right_duration_s, "right")) {
      return err::PLAN_FAILED;
    }
    const double sync_duration_s = std::max(left_duration_s, right_duration_s);
    setTrajectoryDuration(left_goal, sync_duration_s);
    setTrajectoryDuration(right_goal, sync_duration_s);

    RCLCPP_INFO(get_logger(),
                "goto_home(both): sending synchronized home goals "
                "(left=%.2fs right=%.2fs sync=%.2fs speed=%.2f)",
                left_duration_s, right_duration_s, sync_duration_s, speed_scale);
    auto left_goal_future = left_arm_traj_client_->async_send_goal(left_goal);
    auto right_goal_future = right_arm_traj_client_->async_send_goal(right_goal);

    if (left_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
        right_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "goto_home(both): timed out waiting for goal acceptance");
      return err::EXECUTE_FAILED;
    }

    auto left_handle = left_goal_future.get();
    auto right_handle = right_goal_future.get();
    if (!left_handle || !right_handle) {
      RCLCPP_ERROR(get_logger(), "goto_home(both): controller rejected home goal");
      return err::EXECUTE_FAILED;
    }

    auto left_result_future = left_arm_traj_client_->async_get_result(left_handle);
    auto right_result_future = right_arm_traj_client_->async_get_result(right_handle);
    if (left_result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready ||
        right_result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "goto_home(both): timed out waiting for execution result");
      return err::EXECUTE_FAILED;
    }

    const auto left_wrapped = left_result_future.get();
    const auto right_wrapped = right_result_future.get();
    const bool left_ok =
      left_wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      left_wrapped.result &&
      left_wrapped.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
    const bool right_ok =
      right_wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      right_wrapped.result &&
      right_wrapped.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
    if (!left_ok || !right_ok) {
      RCLCPP_ERROR(get_logger(), "goto_home(both): controller execution failed");
      return err::EXECUTE_FAILED;
    }

    left_arm_->setStartStateToCurrentState();
    right_arm_->setStartStateToCurrentState();
    return err::OK;
  }

  int executeBothCarryWithControllers(double requested_width_m,
                                      double speed_scale,
                                      double & actual_width_m,
                                      std::string & message)
  {
    if (!left_arm_traj_client_->wait_for_action_server(std::chrono::seconds(2)) ||
        !right_arm_traj_client_->wait_for_action_server(std::chrono::seconds(2))) {
      message = "carry_action: arm trajectory controller unavailable";
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return err::EXECUTE_FAILED;
    }

    double theta = 0.0;
    if (!solveCarryThetaForWidth(requested_width_m, theta, actual_width_m, message)) {
      return err::BAD_REQUEST;
    }

    std::vector<double> left_positions;
    std::vector<double> right_positions;
    if (!makeCarryPositions(left_arm_, "left", theta, left_positions) ||
        !makeCarryPositions(right_arm_, "right", theta, right_positions)) {
      message = "carry_action: failed to build target joints";
      return err::PLAN_FAILED;
    }

    FollowJointTrajectory::Goal left_goal;
    FollowJointTrajectory::Goal right_goal;
    double left_duration_s = 0.0;
    double right_duration_s = 0.0;
    if (!makeJointTrajectoryGoal(left_arm_, left_positions, speed_scale, left_goal,
                                 left_duration_s, "left", "carry_action") ||
        !makeJointTrajectoryGoal(right_arm_, right_positions, speed_scale, right_goal,
                                 right_duration_s, "right", "carry_action")) {
      message = "carry_action: failed to build trajectory goal";
      return err::PLAN_FAILED;
    }

    const double sync_duration_s = std::max(left_duration_s, right_duration_s);
    setTrajectoryDuration(left_goal, sync_duration_s);
    setTrajectoryDuration(right_goal, sync_duration_s);

    RCLCPP_INFO(get_logger(),
                "carry_action: sending synchronized carry goals "
                "(width=%.3fm theta=%.4frad left=%.2fs right=%.2fs sync=%.2fs speed=%.2f)",
                actual_width_m, theta, left_duration_s, right_duration_s,
                sync_duration_s, speed_scale);
    auto left_goal_future = left_arm_traj_client_->async_send_goal(left_goal);
    auto right_goal_future = right_arm_traj_client_->async_send_goal(right_goal);

    if (left_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready ||
        right_goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      message = "carry_action: timed out waiting for goal acceptance";
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return err::EXECUTE_FAILED;
    }

    auto left_handle = left_goal_future.get();
    auto right_handle = right_goal_future.get();
    if (!left_handle || !right_handle) {
      message = "carry_action: controller rejected carry goal";
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return err::EXECUTE_FAILED;
    }

    auto left_result_future = left_arm_traj_client_->async_get_result(left_handle);
    auto right_result_future = right_arm_traj_client_->async_get_result(right_handle);
    const auto exec_timeout =
      std::chrono::duration<double>(std::max(30.0, sync_duration_s + 5.0));
    if (left_result_future.wait_for(exec_timeout) != std::future_status::ready ||
        right_result_future.wait_for(exec_timeout) != std::future_status::ready) {
      message = "carry_action: timed out waiting for execution result";
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return err::EXECUTE_FAILED;
    }

    const auto left_wrapped = left_result_future.get();
    const auto right_wrapped = right_result_future.get();
    const bool left_ok =
      left_wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      left_wrapped.result &&
      left_wrapped.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
    const bool right_ok =
      right_wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      right_wrapped.result &&
      right_wrapped.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL;
    if (!left_ok || !right_ok) {
      message = "carry_action: controller execution failed";
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
      return err::EXECUTE_FAILED;
    }

    left_arm_->setStartStateToCurrentState();
    right_arm_->setStartStateToCurrentState();
    return err::OK;
  }

  bool carryGripperTarget(const std::string & gripper, double & target_m) const
  {
    if (gripper == "close") {
      target_m = gripper_closed_pos_;
      return true;
    }
    if (gripper == "open") {
      target_m = gripper_open_pos_;
      return true;
    }
    if (gripper == "half_close") {
      target_m = gripper_half_pos_;
      return true;
    }
    return false;
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
  // Used by doPick to skip open-gripper when fingers are already spread before
  // the approach move.
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
             double target_radius,
             double gripper_force,
             double gripper_speed,
             const std::shared_ptr<PickPlaceGoalHandle> & gh)
  {
    auto a = armGroup(arm); auto g = gripGroup(arm);
    if (!a || !g) return err::BAD_REQUEST;

    // Approach uses the user's speed_scale capped at transport_speed_scale_ so
    // that a very slow speed_scale (e.g. 0.01) is visible, while a very high
    // one does not exceed the configured transport limit.
    const double approach_speed = std::min(speed, transport_speed_scale_);
    const double effective_approach =
      effectiveApproachOffset(approach, target_radius);
    RCLCPP_INFO(get_logger(),
                "pick: speed_scale=%.3f  approach_speed=%.3f  transport_limit=%.3f "
                "target_radius=%.3f gripper_force=%.1f",
                speed, approach_speed, transport_speed_scale_,
                target_radius, gripper_force);

    // Open gripper before moving toward the grasp target so approach/descend
    // never push the object with closed fingers.
    if (!isGripperOpen(g)) {
      publishFb(gh, "grasping", "pick.open_gripper", 0.22,
                "opening gripper before approach");
      const int og = setGripper(g, arm, gripper_open_pos_, speed,
                                false, 0.0, gripper_speed);
      if (og) {
        RCLCPP_WARN(get_logger(),
                    "pick.open_gripper failed (rc=%d), continuing anyway", og);
      }
    } else {
      RCLCPP_INFO(get_logger(),
                  "pick.open_gripper: gripper already open, skipping");
    }

    publishFb(gh, "grasping", "pick.approach", 0.30, "moving above object");
    int rc = jointMoveToAvoid(a, offsetZ(grasp, effective_approach),
                              approach_speed, "pick.approach",
                              grasp.position, target_radius);
    if (rc) return rc;

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
    rc = setGripper(g, arm, gripper_closed_pos_, speed, true,
                    gripper_force, gripper_speed);
    if (rc) return rc;

    if (!isGripped(g)) {
      if (debug_assume_grasp_success_) {
        RCLCPP_WARN(get_logger(),
                    "pick.close_gripper: gripper closed empty; "
                    "debug_assume_grasp_success=true, continuing to place");
        publishFb(gh, "grasping", "pick.fake_grasp", 0.60,
                  "RViz debug: assuming object is held");
      } else {
        // Gripper is opened before approach; if retry still triggers the object
        // is likely outside grasp range — lift/open/descend is the recovery.
        RCLCPP_WARN(get_logger(),
                    "pick: gripper closed empty after first attempt, retrying "
                    "lift→open→descend×%d", grasp_retry_count_);
        for (int i = 0; i < grasp_retry_count_ && !isGripped(g); ++i) {
          rc = linearMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retry_lift");
          if (rc) jointMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retry_lift.jnt");
          setGripper(g, arm, gripper_open_pos_, speed,
                     false, 0.0, gripper_speed);
          rc = linearMoveTo(a, grasp, speed, "pick.retry_descend");
          if (rc) jointMoveTo(a, grasp, speed, "pick.retry_descend.jnt");
          setGripper(g, arm, gripper_closed_pos_, speed, true,
                     gripper_force, gripper_speed);
        }
        if (!isGripped(g)) return err::GRIP_NOT_HELD;
      }
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
              double place_obstacle_radius,
              const std::shared_ptr<PickPlaceGoalHandle> & gh)
  {
    auto a = armGroup(arm); auto g = gripGroup(arm);
    if (!a || !g) return err::BAD_REQUEST;

    const double approach_speed = std::min(speed, transport_speed_scale_);
    const double effective_retreat =
      effectiveApproachOffset(retreat, place_obstacle_radius);
    RCLCPP_INFO(get_logger(),
                "place: speed_scale=%.3f  approach_speed=%.3f  transport_limit=%.3f "
                "place_obstacle_radius=%.3f effective_retreat=%.3f",
                speed, approach_speed, transport_speed_scale_,
                place_obstacle_radius, effective_retreat);

    // transport / place.approach: no obstacle sphere (object is held in transit).
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
      if (!rc) {
        last_failure_phase_.clear();
        last_failure_reason_.clear();
      }
    }
    if (rc) return rc;

    publishFb(gh, "placing", "place.open_gripper", 0.92, "opening gripper");
    rc = setGripper(g, arm, gripper_open_pos_, speed);
    if (rc) return rc;

    // place.retreat: avoid the placed-object sphere (center = place pose).
    publishFb(gh, "placing", "place.retreat", 0.97, "lifting away from place");
    const auto retreat_pose = offsetZ(place, effective_retreat);
    if (place_obstacle_radius > 0.0) {
      rc = jointMoveToAvoid(a, retreat_pose, speed, "place.retreat",
                            place.position, place_obstacle_radius);
    } else {
      rc = linearMoveTo(a, retreat_pose, speed, "place.retreat");
      if (rc) {
        RCLCPP_WARN(get_logger(),
                    "place.retreat cartesian failed, falling back to joint-space");
        rc = jointMoveTo(a, retreat_pose, speed, "place.retreat.jnt");
        if (!rc) {
          last_failure_phase_.clear();
          last_failure_reason_.clear();
        }
      }
    }
    return rc;
  }

  int doPostPlaceReturn(const std::string & arm,
                        const std::vector<double> & start_arm_joints,
                        double start_gripper_target,
                        double speed,
                        const std::shared_ptr<PickPlaceGoalHandle> & gh)
  {
    auto a = armGroup(arm); auto g = gripGroup(arm);
    if (!a || !g) return err::BAD_REQUEST;

    const double return_speed = std::min(speed, transport_speed_scale_);
    int rc = err::OK;

    if (!start_arm_joints.empty()) {
      publishFb(gh, "returning", "return.start", 0.985,
                "returning to start pose");
      rc = moveArmToJoints(a, start_arm_joints, return_speed, "return.start");
      if (rc == err::OK) {
        last_failure_phase_.clear();
        last_failure_reason_.clear();
      } else if (rc == err::STOPPED_BY_USER) {
        return rc;
      } else {
        RCLCPP_WARN(get_logger(),
                    "return.start failed (rc=%d), falling back to home", rc);
      }
    }

    if (start_arm_joints.empty() || rc != err::OK) {
      publishFb(gh, "returning", "return.home", 0.985,
                "returning to home pose");
      rc = moveArmHome(a, return_speed, "return.home");
      if (rc) return rc;
      last_failure_phase_.clear();
      last_failure_reason_.clear();
    }

    publishFb(gh, "returning", "return.restore_gripper", 0.995,
              "restoring gripper state");
    rc = setGripper(g, arm, start_gripper_target, speed);
    if (rc) {
      last_failure_phase_ = "return.restore_gripper";
      last_failure_reason_ = "failed to restore initial gripper state";
      return rc;
    }

    return err::OK;
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
    const double gripper_force = normalizedGripperForce(goal->gripper_force);
    const double gripper_speed = normalizedGripperSpeed(goal->gripper_speed);
    const double target_radius =
      (std::isfinite(goal->target_radius) && goal->target_radius > 0.0)
        ? goal->target_radius
        : 0.0;

    auto start_arm = armGroup(goal->arm);
    auto start_grip = gripGroup(goal->arm);
    std::vector<double> start_arm_joints;
    double start_gripper_target = gripper_closed_pos_;
    if (start_arm) {
      start_arm_joints = start_arm->getCurrentJointValues();
    }
    if (start_grip) {
      start_gripper_target = currentGripperTargetOrClosed(start_grip);
    }
    RCLCPP_INFO(get_logger(),
                "execute: captured return target joints=%zu gripper=%.4f",
                start_arm_joints.size(), start_gripper_target);

    geometry_msgs::msg::Pose grasp = goal->grasp_pose;
    geometry_msgs::msg::Pose place = goal->place_pose;

    // Normalize quaternions early; an un-normalized orientation fed to KDL IK
    // produces a corrupted rotation target and causes unpredictable arm motion.
    normalizeQuat(grasp, "grasp_pose", get_logger());
    normalizeQuat(place, "place_pose", get_logger());

    RCLCPP_INFO(get_logger(),
                "execute: grasp pos=[%.3f,%.3f,%.3f] ori=[%.4f,%.4f,%.4f,%.4f]  "
                "place pos=[%.3f,%.3f,%.3f] ori=[%.4f,%.4f,%.4f,%.4f]  "
                "speed=%.3f approach=%.3f retreat=%.3f target_radius=%.3f "
                "gripper_force=%.1f gripper_speed=%.2f",
                grasp.position.x, grasp.position.y, grasp.position.z,
                grasp.orientation.x, grasp.orientation.y,
                grasp.orientation.z, grasp.orientation.w,
                place.position.x, place.position.y, place.position.z,
                place.orientation.x, place.orientation.y,
                place.orientation.z, place.orientation.w,
                speed, approach, retreat, target_radius,
                gripper_force, gripper_speed);

    publishFb(gh, "perceiving", "pick.detect", 0.05, "resolving grasp pose");
    if (goal->pose_source == "camera") {
      int rc = callPerception("grasp", goal->target_name, goal->target_index, grasp);
      if (rc) return finish(gh, result, rc, "grasp perception failed");
      result->perceived_grasp_pose = grasp;
    } else if (!inWorkspace(grasp)) {
      return finish(gh, result, err::BAD_REQUEST, "grasp_pose outside workspace");
    }

    int rc = doPick(goal->arm, grasp, approach, retreat, speed, target_radius,
                    gripper_force, gripper_speed, gh);
    if (rc) return finish(gh, result, rc, "pick failed");

    publishFb(gh, "transporting", "transport", 0.72, "moving to place region");
    if (goal->pose_source == "camera") {
      int prc = callPerception("place", goal->target_name, goal->target_index, place);
      if (prc) return finish(gh, result, prc, "place perception failed");
      result->perceived_place_pose = place;
    } else if (!inWorkspace(place)) {
      return finish(gh, result, err::BAD_REQUEST, "place_pose outside workspace");
    }

    // transport + place.approach/descend: no sphere avoidance.
    // place.retreat uses target_radius around place_pose (released object).
    rc = doPlace(goal->arm, place, approach, retreat, speed, target_radius, gh);
    if (rc) return finish(gh, result, rc, "place failed");

    rc = doPostPlaceReturn(goal->arm, start_arm_joints, start_gripper_target, speed, gh);
    if (rc) return finish(gh, result, rc, "post-place return failed");

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

    using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

    auto plan_one = [&](MoveGroupInterfacePtr arm, const std::string & label,
                        Plan & plan) -> int {
      if (!arm) return err::BAD_REQUEST;
      arm->setMaxVelocityScalingFactor(speed);
      arm->setMaxAccelerationScalingFactor(speed);
      arm->setStartStateToCurrentState();
      arm->setNamedTarget("home");
      if (arm->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "goto_home(%s): home plan failed", label.c_str());
        return err::PLAN_FAILED;
      }
      return err::OK;
    };

    auto execute_one = [&](MoveGroupInterfacePtr arm, const std::string & label,
                           const Plan & plan) -> int {
      if (!arm) return err::BAD_REQUEST;
      if (arm->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "goto_home(%s): home execute failed", label.c_str());
        return err::EXECUTE_FAILED;
      }
      // Sync state after execution so the next operation starts from the
      // correct (home) configuration rather than any cached pre-motion state.
      arm->setStartStateToCurrentState();
      return err::OK;
    };

    int rc = err::OK;
    if (req->arm == "both") {
      rc = executeBothHomeWithControllers(speed);
    } else if (req->arm == "left") {
      Plan plan;
      rc = plan_one(left_arm_, "left", plan);
      if (rc == err::OK) rc = execute_one(left_arm_, "left", plan);
    } else if (req->arm == "right") {
      Plan plan;
      rc = plan_one(right_arm_, "right", plan);
      if (rc == err::OK) rc = execute_one(right_arm_, "right", plan);
    } else {
      rc = err::BAD_REQUEST;
    }

    // Clear the stop flag set by handleStop so that subsequent service calls
    // (e.g. gripper) are not immediately rejected with STOPPED_BY_USER.
    // The next pick_place action also resets this flag at the start of execute().
    if (rc == err::OK) stop_requested_.store(false);

    resp->result_code = rc;
    resp->success = (rc == err::OK);
    resp->message = (rc == err::OK) ? "home reached" : "go-home failed";
  }

  void handleCarry(const std::shared_ptr<openarm_skills::srv::CarryAction::Request> req,
                   std::shared_ptr<openarm_skills::srv::CarryAction::Response> resp)
  {
    if (stop_requested_.load() && !busy_.load()) {
      stop_requested_.store(false);
    }

    double gripper_target = 0.0;
    if (!carryGripperTarget(req->gripper, gripper_target)) {
      resp->success = false;
      resp->result_code = err::BAD_REQUEST;
      resp->message = "gripper must be close|open|half_close";
      resp->actual_width = 0.0;
      return;
    }

    const double speed = (std::isfinite(req->speed_scale) && req->speed_scale > 0.0)
      ? std::max(0.05, std::min(1.0, req->speed_scale))
      : transport_speed_scale_;

    double actual_width = 0.0;
    std::string message;
    int rc = executeBothCarryWithControllers(req->width, speed, actual_width, message);
    if (rc == err::OK) {
      rc = setBothGrippers(gripper_target, speed, false, 0.0, speed);
      if (rc != err::OK) {
        message = "carry_action: gripper command failed";
      }
    }

    if (rc == err::OK) {
      stop_requested_.store(false);
      char buf[160];
      snprintf(buf, sizeof(buf),
               "carry pose reached width=%.3fm gripper=%s",
               actual_width, req->gripper.c_str());
      message = buf;
    } else if (message.empty()) {
      message = "carry_action failed";
    }

    resp->result_code = rc;
    resp->success = (rc == err::OK);
    resp->message = message;
    resp->actual_width = actual_width;
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

    int rc = setGripper(g, req->arm, target, default_speed_scale_,
                        compliance_grasp, req->force, req->speed);
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
  bool   debug_assume_grasp_success_;
  double gripper_open_pos_, gripper_half_pos_, gripper_closed_pos_;
  double gripper_close_thresh_, gripper_max_force_, gripper_default_speed_;
  double eef_obstacle_margin_m_;
  double workspace_radius_, min_z_, max_z_;
  double perception_timeout_s_;
  double carry_joint4_rad_, carry_joint6_rad_, carry_joint7_rad_;
  double carry_default_width_m_;
  double carry_min_width_m_{0.0}, carry_max_width_m_{0.0};

  rclcpp::Node::SharedPtr mg_node_;
  MoveGroupInterfacePtr left_arm_, right_arm_, left_grip_, right_grip_;
  rclcpp::CallbackGroup::SharedPtr traj_callback_group_;

  rclcpp_action::Server<PickPlaceAction>::SharedPtr pick_place_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr left_arm_traj_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr right_arm_traj_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr left_gripper_cmd_client_;
  rclcpp_action::Client<GripperCommand>::SharedPtr right_gripper_cmd_client_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_gripper_aux_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_gripper_aux_pub_;
  rclcpp::Service<openarm_skills::srv::Stop>::SharedPtr stop_srv_;
  rclcpp::Service<openarm_skills::srv::GotoHome>::SharedPtr home_srv_;
  rclcpp::Service<openarm_skills::srv::CarryAction>::SharedPtr carry_srv_;
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
