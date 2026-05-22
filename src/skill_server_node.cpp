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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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
    cartesian_jump_thresh_  = get("cartesian_jump_threshold", 0.0);
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

    std::vector<geometry_msgs::msg::Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory traj;

    int attempts = plan_retry_count_ + 1;
    double fraction = 0.0;
    while (attempts-- > 0 && !stop_requested_.load()) {
      fraction = arm->computeCartesianPath(
        waypoints, cartesian_eef_step_, cartesian_jump_thresh_, traj);
      if (fraction >= cartesian_min_fraction_) break;
      RCLCPP_WARN(get_logger(),
                  "[%s] cartesian plan fraction=%.2f, retrying...", phase.c_str(), fraction);
    }
    if (fraction < cartesian_min_fraction_) {
      RCLCPP_ERROR(get_logger(), "[%s] cartesian plan FAILED (frac=%.2f)",
                   phase.c_str(), fraction);
      return err::PLAN_FAILED;
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
    arm->setStartStateToCurrentState();

    const std::string eef = arm->getEndEffectorLink();
    if (!arm->setPoseTarget(target, eef)) {
      RCLCPP_ERROR(get_logger(), "[%s] IK failed for link '%s'", phase.c_str(), eef.c_str());
      logTargetPose(get_logger(), phase, target);
      return err::PLAN_FAILED;
    }

    int attempts = plan_retry_count_ + 1;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    while (attempts-- > 0 && !stop_requested_.load()) {
      if (arm->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        if (arm->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          return err::OK;
        }
        RCLCPP_WARN(get_logger(), "[%s] execute failed, retrying", phase.c_str());
        continue;
      }
      RCLCPP_WARN(get_logger(), "[%s] plan failed, retrying", phase.c_str());
    }
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

    publishFb(gh, "grasping", "pick.approach", 0.30, "moving above object");
    int rc = jointMoveTo(a, offsetZ(grasp, approach),
                          transport_speed_scale_, "pick.approach");
    if (rc) return rc;

    publishFb(gh, "grasping", "pick.descend", 0.45, "descending to grasp");
    rc = linearMoveTo(a, grasp, speed, "pick.descend");
    if (rc) return rc;

    publishFb(gh, "grasping", "pick.close_gripper", 0.55, "grasping object");
    rc = setGripper(g, gripper_closed_pos_, speed, true);
    if (rc) return rc;

    if (!isGripped(g)) {
      RCLCPP_WARN(get_logger(), "pick: gripper closed empty, retrying once");
      // simple retry: lift, re-descend, grasp
      for (int i = 0; i < grasp_retry_count_ && !isGripped(g); ++i) {
        linearMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retry_lift");
        setGripper(g, gripper_open_pos_, speed);
        linearMoveTo(a, grasp, speed, "pick.retry_descend");
        setGripper(g, gripper_closed_pos_, speed, true);
      }
      if (!isGripped(g)) return err::GRIP_NOT_HELD;
    }

    publishFb(gh, "grasping", "pick.retreat", 0.65, "lifting object");
    rc = linearMoveTo(a, offsetZ(grasp, retreat), speed, "pick.retreat");
    return rc;
  }

  int doPlace(const std::string & arm,
              const geometry_msgs::msg::Pose & place,
              double approach, double retreat, double speed,
              const std::shared_ptr<PickPlaceGoalHandle> & gh)
  {
    auto a = armGroup(arm); auto g = gripGroup(arm);
    if (!a || !g) return err::BAD_REQUEST;

    publishFb(gh, "placing", "place.approach", 0.78, "moving above target");
    int rc = jointMoveTo(a, offsetZ(place, approach),
                          transport_speed_scale_, "place.approach");
    if (rc) return rc;

    publishFb(gh, "placing", "place.descend", 0.85, "descending to place");
    rc = linearMoveTo(a, place, speed, "place.descend");
    if (rc) return rc;

    publishFb(gh, "placing", "place.open_gripper", 0.92, "opening gripper");
    rc = setGripper(g, gripper_open_pos_, speed);
    if (rc) return rc;

    publishFb(gh, "placing", "place.retreat", 0.97, "lifting away");
    rc = linearMoveTo(a, offsetZ(place, retreat), speed, "place.retreat");
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
    const double approach = goal->approach_offset_m > 0 ? goal->approach_offset_m
                                                        : approach_offset_m_;
    const double retreat  = goal->retreat_offset_m  > 0 ? goal->retreat_offset_m
                                                        : retreat_offset_m_;
    const double speed    = goal->speed_scale > 0 ? goal->speed_scale
                                                  : default_speed_scale_;

    geometry_msgs::msg::Pose grasp = goal->grasp_pose;
    geometry_msgs::msg::Pose place = goal->place_pose;

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
    result->message = msg;
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
    auto run_one = [&](MoveGroupInterfacePtr arm) -> int {
      arm->setMaxVelocityScalingFactor(req->speed_scale > 0 ? req->speed_scale
                                                            : transport_speed_scale_);
      arm->setStartStateToCurrentState();
      arm->setNamedTarget("home");
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arm->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) return err::PLAN_FAILED;
      if (arm->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) return err::EXECUTE_FAILED;
      return err::OK;
    };

    int rc = err::OK;
    if (req->arm == "left" || req->arm == "both")  rc = run_one(left_arm_);
    if (rc == err::OK && (req->arm == "right" || req->arm == "both")) rc = run_one(right_arm_);

    resp->result_code = rc;
    resp->success = (rc == err::OK);
    resp->message = (rc == err::OK) ? "home reached" : "go-home failed";
  }

  void handleGripper(const std::shared_ptr<openarm_skills::srv::Gripper::Request> req,
                     std::shared_ptr<openarm_skills::srv::Gripper::Response> resp)
  {
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
