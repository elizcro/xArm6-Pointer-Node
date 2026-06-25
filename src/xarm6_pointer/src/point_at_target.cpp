// point_at_target.cpp
//
// Aim the xArm6 end-effector (the tool's +Z axis) at an arbitrary target
// point in space, stopping a configurable "standoff" distance short of the
// point so the tool never touches it. Targets are received on the
// `target_point` topic (geometry_msgs/PointStamped) so the same running node
// can be commanded to point at many different points.
//
// Safety model:
//   * Every target is validated against a reachable-workspace envelope BEFORE
//     any motion is attempted (reach radius + minimum height above the base).
//   * MoveIt 2 plans the motion. Planning enforces joint limits and
//     self-collision checking, and (optionally) an added ground-plane
//     collision object. If MoveIt cannot find a valid, collision-free plan,
//     the node simply does NOT move the robot.
//   * Velocity and acceleration are scaled down (default 10%) for slow,
//     observable motion.
//   * plan() and execute() are separate: we only execute a plan that
//     succeeded.
//
// This node does NOT start move_group or RViz. Launch the xArm MoveIt stack
// first (fake for RViz testing, or realmove for the physical arm); this node
// connects to that running move_group. See the README.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>


#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("xarm6_pointer");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // automatically_declare_parameters_from_overrides(true) lets the large set
  // of MoveIt parameters injected by the launch file (robot_description,
  // robot_description_semantic, kinematics, joint_limits, ...) be accepted on
  // this node without us declaring each one. MoveGroupInterface reads them.
  auto node = std::make_shared<rclcpp::Node>(
    "xarm6_pointer",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // ---- Parameters (with safe defaults if not provided) --------------------
  auto get_str = [&](const std::string & n, const std::string & d) {
    if (!node->has_parameter(n)) node->declare_parameter(n, d);
    return node->get_parameter(n).as_string();
  };
  auto get_double = [&](const std::string & n, double d) {
    if (!node->has_parameter(n)) node->declare_parameter(n, d);
    return node->get_parameter(n).as_double();
  };
  auto get_bool = [&](const std::string & n, bool d) {
    if (!node->has_parameter(n)) node->declare_parameter(n, d);
    return node->get_parameter(n).as_bool();
  };

  const std::string planning_group = get_str("planning_group", "xarm6");
  const std::string planning_frame = get_str("planning_frame", "link_base");
  const std::string ee_link = get_str("ee_link", "tool_tip");
  const std::string target_topic = get_str("target_topic", "target_point");

  const double standoff = get_double("standoff_distance", 0.7);  // m, tool stops this far from target
  const double min_reach = get_double("min_reach", 0.10);         // m, min EE distance from base origin
  const double max_reach = get_double("max_reach", 0.65);         // m, max EE distance (xArm6 reach ~0.70)
  const double vel_scale = get_double("vel_scale", 0.9);          // 0..1
  const double acc_scale = get_double("acc_scale", 0.25);          // 0..1
  const double planning_time = get_double("planning_time", 2.0); // s per roll sample
  const int planning_attempts = static_cast<int>(get_double("planning_attempts", 10.0));
  const double goal_pos_tol = get_double("goal_pos_tol", 0.01);       // m
  const double goal_orient_tol = get_double("goal_orient_tol", 0.01);  // rad about EACH axis, keep tight 
  const bool position_only = get_bool("position_only", false);        // diagnostic: ignore orientation
  const int roll_samples = static_cast<int>(get_double("roll_samples", 12.0));  // # of roll angles to try about the pointing axis
 
  //pedestal the xarm is sitting on
  const double pedestal_size_x = get_double("pedestal_size_x", 0.762);
  const double pedestal_size_y = get_double("pedestal_size_y", 1.524);
  const double pedestal_size_z = get_double("pedestal_size_z", 0.8382);
  const double pedestal_offset_x = get_double("pedestal_offset_x", 0.0127); // m, pedestal center offset from arm base in X (~ centered)
  const double pedestal_offset_y = get_double("pedestal_offset_y", -0.6858); // m, pedestal center offset in Y 5ft in side
  //wall the xarm is next to 
  const double wall_size_x = get_double("wall_size_x", 0.1);
  const double wall_size_y = get_double("wall_size_y", 3.048);
  const double wall_size_z = get_double("wall_size_z", 0.8382);
  const double wall_offset_x = get_double("wall_offset_x", 0.0127);
  const double wall_offset_y = get_double("wall_offset_y", 0.0);

  // ---- Spin the node in the background so MoveGroupInterface works ---------
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  RCLCPP_INFO(LOGGER, "Connecting to move_group for planning group '%s' ...",
              planning_group.c_str());

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  moveit::planning_interface::PlanningSceneInterface psi;

  move_group.setPoseReferenceFrame(planning_frame); // link_base
  move_group.setEndEffectorLink(ee_link); 	    // ee_link defaults to "tool_tip"
  move_group.setMaxVelocityScalingFactor(vel_scale);
  move_group.setMaxAccelerationScalingFactor(acc_scale);
  move_group.setPlanningTime(planning_time);
  move_group.setNumPlanningAttempts(planning_attempts);
  move_group.setGoalPositionTolerance(goal_pos_tol);
  move_group.setGoalOrientationTolerance(goal_orient_tol);

  RCLCPP_INFO(LOGGER, "Planning frame : %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(LOGGER, "End effector   : %s", move_group.getEndEffectorLink().c_str());
  RCLCPP_INFO(LOGGER, "Standoff       : %.3f m | reach [%.2f, %.2f] m",
              standoff, min_reach, max_reach);
  RCLCPP_INFO(LOGGER, "Pedestal       : %.3f x %.3f x %.3f m, centered at (%.3f, %.3f) (top at z=0)",
              pedestal_size_x, pedestal_size_y, pedestal_size_z,
              pedestal_offset_x, pedestal_offset_y);
  RCLCPP_INFO(LOGGER, "Wall       : %.3f x %.3f x %.3f m, centered at (%.3f, %.3f) (top at z=0)",
              wall_size_x, wall_size_y, wall_size_z,
              wall_offset_x, wall_offset_y);
  RCLCPP_INFO(LOGGER, "Vel/Acc scaling: %.2f / %.2f", vel_scale, acc_scale);

  // Pedestal mounting structure(top surface at base z=0) ---
  {
    moveit_msgs::msg::CollisionObject pedestal;
    pedestal.header.frame_id = planning_frame;
    pedestal.id = "pedestal";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {pedestal_size_x, pedestal_size_y, pedestal_size_z};
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = pedestal_offset_x;
    box_pose.position.y = pedestal_offset_y;
    //top face at z = -0.001 (1 mm safety  gap below the base mounting plate)
    box_pose.position.z = -pedestal_size_z / 2.0 - 0.001;
    pedestal.primitives.push_back(box);
    pedestal.primitive_poses.push_back(box_pose);
    pedestal.operation = pedestal.ADD;
    psi.applyCollisionObject(pedestal);
    RCLCPP_INFO(LOGGER, "Added pedestal structure collision object (%.3f x %.3f x %.3f m).",
                pedestal_size_x, pedestal_size_y, pedestal_size_z);
  }
  
    // Wall next to the table (top surface at base z=0) ---
  {
    moveit_msgs::msg::CollisionObject wall;
    wall.header.frame_id = planning_frame;
    wall.id = "wall";
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {wall_size_x, wall_size_y, wall_size_z};
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = wall_offset_x;
    box_pose.position.y = wall_offset_y;
    //top face at z = -0.001 (1 mm safety  gap below the base mounting plate)
    box_pose.position.z = wall_size_z / 2.0 - pedestal_size_z - 0.001;
    wall.primitives.push_back(box);
    wall.primitive_poses.push_back(box_pose);
    wall.operation = wall.ADD;
    psi.applyCollisionObject(wall);
    RCLCPP_INFO(LOGGER, "Added wall collision object (%.3f x %.3f x %.3f m).",
                wall_size_x, wall_size_y, wall_size_z);
  }

  // subscribe to /tf and /tf_static (only used if a target arrives in a non-planning frame) ---------
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  [[maybe_unused]] auto tf_listener =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // ---- Shared state between subscription callback and main loop -----------
  std::mutex mtx;
  std::optional<geometry_msgs::msg::PointStamped> pending;
  bool home_requested = false; // <-- set by the /pointer/home callback

  [[maybe_unused]] auto sub = node->create_subscription<geometry_msgs::msg::PointStamped>(
    target_topic, 10,
    [&mtx, &pending](geometry_msgs::msg::PointStamped::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(mtx);
      //convert incoming coordinates from cm to m
      msg->point.x /= 100.0;
      msg->point.y /= 100.0;
      msg->point.z /= 100.0;
      pending = *msg;  // keep only the latest; ignore backlog while moving
    });
    
  [[maybe_unused]] auto home_sub = node->create_subscription<std_msgs::msg::Empty>(
    "pointer/home", 10,
    [&mtx, &home_requested](std_msgs::msg::Empty::SharedPtr) {
      std::lock_guard<std::mutex> lk(mtx);
      home_requested = true;
    });

  auto result_pub = node->create_publisher<std_msgs::msg::Bool>("/pointer/result", 10);

  RCLCPP_INFO(LOGGER, "Ready. Publish a target on '%s' (PointStamped).",
              target_topic.c_str());

  // Core: turn a target point into a "pointing" pose and execute -------
  auto publish_result = [&](bool success) {
    std_msgs::msg::Bool msg;
    msg.data = success;
    result_pub->publish(msg);
  };

  auto process = [&](const geometry_msgs::msg::PointStamped & pin) {
    const tf2::Vector3 target(pin.point.x, pin.point.y, pin.point.z);
    // Targets published from command line are always in the link_base (planning) frame

    // Target converted to a vector
    const double target_r = target.length(); //distance from base origin to target

    RCLCPP_INFO(LOGGER, "Target in %s: (%.3f, %.3f, %.3f), |r|=%.3f m",
                planning_frame.c_str(), target.x(), target.y(), target.z(), target_r);

    // Check if it's a valid, reachable targets
    if (target_r < 1e-3) {
      RCLCPP_WARN(LOGGER, "Target is at the base origin; cannot define a pointing direction. Skipping.");
      publish_result(false);
      return;
    }

    // Pointing direction: from the base origin toward the target.
    tf2::Vector3 n_hat = target / target_r;

    // The end-effector sits 'standoff' meters short of the target, on the
    // base->target ray, so the tool points outward at the target but does not
    // reach it.
    const tf2::Vector3 ee_pos = target - n_hat * standoff;
    const double ee_r = ee_pos.length();

    if (ee_r < min_reach || ee_r > max_reach) {
      RCLCPP_WARN(LOGGER,
                  "Standoff pose at |r|=%.3f m is outside the safe reach envelope [%.2f, %.2f] m. Skipping.",
                  ee_r, min_reach, max_reach);
      publish_result(false);
      return;
    }

    // Build an orientation whose +Z axis aligns with n_hat (look-at orientation)
    //pick a reference direction
    tf2::Vector3 up(0.0, 0.0, 1.0);
    if (std::fabs(n_hat.dot(up)) > 0.95) {
      up = tf2::Vector3(1.0, 0.0, 0.0);  // avoid degeneracy when pointing near-vertical
    }
    // build an orthonormal basis (Gram-Schmidt orthogonalization)
    tf2::Vector3 x_axis = up.cross(n_hat); // computes a vector perpendicular to both units
    if (x_axis.length() < 1e-6) {
      up = tf2::Vector3(1.0, 0.0, 0.0);
      x_axis = up.cross(n_hat);
    }
    x_axis.normalize(); /// rescales it to length 1
    tf2::Vector3 y_axis = n_hat.cross(x_axis); // gives vector perpendicular to both n_hat and x_axis
    y_axis.normalize();
    // x_axis, y_axis, and n_hat form orthonormal basis

    // Rotation matrix: columns are the tool axes expressed in the base
    // This is the "base" orientation (roll = 0 about the pointing axis)
    tf2::Matrix3x3 rot(
      x_axis.x(), y_axis.x(), n_hat.x(),
      x_axis.y(), y_axis.y(), n_hat.y(),
      x_axis.z(), y_axis.z(), n_hat.z());
    
    //convert rotation matrix to a quaternion
    tf2::Quaternion base_q;
    rot.getRotation(base_q);
    base_q.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = ee_pos.x();
    pose.position.y = ee_pos.y();
    pose.position.z = ee_pos.z();

    RCLCPP_INFO(LOGGER, "Planning to standoff pos(%.3f, %.3f, %.3f), aiming at target.",
                ee_pos.x(), ee_pos.y(), ee_pos.z());

    // 4) Plan, then execute only if planning succeeded.
    //
    // For pointing, roll about the tool's Z (pointing) axis is a free DOF: any
    // roll still aims the tool at the target. Different rolls yield different
    // arm configurations, some of which collide (e.g. with the ground plane)
    // while others don't. So we try several rolls and use the first that
    // produces a valid, collision-free plan. This keeps the ground-plane
    // safety check active while still finding a workable pointing pose.
    move_group.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;

    if (position_only) {
      // Diagnostic mode: constrain only the EE position, leave orientation
      // free. If this succeeds but full-pose planning fails, the orientation
      // goal (KDL IK) is the bottleneck -> see TRAC-IK in the README.
      move_group.clearPoseTargets();
      move_group.setPositionTarget(ee_pos.x(), ee_pos.y(), ee_pos.z(), ee_link);
      RCLCPP_WARN(LOGGER, "position_only mode: ignoring pointing orientation.");
      planned = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    } else {
      // roll sampling
      const int n = std::max(1, roll_samples);
      for (int i = 0; i < n && rclcpp::ok(); ++i) {
        // Spread samples over a full turn, starting at 0 (most "natural").
        const double roll = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(n);
        tf2::Quaternion q_roll;
        q_roll.setRPY(0.0, 0.0, roll);          // rotation about the tool's own Z
        tf2::Quaternion q = base_q * q_roll;    // apply roll in the tool frame
        q.normalize();

        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        move_group.clearPoseTargets();
        move_group.setPoseTarget(pose, ee_link);
        if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(LOGGER, "Found a valid plan at roll %.0f deg (sample %d/%d).",
                      roll * 180.0 / M_PI, i + 1, n);
          planned = true;
          break;
        }
        RCLCPP_INFO(LOGGER, "Roll %.0f deg: no valid plan, trying next ...",
                    roll * 180.0 / M_PI);
      }
    }

    if (!planned) {
      RCLCPP_WARN(LOGGER,
                  "MoveIt could not find a valid plan at any roll for this target "
                  "(unreachable, joint limits, or collision). Not moving.");
      move_group.clearPoseTargets();
      publish_result(false);
      return;
    }

    RCLCPP_INFO(LOGGER, "Plan found. Executing at %.0f%% speed ...", vel_scale * 100.0);
    const bool executed =
      (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    move_group.clearPoseTargets();

    if (executed) {
      RCLCPP_INFO(LOGGER, "Done. End-effector is pointing at the target.");
      rclcpp::sleep_for(std::chrono::milliseconds(1500));
    } else {
      RCLCPP_WARN(LOGGER, "Execution did not report success. Check the robot/controllers.");
    }
    publish_result(executed);
  };
  
  auto go_home = [&]() {
    RCLCPP_INFO(LOGGER, "Returning to home position ...");
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();

    // Home is an end-effector POSITION goal (orientation left free for IK).
    // Your (0, 40, 30) cm is (0.00, 0.40, 0.30) m in link_base.
    const double deg = M_PI / 180.0;
    move_group.setJointValueTarget(std::vector<double>{
      87.8  * deg,   // J1
      -1.4  * deg,   // J2
      -2.5  * deg,   // J3
      179.5 * deg,   // J4
      130.3 * deg,   // J5
      -12.2 * deg    // J6
    });

    moveit::planning_interface::MoveGroupInterface::Plan plan;   // was  :Plan
    if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(LOGGER, "Could not plan a path to the home position.");  // was RCLCPP(
      return;
    }
    if (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) {  // was moveiit
      RCLCPP_INFO(LOGGER, "Home position reached.");
    } else {
      RCLCPP_WARN(LOGGER, "Home move did not report success.");
    }
    move_group.clearPoseTargets();
  };

  // ---- Main loop: process the most recent target, one at a time -----------
  rclcpp::Rate rate(10.0);
  while (rclcpp::ok()) {
    std::optional<geometry_msgs::msg::PointStamped> t;
    bool do_home = false;
    {
      std::lock_guard<std::mutex> lk(mtx);
      if (pending) {
        t = pending;
        pending.reset();
      } else if (home_requested) {
        home_requested = false;
        do_home = true;
      }
    }
    if (t) {
      process(*t);
    } else if (do_home) {
      go_home();
    }
    rate.sleep();
  }

  executor.cancel();
  if (spin_thread.joinable()) spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
