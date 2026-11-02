// pick_place.cpp
// Repeated pick-and-place of a box between two table locations with a Franka Panda,
// using MoveIt 2's MoveGroupInterface. Runs against moveit_resources_panda_moveit_config's
// demo.launch.py (mock ros2_control hardware, RViz).
//
// Each cycle: open gripper -> move above object -> straight-line descend -> attach + close ->
// straight-line lift -> move above target -> descend -> open + detach -> retreat.
// The next cycle moves the box back, so N cycles can run unattended.
// Prints per-cycle success, duration, and a summary (success rate, mean cycle time).

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using geometry_msgs::msg::Pose;

namespace
{
// Names from moveit_resources_panda_moveit_config/config/panda.srdf
const std::string kArmGroup = "panda_arm";
const std::string kHandGroup = "hand";
const std::string kEefLink = "panda_hand";
const std::string kFrame = "panda_link0";
const std::vector<std::string> kTouchLinks = {"panda_hand", "panda_leftfinger", "panda_rightfinger"};

const std::string kObjectId = "box";
constexpr double kBoxSize = 0.04;          // 4 cm cube; open gripper span is 2 x 0.035 = 7 cm
constexpr double kHandToFingertip = 0.105; // panda_hand origin -> centre of finger pads (approx.)
constexpr double kApproachHeight = 0.12;   // clearance above grasp pose for approach / retreat
constexpr double kCartesianStep = 0.005;   // 5 mm interpolation for straight-line moves

struct Point2D
{
  double x;
  double y;
};

moveit_msgs::msg::CollisionObject makeBox(const Point2D& at)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.header.frame_id = kFrame;
  obj.id = kObjectId;

  shape_msgs::msg::SolidPrimitive prim;
  prim.type = prim.BOX;
  prim.dimensions = {kBoxSize, kBoxSize, kBoxSize};

  Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = at.x;
  pose.position.y = at.y;
  pose.position.z = kBoxSize / 2.0;  // resting on the z = 0 plane

  obj.primitives.push_back(prim);
  obj.primitive_poses.push_back(pose);
  obj.operation = obj.ADD;
  return obj;
}

class PickPlace
{
public:
  explicit PickPlace(const rclcpp::Node::SharedPtr& node)
    : node_(node), log_(node->get_logger()), arm_(node, kArmGroup), hand_(node, kHandGroup)
  {
    arm_.setEndEffectorLink(kEefLink);
    arm_.setPoseReferenceFrame(kFrame);
    arm_.setPlanningTime(5.0);
    arm_.setNumPlanningAttempts(5);
    arm_.setMaxVelocityScalingFactor(0.5);
    arm_.setMaxAccelerationScalingFactor(0.5);
  }

  // Go to the SRDF "ready" pose and record the top-down gripper orientation used for all grasps.
  bool init()
  {
    if (!moveNamed(arm_, "ready") || !moveNamed(hand_, "open"))
      return false;
    grasp_orientation_ = arm_.getCurrentPose(kEefLink).pose.orientation;
    return true;
  }

  void spawnBox(const Point2D& at)
  {
    scene_.applyCollisionObject(makeBox(at));
  }

  // One pick-and-place cycle. Returns true only if every stage succeeded.
  bool cycle(const Point2D& from, const Point2D& to)
  {
    const double grasp_z = kBoxSize / 2.0 + kHandToFingertip;

    if (!stage("open gripper", moveNamed(hand_, "open"))) return false;
    if (!stage("move above object", movePose(poseAt(from, grasp_z + kApproachHeight)))) return false;
    if (!stage("descend to grasp", moveStraight(poseAt(from, grasp_z)))) return false;
    if (!stage("attach object", arm_.attachObject(kObjectId, kEefLink, kTouchLinks))) return false;
    if (!stage("close gripper", moveNamed(hand_, "close"))) return false;
    if (!stage("lift", moveStraight(poseAt(from, grasp_z + kApproachHeight)))) return false;
    if (!stage("move above target", movePose(poseAt(to, grasp_z + kApproachHeight)))) return false;
    if (!stage("descend to place", moveStraight(poseAt(to, grasp_z)))) return false;
    if (!stage("open gripper", moveNamed(hand_, "open"))) return false;
    if (!stage("detach object", arm_.detachObject(kObjectId))) return false;
    if (!stage("retreat", moveStraight(poseAt(to, grasp_z + kApproachHeight)))) return false;
    return true;
  }

  // After a failed cycle: release anything held, return home, put the box back at `at`.
  void recover(const Point2D& at)
  {
    RCLCPP_WARN(log_, "Recovering: releasing object, returning to 'ready', respawning box");
    arm_.detachObject(kObjectId);
    moveNamed(hand_, "open");
    moveNamed(arm_, "ready");
    scene_.removeCollisionObjects({kObjectId});
    spawnBox(at);
  }

  bool goHome()
  {
    return moveNamed(arm_, "ready");
  }

private:
  Pose poseAt(const Point2D& p, double z) const
  {
    Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = z;
    pose.orientation = grasp_orientation_;
    return pose;
  }

  bool stage(const char* name, bool ok)
  {
    if (ok)
      RCLCPP_INFO(log_, "  [ok]   %s", name);
    else
      RCLCPP_ERROR(log_, "  [FAIL] %s", name);
    return ok;
  }

  bool moveNamed(MoveGroupInterface& group, const std::string& target)
  {
    group.setNamedTarget(target);
    return static_cast<bool>(group.move());
  }

  // Free-space move (OMPL) to a pose target.
  bool movePose(const Pose& target)
  {
    arm_.setPoseTarget(target, kEefLink);
    MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(arm_.plan(plan)))
      return false;
    return static_cast<bool>(arm_.execute(plan));
  }

  // Straight-line end-effector move, used for approach / lift / retreat.
  bool moveStraight(const Pose& target)
  {
    std::vector<Pose> waypoints{target};
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = arm_.computeCartesianPath(waypoints, kCartesianStep, 0.0, trajectory);
    if (fraction < 0.99)
    {
      RCLCPP_WARN(log_, "  Cartesian path only %.0f%% achievable", fraction * 100.0);
      return false;
    }
    return static_cast<bool>(arm_.execute(trajectory));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger log_;
  MoveGroupInterface arm_;
  MoveGroupInterface hand_;
  PlanningSceneInterface scene_;
  geometry_msgs::msg::Quaternion grasp_orientation_;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "pick_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // MoveGroupInterface needs the node spinning to receive joint states.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int cycles = 3;
  node->get_parameter_or("cycles", cycles, 3);

  const Point2D station_a{0.5, -0.2};
  const Point2D station_b{0.5, 0.2};

  PickPlace app(node);
  int successes = 0;
  double total_time = 0.0;

  if (!app.init())
  {
    RCLCPP_FATAL(node->get_logger(), "Could not reach 'ready' pose. Is demo.launch.py running?");
  }
  else
  {
    app.spawnBox(station_a);
    for (int i = 0; i < cycles; ++i)
    {
      // Alternate direction so the box shuttles A -> B -> A ...
      const Point2D& from = (i % 2 == 0) ? station_a : station_b;
      const Point2D& to = (i % 2 == 0) ? station_b : station_a;

      RCLCPP_INFO(node->get_logger(), "Cycle %d/%d", i + 1, cycles);
      const auto start = std::chrono::steady_clock::now();
      const bool ok = app.cycle(from, to);
      const double secs =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

      if (ok)
      {
        ++successes;
        total_time += secs;
        RCLCPP_INFO(node->get_logger(), "Cycle %d succeeded in %.2f s", i + 1, secs);
      }
      else
      {
        RCLCPP_ERROR(node->get_logger(), "Cycle %d failed after %.2f s", i + 1, secs);
        app.recover(to);  // put the box where the next cycle expects it
      }
    }
    app.goHome();

    RCLCPP_INFO(node->get_logger(), "==== Summary ====");
    RCLCPP_INFO(node->get_logger(), "Success rate: %d/%d (%.0f%%)", successes, cycles,
                cycles > 0 ? 100.0 * successes / cycles : 0.0);
    if (successes > 0)
      RCLCPP_INFO(node->get_logger(), "Mean cycle time (successful): %.2f s", total_time / successes);
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
