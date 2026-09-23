#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace
{
constexpr char kPlanningGroup[] = "ur_manipulator";
using Stroke = std::vector<geometry_msgs::msg::Pose>;

geometry_msgs::msg::Pose offset_pose(
  const geometry_msgs::msg::Pose & pose, double x_offset, double y_offset, double z_offset)
{
  auto result = pose;
  result.position.x += x_offset;
  result.position.y += y_offset;
  result.position.z += z_offset;
  return result;
}

// Mặt phẳng vẽ là base_link Y-Z. Trục X là pháp tuyến và được dùng để nâng bút.
std::vector<Stroke> make_letter_n(
  const geometry_msgs::msg::Pose & center, double width, double height)
{
  const auto point = [&center](double y, double z) { return offset_pose(center, 0.0, y, z); };
  const double half_w = width / 2.0;
  const double half_h = height / 2.0;
  const auto bl = point(-half_w, -half_h);
  const auto tl = point(-half_w, half_h);
  const auto br = point(half_w, -half_h);
  const auto tr = point(half_w, half_h);

  return {{bl, tl, br, tr}};
}

std::vector<Stroke> make_letter_n_only(
  const geometry_msgs::msg::Pose & center, double width, double height)
{
  return make_letter_n(center, width, height);
}

void publish_stroke_markers(
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
  const std::vector<Stroke> & strokes, const std::string & frame_id, const rclcpp::Time & stamp)
{
  int id = 0;
  for (const auto & stroke : strokes) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = "letter_path_n";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.008;  // 8 mm: clearly visible in RViz.
    marker.color.r = 0.1F;
    marker.color.g = 1.0F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;
    for (const auto & pose : stroke) {
      geometry_msgs::msg::Point point;
      point.x = pose.position.x;
      point.y = pose.position.y;
      point.z = pose.position.z;
      marker.points.push_back(point);
    }
    publisher->publish(marker);
  }
}

bool move_to_pose(
  moveit::planning_interface::MoveGroupInterface & group,
  const geometry_msgs::msg::Pose & target, const std::string & description)
{
  group.setStartStateToCurrentState();
  group.clearPoseTargets();
  group.setPoseTarget(target);
  if (group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(group.getNode()->get_logger(), "Could not %s.", description.c_str());
    return false;
  }
  return true;
}
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "ur3_letter_drawer", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  const auto finish = [&executor, &spinner](int code) {
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return code;
    };

  const auto logger = node->get_logger();
  moveit::planning_interface::MoveGroupInterface group(node, kPlanningGroup);
  const auto marker_topic = node->get_parameter_or("marker_topic", std::string("/letter_path_n"));
  const auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    marker_topic, rclcpp::QoS(1).transient_local());
  const double height = node->get_parameter_or("letter_height", 0.20);
  const double width = node->get_parameter_or("letter_width", 0.16);
  const double pen_lift = node->get_parameter_or("pen_lift", 0.04);
  const double eef_step = node->get_parameter_or("eef_step", 0.005);
  const double min_fraction = node->get_parameter_or("min_fraction", 0.95);
  if (height <= 0.0 || width <= 0.0 || pen_lift <= 0.0 || eef_step <= 0.0 || min_fraction <= 0.0 ||
    min_fraction > 1.0)
  {
    RCLCPP_ERROR(logger, "Invalid parameters: dimensions must be positive and min_fraction must be in (0, 1].");
    return finish(1);
  }

  group.setPlanningTime(10.0);
  group.setMaxVelocityScalingFactor(0.20);
  group.setMaxAccelerationScalingFactor(0.20);
  group.setPoseReferenceFrame("base_link");
  RCLCPP_INFO(logger, "Waiting for robot joint state...");
  if (!group.startStateMonitor(10.0) || !group.getCurrentState(10.0)) {
    RCLCPP_ERROR(logger, "No current state received. Is /joint_states available?");
    return finish(1);
  }

  const std::map<std::string, double> ready = {
    {"shoulder_pan_joint", 0.0}, {"shoulder_lift_joint", -1.57}, {"elbow_joint", -1.57},
    {"wrist_1_joint", -1.57}, {"wrist_2_joint", 1.57}, {"wrist_3_joint", 0.0}};
  group.setStartStateToCurrentState();
  group.setJointValueTarget(ready);
  if (group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Could not reach the checked ready joint configuration.");
    return finish(1);
  }

  const auto strokes = make_letter_n_only(group.getCurrentPose().pose, width, height);
  publish_stroke_markers(marker_pub, strokes, group.getPlanningFrame(), node->now());
  RCLCPP_INFO(logger, "Vẽ chữ N trong mặt phẳng base_link Y-Z (%.2f m x %.2f m).", width, height);
  for (std::size_t index = 0; index < strokes.size(); ++index) {
    const auto & stroke = strokes[index];
    // Chuyển động trung gian được MoveIt lập kế hoạch; nâng trục X để tách từng đoạn vẽ.
    if (!move_to_pose(group, offset_pose(stroke.front(), pen_lift, 0.0, 0.0), "di chuyển lên trên đoạn vẽ tiếp theo") ||
      !move_to_pose(group, stroke.front(), "hạ dụng cụ xuống mặt phẳng vẽ"))
    {
      return finish(2);
    }

    group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;
    const double fraction = group.computeCartesianPath(stroke, eef_step, trajectory, true, &error_code);
    RCLCPP_INFO(logger, "Đoạn %zu/%zu Cartesian fraction: %.1f%%.",
      index + 1, strokes.size(), fraction * 100.0);
    if (fraction < min_fraction || error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(logger, "Đoạn %zu chưa hoàn tất hoặc bị giới hạn va chạm; không thực thi.", index + 1);
      return finish(3);
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = trajectory;
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger, "Thực thi đoạn %zu thất bại.", index + 1);
      return finish(4);
    }
  }
  RCLCPP_INFO(logger, "Hoàn tất vẽ chữ N.");
  return finish(0);
}
