#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "avps_controller/pure_pursuit.hpp"

namespace avps
{

class ControllerNode : public rclcpp::Node
{
public:
  explicit ControllerNode(const rclcpp::NodeOptions & opts = {})
  : Node("avps_controller_node", opts)
  {
    declare_parameter("controller_frequency", 10.0);
    declare_parameter("lookahead_dist",       0.6);
    declare_parameter("linear_velocity",      0.3);
    declare_parameter("max_angular_vel",      1.5);
    declare_parameter("goal_tolerance",       0.25);

    PurePursuit::Params pp_params;
    pp_params.lookahead_dist  = get_parameter("lookahead_dist").as_double();
    pp_params.linear_velocity = get_parameter("linear_velocity").as_double();
    pp_params.max_angular_vel = get_parameter("max_angular_vel").as_double();
    pp_params.goal_tolerance  = get_parameter("goal_tolerance").as_double();
    controller_ = std::make_unique<PurePursuit>(pp_params);

    double freq = get_parameter("controller_frequency").as_double();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10));

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/plan", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Path::SharedPtr msg) { on_path(msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_odom(msg); });

    auto period = std::chrono::duration<double>(1.0 / freq);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { control_loop(); });

    RCLCPP_INFO(get_logger(),
      "Pure Pursuit Controller ready. Frequency: %.1f Hz, "
      "Lookahead: %.2f m, Speed: %.2f m/s",
      freq, pp_params.lookahead_dist, pp_params.linear_velocity);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr    path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<PurePursuit> controller_;

  Pose2D current_pose_;
  bool   have_pose_{false};
  bool   have_path_{false};

  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty path. Stopping.");
      controller_->clear_path();
      have_path_ = false;
      return;
    }

    std::vector<Pose2D> path;
    path.reserve(msg->poses.size());

    for (const auto & ps : msg->poses) {
      Pose2D p;
      p.x = ps.pose.position.x;
      p.y = ps.pose.position.y;

      tf2::Quaternion q;
      tf2::fromMsg(ps.pose.orientation, q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      p.yaw = yaw;

      path.push_back(p);
    }

    controller_->set_path(path);
    have_path_ = true;
    RCLCPP_INFO(get_logger(), "New path received: %zu waypoints.", path.size());
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_pose_.yaw = yaw;

    have_pose_ = true;
  }

  void control_loop()
  {
    if (!have_pose_ || !have_path_) {
      return;
    }

    auto cmd_result = controller_->compute(current_pose_);

    geometry_msgs::msg::Twist cmd;

    if (cmd_result.goal_reached) {
      RCLCPP_INFO_ONCE(get_logger(), "Goal reached! Stopping.");
      have_path_ = false;
      controller_->clear_path();
    } else {
      cmd.linear.x  = cmd_result.linear;
      cmd.angular.z = cmd_result.angular;
    }

    cmd_pub_->publish(cmd);
  }
};

}  // namespace avps

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<avps::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}