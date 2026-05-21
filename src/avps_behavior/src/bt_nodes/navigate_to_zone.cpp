#include "avps_behavior/bt_nodes/navigate_to_zone.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace avps::bt
{

NavigateToZone::NavigateToZone(
  const std::string & name,
  const BT::NodeConfiguration & config,
  rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config), node_(node)
{
  action_client_ = rclcpp_action::create_client<NavToPose>(
    node_, "navigate_to_pose");
}

BT::NodeStatus NavigateToZone::onStart()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
      "NavigateToPose action server not available!");
    return BT::NodeStatus::FAILURE;
  }

  double goal_x, goal_y, goal_yaw;
  if (!getInput("goal_x",   goal_x)  ||
      !getInput("goal_y",   goal_y)  ||
      !getInput("goal_yaw", goal_yaw))
  {
    RCLCPP_ERROR(node_->get_logger(), "Missing goal input ports.");
    return BT::NodeStatus::FAILURE;
  }

  NavToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp    = node_->get_clock()->now();
  goal_msg.pose.pose.position.x = goal_x;
  goal_msg.pose.pose.position.y = goal_y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, goal_yaw);
  goal_msg.pose.pose.orientation = tf2::toMsg(q);

  auto send_opts = rclcpp_action::Client<NavToPose>::SendGoalOptions();
  send_opts.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      goal_done_      = true;
      goal_succeeded_ = (result.code ==
                         rclcpp_action::ResultCode::SUCCEEDED);
    };

  goal_future_ = action_client_->async_send_goal(goal_msg, send_opts);
  goal_sent_   = true;
  goal_done_   = false;

  RCLCPP_INFO(node_->get_logger(),
    "Navigating to zone: (%.2f, %.2f, %.2f)", goal_x, goal_y, goal_yaw);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToZone::onRunning()
{
  if (!goal_done_) {
    return BT::NodeStatus::RUNNING;
  }
  return goal_succeeded_ ?
    BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void NavigateToZone::onHalted()
{
  if (goal_handle_) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  RCLCPP_INFO(node_->get_logger(), "NavigateToZone halted.");
}

}  // namespace avps::bt