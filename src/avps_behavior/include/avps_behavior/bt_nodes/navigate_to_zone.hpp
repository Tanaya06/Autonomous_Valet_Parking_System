#pragma once

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace avps::bt
{

class NavigateToZone : public BT::StatefulActionNode
{
public:
  using NavToPose   = nav2_msgs::action::NavigateToPose;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<NavToPose>;

  NavigateToZone(const std::string & name,
                 const BT::NodeConfiguration & config,
                 rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("goal_x",   "Target X coordinate"),
      BT::InputPort<double>("goal_y",   "Target Y coordinate"),
      BT::InputPort<double>("goal_yaw", "Target heading (radians)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavToPose>::SharedPtr action_client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_sent_{false};
  bool goal_done_{false};
  bool goal_succeeded_{false};
};

}  // namespace avps::bt