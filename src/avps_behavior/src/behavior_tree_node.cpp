#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
// Jazzy: behaviortree_cpp/ instead of behaviortree_cpp_v3/
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

#include "avps_behavior/bt_nodes/navigate_to_zone.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("avps_behavior_node");

  BT::BehaviorTreeFactory factory;

  factory.registerBuilder<avps::bt::NavigateToZone>(
    "NavigateToZone",
    [node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<avps::bt::NavigateToZone>(name, config, node);
    });

  std::string bt_xml_path;
  node->declare_parameter("bt_xml_path", "");
  node->get_parameter("bt_xml_path", bt_xml_path);

  if (bt_xml_path.empty()) {
    RCLCPP_ERROR(node->get_logger(), "bt_xml_path parameter not set!");
    return 1;
  }

  auto tree = factory.createTreeFromFile(bt_xml_path);

  BT::StdCoutLogger logger(tree);

  RCLCPP_INFO(node->get_logger(), "Behavior Tree loaded. Starting execution.");

  rclcpp::Rate rate(10.0);
  // Jazzy: tickOnce() replaces tickRoot() from BT.CPP v3
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    status = tree.tickOnce();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(),
    "Behavior Tree finished with status: %s",
    status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");

  rclcpp::shutdown();
  return 0;
}