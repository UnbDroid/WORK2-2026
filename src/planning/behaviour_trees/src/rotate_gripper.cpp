// include/planning_bt_nodes/rotate_gripper.hpp
#pragma once // evita que o arquivo .hpp seja incluído mais de uma vez durante a compilação.
#include <string>
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace planning_bt_nodes
{

class RotateGripper : public BT::StatefulActionNode // herança de ação do Behavior Tree que pode permanecer executando por algum tempo
{
public:
  RotateGripper(const std::string & name, const BT::NodeConfig & config) //construtor em que nome é o nome da instância na BT
  : BT::StatefulActionNode(name, config)
  {
    config.blackboard->get("node", node_); 
    pub_ = node_->create_publisher<std_msgs::msg::String>("/topico_garra", 10); //cria publisher para conversar com microros
    sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/topico_garra_status", 10,
      [this](std_msgs::msg::String::SharedPtr msg) { //define funcao de callback
        if (msg->data == expected_reply_) { done_ = true; } //verifica se a mensagem de feedback chegou certo
      }); //cria subscriber para receber feedback de completion do microros
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("target")}; //o valor de slot1 vai para o código através do port target
  }

  BT::NodeStatus onStart() override
  {
    std::string target;
    if (!getInput("target", target)) {
      RCLCPP_ERROR(node_->get_logger(), "[RotateGripper] missing input [target]");
      return BT::NodeStatus::FAILURE;
    }

    if (target == "slot1") expected_reply_ = "1";
    else if (target == "slot2") expected_reply_ = "2";
    else if (target == "slot3") expected_reply_ = "3";
    else if (target == "origin") expected_reply_ = "0";
    else {
      RCLCPP_ERROR(node_->get_logger(), "[RotateGripper] unknown target: %s", target.c_str());
      return BT::NodeStatus::FAILURE;
    }

    std_msgs::msg::String msg;
    msg.data = expected_reply_;
    pub_->publish(msg);

    done_ = false;
    start_time_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // spin_some so the subscription callback actually fires
    rclcpp::spin_some(node_);

    if (done_) {
      return BT::NodeStatus::SUCCESS;
    }

    // Path A fallback: hard timeout as a safety net (tune per motor)
    if ((node_->now() - start_time_).seconds() > 5.0) {
      RCLCPP_WARN(node_->get_logger(), "[RotateGripper] timed out waiting for feedback");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::string expected_reply_;
  bool done_{false};
  rclcpp::Time start_time_;
};

}  // namespace planning_bt_nodes