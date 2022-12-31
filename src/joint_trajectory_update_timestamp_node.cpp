// Copyright (c) 2022 Naoki Takahashi
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace unitree_go1_simulator
{
class JointTrajectoryUpdateTimestampNode : public rclcpp::Node
{
public:
  explicit JointTrajectoryUpdateTimestampNode(const rclcpp::NodeOptions &);
  ~JointTrajectoryUpdateTimestampNode();

private:
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajectory_subscription;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_joint_trajecotry_publisher;

  void jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory &);
};


JointTrajectoryUpdateTimestampNode::JointTrajectoryUpdateTimestampNode(const rclcpp::NodeOptions &options)
  : rclcpp::Node("joint_trajectory_update_timestamp_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Start joint_trajectory_update_timestamp_node");

  m_joint_trajectory_subscription = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    "joint_group_position_controller/command",
    10,
    std::bind(
      &JointTrajectoryUpdateTimestampNode::jointTrajectoryCallback,
      this,
      std::placeholders::_1
    )
  );

  m_joint_trajecotry_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory_controller/joint_trajectory", 10
  );
}

JointTrajectoryUpdateTimestampNode::~JointTrajectoryUpdateTimestampNode()
{
  RCLCPP_INFO(this->get_logger(), "Finish joint_trajectory_update_timestamp_node");
}

void JointTrajectoryUpdateTimestampNode::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory &msg)
{
  if(m_joint_trajecotry_publisher)
  {
    trajectory_msgs::msg::JointTrajectory updated_timestamp_msg{msg};

    updated_timestamp_msg.header.stamp = this->get_clock()->now();

    m_joint_trajecotry_publisher->publish(updated_timestamp_msg);
  }
}
}  // namespace unitree_go1_simulator

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_go1_simulator::JointTrajectoryUpdateTimestampNode)
