// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <list>
#include <memory>
#include <string>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

struct BridgeIgnToRos2Handles
{
  std::shared_ptr<ignition::transport::Node> ign_subscriber;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_publisher;
};

class PoseBridge : public rclcpp::Node
{
public:
  PoseBridge() : Node("ign_to_ros2_pose_bridge") {}

  void convert_ign_to_ros2(const ignition::msgs::Pose &ign_msg, nav_msgs::msg::Odometry &ros_msg)
  {
    ros_msg.header.stamp = this->get_clock()->now();
    ros_msg.header.frame_id = "world";
    ros_msg.pose.pose.orientation.w = ign_msg.orientation().w();
    ros_msg.pose.pose.orientation.x = ign_msg.orientation().x();
    ros_msg.pose.pose.orientation.y = ign_msg.orientation().y();
    ros_msg.pose.pose.orientation.z = ign_msg.orientation().z();
    ros_msg.pose.pose.position.x = ign_msg.position().x();
    ros_msg.pose.pose.position.y = ign_msg.position().y();
    ros_msg.pose.pose.position.z = ign_msg.position().z();
  }

  void ign_callback(const ignition::msgs::Pose_V &ign_msg, rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_pub)
  {
    auto size_ = ign_msg.pose_size();
    for (int i = 0; i < size_; i++)
    {
      std::string topic_name = ros_pub->get_topic_name();
      std::string vehicle_name = topic_name.substr(topic_name.rfind("/") + 1, topic_name.length() - topic_name.rfind("/"));
      if (std::string(ign_msg.pose(i).name()) == vehicle_name)
      {
        nav_msgs::msg::Odometry ros_msg;
        convert_ign_to_ros2(ign_msg.pose(i), ros_msg);
        ros_pub->publish(ros_msg);
      }
    }
  }

  void create_ign_subscriber(
    std::shared_ptr<ignition::transport::Node> node,
    const std::string &topic_name,
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_pub)
  {
    std::function<void(const ignition::msgs::Pose_V &, const ignition::transport::MessageInfo &)> subCb =
      [this, ros_pub](const ignition::msgs::Pose_V &_msg, const ignition::transport::MessageInfo &_info)
      {
        this->ign_callback(_msg, ros_pub);
      };

    node->Subscribe(topic_name, subCb);
  }

  BridgeIgnToRos2Handles create_bridge_from_ign_to_ros2(
    std::shared_ptr<ignition::transport::Node> ign_node,
    const std::string &ign_topic_name,
    const std::string &ros_topic_name)
  {
    auto ros_pub = this->create_publisher<nav_msgs::msg::Odometry>(ros_topic_name, 10);

    this->create_ign_subscriber(ign_node, ign_topic_name, ros_pub);

    BridgeIgnToRos2Handles handles;
    handles.ign_subscriber = ign_node;
    handles.ros_publisher = ros_pub;
    return handles;
  }
};

int main(int argc, char *argv[])
{
  if (argc < 2)
  {
    std::cerr << "Usage: parameter_bridge <ign_topic> <vehicle_name1> <vehicle_name2> ..." << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseBridge>();

  auto ign_node = std::make_shared<ignition::transport::Node>();
  std::list<BridgeIgnToRos2Handles> ign_to_ros2_handles;

  std::string ign_topic_name = argv[1];
  for (int i = 2; i < argc; ++i)
  {
    std::string vehicle_name = std::string(argv[i]);
    std::string ros_topic_name = std::string("/pose/groundtruth/") + vehicle_name;

    try
    {
      ign_to_ros2_handles.push_back(node->create_bridge_from_ign_to_ros2(
        ign_node, ign_topic_name, ros_topic_name));
    }
    catch (std::exception &e)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to create a bridge for topic [%s] with ROS 2 topic [%s] and Ignition Transport Topic [%s]", vehicle_name.c_str(), ros_topic_name.c_str(), ign_topic_name.c_str());
    }
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
