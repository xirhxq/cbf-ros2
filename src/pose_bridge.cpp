#include <iostream>
#include <list>
#include <memory>
#include <string>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/pose_v.pb.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

struct BridgeIgnToRos2Handles
{
  std::shared_ptr<ignition::transport::Node> ign_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_publisher;
};

class PoseBridge : public rclcpp::Node
{
public:
  PoseBridge() : Node("ign_to_ros2_pose_bridge") {}

  void convert_ign_to_ros2(const ignition::msgs::Pose &ign_msg, geometry_msgs::msg::PoseStamped &ros_msg)
  {
    ros_msg.header.stamp = this->get_clock()->now();
    ros_msg.header.frame_id = "world";
    ros_msg.pose.orientation.w = ign_msg.orientation().w();
    ros_msg.pose.orientation.x = ign_msg.orientation().x();
    ros_msg.pose.orientation.y = ign_msg.orientation().y();
    ros_msg.pose.orientation.z = ign_msg.orientation().z();
    ros_msg.pose.position.x = ign_msg.position().x();
    ros_msg.pose.position.y = ign_msg.position().y();
    ros_msg.pose.position.z = ign_msg.position().z();
  }

  void ign_callback(const ignition::msgs::Pose_V &ign_msg, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_pub)
  {
    auto size_ = ign_msg.pose_size();
    for (int i = 0; i < size_; i++)
    {
      std::string topic_name = ros_pub->get_topic_name();
      size_t firstSlash = topic_name.find('/');
      size_t secondSlash = topic_name.find('/', firstSlash + 1);
      std::string vehicle_name = topic_name.substr(firstSlash + 1, secondSlash - firstSlash - 1);
      if (std::string(ign_msg.pose(i).name()) == vehicle_name)
      {
        geometry_msgs::msg::PoseStamped ros_msg;
        convert_ign_to_ros2(ign_msg.pose(i), ros_msg);
        ros_pub->publish(ros_msg);
      }
    }
  }

  void create_ign_subscriber(
    std::shared_ptr<ignition::transport::Node> node,
    const std::string &topic_name,
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_pub)
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
    auto ros_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(ros_topic_name, 10);

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
    std::cerr << "Usage: parameter_bridge <vehicle_numbers> ..." << std::endl;
    return -1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseBridge>();

  auto ign_node = std::make_shared<ignition::transport::Node>();
  std::list<BridgeIgnToRos2Handles> ign_to_ros2_handles;

  std::string ign_topic_name("/world/coast/pose/info");

  size_t vehicle_numbers = std::stoi(argv[1]);
  printf("Creating %zu vehicle pose bridges\n", vehicle_numbers);
  for (size_t i = 1; i <= vehicle_numbers; ++i)
  {
    std::string vehicle_name = std::string("uav_") + std::to_string(i);
    std::string ros_topic_name = vehicle_name + std::string("/pose/groundtruth");
    printf("Creating ROS 2 topic [%s]\n", ros_topic_name.c_str());

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
