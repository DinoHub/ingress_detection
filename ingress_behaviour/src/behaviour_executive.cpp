/*
ros2 node with service server to start behaviour
/clear_ingress: service server to start behaviour
/detections: sub to bbox of ingress door/window
/?? : sub to depth image
*/

// Include the C++ standard library headers
#include <memory> // Dynamic memory management. make_shared
 
// Dependencies
#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
// #include "std_msgs/msg/string.hpp" // Handles String messages in ROS 2
using std::placeholders::_1;
#include "../include/ingress_behaviour/behaviour_executive.h"
#include "robot_interfaces/msg/bounding_boxes.hpp"

void BehaviourExecutive::detections_callback(const robot_interfaces::msg::BoundingBoxes::SharedPtr msg)
{
    //TODO: store detection into detection_info
    detection_info.xmin = msg->boxes[0].xmin;
    RCLCPP_INFO(this->get_logger(), "I heard: '%i'", msg->boxes[0].xmin);
    RCLCPP_INFO(this->get_logger(), "stored: '%i'", detection_info.xmin);
}

// //TODO: add depth msg type
// void BehaviourExecutive::depth_callback(const ? msg) 
// {
//   RCLCPP_INFO(this->get_logger(), "receiving depth data");
// }

BehaviourExecutive::BehaviourExecutive(): Node("behaviour_executive")
{
  // Creates subscribers for detection and depth data
  detection_subscription = this->create_subscription<robot_interfaces::msg::BoundingBoxes>(
    "detections", 10, std::bind(&BehaviourExecutive::detections_callback, this, _1));
// TODO: add depth msg type
//   depth_subscription = this->create_subscription<?>(
//     "?", 10, std::bind(&BehaviourExecutive::depth_callback, this, _1));
  RCLCPP_INFO(this->get_logger(), "node started!!");
}
 
int main(int argc, char * argv[])
{
  // Launch ROS 2
  rclcpp::init(argc, argv);
   
  // Prepare to receive messages that arrive on the topic
  rclcpp::spin(std::make_shared<BehaviourExecutive>());
   
  // Shutdown routine for ROS2
  rclcpp::shutdown();
  return 0;
}