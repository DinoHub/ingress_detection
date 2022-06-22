#include "rclcpp/rclcpp.hpp" // ROS Clienty Library for C++
#include "robot_interfaces/msg/bounding_boxes.hpp"

struct detectionInfo {
  public:    
    std::string class_id;
    float confidence;
    int xmin, ymin, xmax, ymax;
    int img_width, img_height;
};

class BehaviourExecutive : public rclcpp::Node
{
  public:
    // Constructor. Node named here.
    BehaviourExecutive();
 
  private:
    // create detect and depth data structure 
    detectionInfo detection_info;

    // Callbacks for detection and depth to update values stored
    void detections_callback(const robot_interfaces::msg::BoundingBoxes::SharedPtr msg);
    // TODO: add depth msg type
    // void depth_callback(const ? msg);  
    
    // Declare the subscription attribute
    rclcpp::Subscription<robot_interfaces::msg::BoundingBoxes>::SharedPtr detection_subscription;
    // TODO: add depth msg type
    // rclcpp::Subscription<?>::SharedPtr depth_subscription;
};