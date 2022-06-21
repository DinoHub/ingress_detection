## Requirements
1. ros2 foxy and colcon

## How to use
1. Clone into your ros2_ws/src directory
2. From ros2_ws, `rosdep install -i --from-path src --rosdistro foxy -y`
3. Build using `colcon build --packages-select robot_interfaces ingress_detector --symlink-install`
4. Source your install with `. install/local_setup.bash`
5. Test the node with `ros2 run ingress_detector detector`
6. Start your webcam node with `ros2 run ingress_detector webcam_pub` or other camera drivers.

## TODO
- [ ] test with realsense
- [ ] add actual detection model in, add drawing code to visualize BB  
- [ ] test that detection msg are correctly published  

## Packages
- ingress_detector main package which contains object detection model and publishes the detections. 
- robot_interfaces: custom msg for detection bounding boxes.
