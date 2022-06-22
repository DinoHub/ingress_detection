'''
ros2 node which runs object detection, publishes detection
/video_frames(sensor_msgs/Image): sub to rgb image 
/detections(robot_interfaces/msg/BoundingBoxes): publishes detection bounding boxes
'''
import random
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from robot_interfaces.msg import BoundingBox, BoundingBoxes # custom msg
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 


class IngressDetector(Node):
    """
    Create an IngressDetector class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        model: object detection model
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('ingress_detector')
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()
        # self.model = model
        # Subscribes to incoming video. The queue size is 10 messages.
        self.subscription = self.create_subscription(\
            Image, 'video_frames', self.listener_callback, 10)
        self.subscription # prevent unused variable warning
        # Publishes detection bounding boxes from model
        self.publisher_ = self.create_publisher(\
            BoundingBoxes, 'detections', 10)
        self.boxes_msg = BoundingBoxes()
        self.get_logger().info(f'ingress_detector node has started!')

    def listener_callback(self, data):
        """
        Callback function when video frame is received.
        """
        # Display the message on the console
        self.get_logger().debug(f'Receiving video frame {data.height} by {data.width}')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data)
        
        # TODO: run prediction with detection model 
        # detections = self.model.predict(current_frame)
        # Create a new msg, fill it, and publish.
        self.boxes_msg = BoundingBoxes()
        # for detection in detections:
        self.pack_rosmsg()
        self.publisher_.publish(self.boxes_msg)

        # DEBUG: Display image
        cv2.imshow("camera", current_frame)
        cv2.waitKey(1)

    def pack_rosmsg(self, input = None):
        box = BoundingBox()
        # box.class_id = ?
        # box.confidence = ?
        box.xmin = random.randint(1,50)
        # box.ymin = ?
        # box.xmax = ?
        # box.ymax = ?
        # box.img_width = ?
        # box.img_height = ?
        self.boxes_msg.boxes.append(box)

  
def main(args=None):
  
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # TODO: load detection model and pass into node

    # Create the node
    ingress_detector = IngressDetector()
    
    # Spin the node so the callback function is called.
    rclpy.spin(ingress_detector)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ingress_detector.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()