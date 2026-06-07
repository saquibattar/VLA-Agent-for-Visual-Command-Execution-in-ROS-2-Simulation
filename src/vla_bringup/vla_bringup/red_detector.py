import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

class MultiObjectDetector(Node):
    def __init__(self):
        super().__init__('multi_object_detector')
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publish bearings for each color (BLUE REMOVED)
        self.red_bearing_pub = self.create_publisher(Float32, '/red_object/bearing', 10)
        self.purple_bearing_pub = self.create_publisher(Float32, '/purple_object/bearing', 10)
        
        self.get_logger().info('Multi-Object Detector started (Red, Purple)')
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Detect RED
        self.detect_and_publish(hsv, frame, 
                               lower1=np.array([0, 70, 50]), upper1=np.array([10, 255, 255]),
                               lower2=np.array([170, 70, 50]), upper2=np.array([180, 255, 255]),
                               publisher=self.red_bearing_pub)
        
        # Detect PURPLE (loosened to account for Gazebo rendering)
        # Still filters most walls but detects the actual object
        self.detect_and_publish(hsv, frame,
                               lower1=np.array([125, 70, 80]), upper1=np.array([155, 255, 255]),
                               lower2=None, upper2=None,
                               publisher=self.purple_bearing_pub)
        
        # BLUE DETECTION REMOVED
    
    def detect_and_publish(self, hsv, frame, lower1, upper1, lower2, upper2, publisher):
        # Create mask
        if lower2 is not None and upper2 is not None:
            mask = cv2.inRange(hsv, lower1, upper1) + cv2.inRange(hsv, lower2, upper2)
        else:
            mask = cv2.inRange(hsv, lower1, upper1)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return
        
        largest = max(contours, key=cv2.contourArea)
        
        if cv2.contourArea(largest) < 200:
            return
        
        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        image_width = frame.shape[1]
        error_pixels = cx - image_width // 2
        
        # Convert pixel error → radians
        bearing = -error_pixels / image_width * 1.0
        
        msg_out = Float32()
        msg_out.data = float(bearing)
        publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = MultiObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
