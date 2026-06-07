import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

import cv2
import numpy as np
import random
import math
import tf2_ros
import time


class MultiObjectNavigator(Node):

    def __init__(self):
        super().__init__('multi_object_navigator')

        self.state = "IDLE"
        self.goal_handle = None
        self.current_target = None  # "RED", "PURPLE", or "BLUE"
        self.final_target = None  # For two-phase navigation (curtain → blue)
        
        # Object coordinates
        self.object_coords = {
            'RED': {'x': 0.6066, 'y': 7.5384},
            'PURPLE': {'x': 7.5612, 'y': -0.7680},
            'BLUE': {'x': -7.076, 'y': 7.815},
            'CURTAIN': {'x': -6.0, 'y': 5.6}
        }
        
        # For paper's approach: track traversable obstacles
        self.traversable_obstacles = set()
        
        self.frames_without_object = 0
        self.max_frames_lost = 50

        self.last_goal_time = 0
        self.goal_send_interval = 0.5

        self.center_tolerance = 40
        self.target_object_height = 260
        self.visual_servo_trigger = 120
        
        self.max_linear_speed = 0.12
        self.min_linear_speed = 0.04
        self.angular_gain = 0.005

        self.robot_radius = 0.25
        self.safety_margin = 0.20
        self.stop_distance = self.robot_radius + self.safety_margin

        self.lidar_ranges = None
        self.mission_complete = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.create_subscription(String, '/robot_action', self.command_callback, 10)
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.camera_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.1, self.safety_timer_callback)

        self.get_logger().info("Multi-Object Navigator Ready (Red/Purple/Blue)")

    def safety_timer_callback(self):
        if self.mission_complete or self.state == "IDLE":
            self.cmd_pub.publish(Twist())

    def command_callback(self, msg):

        # ORIGINAL APPROACH: RED (unchanged)
        if msg.data == "START_RED" and self.state == "IDLE":
            self.get_logger().info("Starting RED object search")
            self.current_target = "RED"
            self.traversable_obstacles.clear()
            self.state = "EXPLORING"
            self.mission_complete = False
            self.send_random_goal()
        
        # ORIGINAL APPROACH: PURPLE (unchanged)
        elif msg.data == "START_PURPLE" and self.state == "IDLE":
            self.get_logger().info("Starting PURPLE object search")
            self.current_target = "PURPLE"
            self.traversable_obstacles.clear()
            self.state = "EXPLORING"
            self.mission_complete = False
            self.send_random_goal()
        
        # PAPER'S APPROACH: BLUE (through curtain)
        elif msg.data == "START_BLUE_THROUGH_CURTAIN" and self.state == "IDLE":
            self.get_logger().info("Starting BLUE object search (paper's approach)")
            self.get_logger().info("Action-aware attributes:")
            self.get_logger().info("  - Curtain: TRAVERSABLE (Pa=1)")
            self.get_logger().info("  - Blue object: GOAL")
            self.get_logger().info("Phase 1: Navigate to curtain")
            
            self.traversable_obstacles.add('CURTAIN')
            self.current_target = "CURTAIN"
            self.final_target = "BLUE"
            self.state = "NAVIGATING"
            self.mission_complete = False
            self.send_approach_goal()

        elif msg.data == "STOP":
            self.cancel_goal()
            self.cmd_pub.publish(Twist())
            self.state = "IDLE"
            self.current_target = None
            self.final_target = None
            self.traversable_obstacles.clear()
            self.mission_complete = True

    def check_safety(self):
        if self.lidar_ranges is None:
            return True
        
        front_dist = self.get_front_distance()
        
        # Paper's approach: check if we're near curtain area
        if 'CURTAIN' in self.traversable_obstacles:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y
                
                curtain_x = self.object_coords['CURTAIN']['x']
                curtain_y = self.object_coords['CURTAIN']['y']
                
                dist_to_curtain = math.sqrt((robot_x - curtain_x)**2 + (robot_y - curtain_y)**2)
                
                # If within 3m of curtain, ignore all safety checks
                if dist_to_curtain < 3.0:
                    self.get_logger().info(f"Near curtain ({dist_to_curtain:.2f}m), ignoring obstacles")
                    return True
            except:
                pass
        
        # Original approach: normal safety checks
        if front_dist < self.stop_distance:
            self.cancel_goal()
            for _ in range(5):
                self.cmd_pub.publish(Twist())
            
            if self.state == "EXPLORING":
                self.send_random_goal()
                return False
            elif self.state == "NAVIGATING" or self.state == "VISUAL_SERVO":
                self.get_logger().info(f"SUCCESS! Stopped at {front_dist:.2f}m from {self.current_target} object")
                self.state = "IDLE"
                self.current_target = None
                self.final_target = None
                self.traversable_obstacles.clear()
                self.mission_complete = True
                return False
        
        return True

    def send_random_goal(self):

        if not self.action_client.wait_for_server(timeout_sec=3.0):
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Wider exploration range for better coverage
        goal_msg.pose.pose.position.x = random.uniform(-8.0, 8.0)
        goal_msg.pose.pose.position.y = random.uniform(-8.0, 8.0)
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Exploring → ({goal_msg.pose.pose.position.x:.2f}, "
            f"{goal_msg.pose.pose.position.y:.2f})"
        )

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def send_approach_goal(self):
        
        if self.current_target is None:
            return
        
        current_time = time.time()
        if current_time - self.last_goal_time < self.goal_send_interval:
            return
        
        if not self.action_client.wait_for_server(timeout_sec=3.0):
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
        except:
            return

        target_coords = self.object_coords[self.current_target]
        dx = target_coords['x'] - robot_x
        dy = target_coords['y'] - robot_y
        distance = math.sqrt(dx**2 + dy**2)
        
        self.get_logger().info(f"Distance to {self.current_target}: {distance:.2f}m")
        
        # Switch to visual servo when close
        if distance < 1.2:
            # If we reached curtain, now go to final target (blue)
            if self.current_target == "CURTAIN" and self.final_target == "BLUE":
                self.get_logger().info("Reached curtain! Now navigating to BLUE object")
                self.current_target = "BLUE"
                self.final_target = None
                self.state = "NAVIGATING"
                return
            
            # For all targets: switch to visual servo
            self.get_logger().info(f"Close ({distance:.2f}m), switching to visual servo")
            self.cancel_goal()
            self.state = "VISUAL_SERVO"
            return
        
        # For BLUE through curtain: when close but not too close, stop and let camera take over
        if 'CURTAIN' in self.traversable_obstacles and self.current_target == "BLUE" and distance < 2.0:
            self.get_logger().info(f"Near BLUE target ({distance:.2f}m), letting camera take over")
            self.cancel_goal()
            # Stay in NAVIGATING - camera will detect and switch to visual servo
            return
        
        # PAPER'S APPROACH: Direct navigation for BLUE/CURTAIN
        if 'CURTAIN' in self.traversable_obstacles:
            goal_x = target_coords['x']
            goal_y = target_coords['y']
            self.get_logger().info(f"Direct Nav2 goal to {self.current_target}: ({goal_x:.2f}, {goal_y:.2f})")
        else:
            # ORIGINAL APPROACH: Adaptive steps for RED/PURPLE
            if distance < 2.0:
                step = 0.5
            elif distance < 4.0:
                step = 0.9
            else:
                step = 1.3
            step = min(step, distance * 0.65)
            
            goal_x = robot_x + (dx / distance) * step
            goal_y = robot_y + (dy / distance) * step
            self.get_logger().info(f"Nav2 step: ({goal_x:.2f}, {goal_y:.2f}) - {step:.2f}m toward {self.current_target}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.last_goal_time = current_time

    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected goal")
            
            if self.state == "EXPLORING":
                self.send_random_goal()
            elif self.state == "NAVIGATING":
                if self.current_target in ["RED", "PURPLE"]:
                    self.state = "EXPLORING"
                    self.send_random_goal()
                else:
                    # For BLUE, keep trying
                    self.send_approach_goal()
            return

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):

        result = future.result()
        self.get_logger().info(f"Goal completed")

        if self.state == "EXPLORING":
            self.send_random_goal()
        elif self.state == "NAVIGATING":
            # Continue approaching
            self.send_approach_goal()

    def camera_callback(self, msg):

        if self.state == "IDLE" or self.mission_complete or self.current_target is None:
            return

        if not self.check_safety():
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect based on current target
        if self.current_target == "RED":
            mask = (
                cv2.inRange(hsv, np.array([0,70,50]), np.array([10,255,255])) +
                cv2.inRange(hsv, np.array([170,70,50]), np.array([180,255,255]))
            )
        elif self.current_target == "PURPLE":
            mask = cv2.inRange(hsv, np.array([125,70,80]), np.array([155,255,255]))
        elif self.current_target == "BLUE":
            mask = cv2.inRange(hsv, np.array([100, 120, 80]), np.array([135, 255, 255]))
        elif self.current_target == "CURTAIN":
            # Don't detect curtain in camera (coordinate-only)
            return
        else:
            return

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self.frames_without_object += 1
            
            if self.state == "NAVIGATING" and self.frames_without_object > 10:
                self.send_approach_goal()
                self.frames_without_object = 0
            
            if self.state == "VISUAL_SERVO" and self.frames_without_object > self.max_frames_lost:
                self.get_logger().info(f"Mission complete - {self.current_target} reached")
                for _ in range(5):
                    self.cmd_pub.publish(Twist())
                self.state = "IDLE"
                self.current_target = None
                self.final_target = None
                self.traversable_obstacles.clear()
                self.mission_complete = True
            
            return

        largest = max(contours, key=cv2.contourArea)

        if cv2.contourArea(largest) < 400:
            self.frames_without_object += 1
            if self.state == "NAVIGATING" and self.frames_without_object > 10:
                self.send_approach_goal()
                self.frames_without_object = 0
            return

        self.frames_without_object = 0

        x, y, w, h = cv2.boundingRect(largest)
        cx = x + w // 2
        object_height = h
        
        image_center_x = frame.shape[1] // 2
        error_x = cx - image_center_x

        if self.state == "EXPLORING":
            self.get_logger().info(f"{self.current_target} spotted (h={object_height}px)!")
            self.cancel_goal()
            self.state = "NAVIGATING"
            return

        if self.state == "NAVIGATING":
            
            if object_height > self.visual_servo_trigger:
                self.get_logger().info(f"{self.current_target} large ({object_height}px)")
                self.cancel_goal()
                self.state = "VISUAL_SERVO"
                return
            
            self.send_approach_goal()
            return

        if self.state == "VISUAL_SERVO":
            self.visual_servo_control(error_x, object_height, frame.shape)

    def visual_servo_control(self, error_x, object_height, image_shape):

        twist = Twist()

        if object_height >= self.target_object_height:
            self.get_logger().info(f"SUCCESS! Centered on {self.current_target} (h={object_height}px)")
            for _ in range(10):
                self.cmd_pub.publish(Twist())
            
            self.state = "IDLE"
            self.current_target = None
            self.final_target = None
            self.traversable_obstacles.clear()
            self.mission_complete = True
            return

        if abs(error_x) > self.center_tolerance:
            twist.angular.z = -self.angular_gain * error_x
            twist.linear.x = 0.0
        else:
            remaining = self.target_object_height - object_height
            speed_ratio = min(1.0, remaining / 100.0)
            speed_ratio = max(0.3, speed_ratio)
            
            twist.linear.x = self.min_linear_speed + \
                           (self.max_linear_speed - self.min_linear_speed) * speed_ratio
            twist.angular.z = -self.angular_gain * error_x * 0.2

        self.cmd_pub.publish(twist)

    def scan_callback(self, msg):

        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = 10.0
        ranges[np.isinf(ranges)] = 10.0
        ranges[np.isnan(ranges)] = 10.0

        self.lidar_ranges = ranges

    def get_front_distance(self):

        if self.lidar_ranges is None:
            return 10.0

        center = len(self.lidar_ranges) // 2
        return np.min(self.lidar_ranges[center - 25:center + 25])

    def cancel_goal(self):

        if self.goal_handle:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None


def main(args=None):

    rclpy.init(args=args)
    node = MultiObjectNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
