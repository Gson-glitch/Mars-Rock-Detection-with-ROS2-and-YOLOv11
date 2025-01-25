import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import os
import math

class RockDetectionNode(Node):
    def __init__(self):
        super().__init__('rock_detection_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('confidence_threshold', 0.5),
                ('turtle_linear_speed', 0.5),
                ('turtle_angular_scale', 1.0),
                ('min_distance_to_wall', 1.0),
                ('max_x', 11.0),
                ('max_y', 11.0),
                ('min_x', 0.0),
                ('min_y', 0.0)
            ]
        )
        
        try:
            model_path = os.path.join(os.path.dirname(__file__), 'models', 'best.pt')
            self.get_logger().info(f'Loading model from: {model_path}')
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            raise
        
        # Publishers
        self.detection_pub = self.create_publisher(Image, 'rock_detections', 10)
        self.turtle_cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Subscribers
        self.video_sub = self.create_subscription(
            Image,
            'video_stream',
            self.video_callback,
            10)
        
        # Turtle state
        self.current_x = 5.544445 
        self.current_y = 5.544445
        self.current_theta = 0.0
        
        self.get_logger().info('Rock Detection Node initialized')

    def is_safe_movement(self, cmd_vel, duration=0.1):
        """Check if proposed movement would lead to collision"""
        next_x = self.current_x + (cmd_vel.linear.x * math.cos(self.current_theta) * duration)
        next_y = self.current_y + (cmd_vel.linear.x * math.sin(self.current_theta) * duration)
        
        min_dist = self.get_parameter('min_distance_to_wall').value
        max_x = self.get_parameter('max_x').value - min_dist
        max_y = self.get_parameter('max_y').value - min_dist
        min_x = self.get_parameter('min_x').value + min_dist
        min_y = self.get_parameter('min_y').value + min_dist
        
        return min_x <= next_x <= max_x and min_y <= next_y <= max_y

    def adjust_movement_for_safety(self, cmd_vel):
        """Adjust movement commands to avoid walls"""
        if not self.is_safe_movement(cmd_vel):
            cmd_vel.linear.x *= 0.5
            
            if not self.is_safe_movement(cmd_vel):
                cmd_vel.linear.x = 0.0
                
                if abs(cmd_vel.angular.z) > 0.1:
                    cmd_vel.angular.z *= 0.7
        
        return cmd_vel

    def move_turtle_towards_rock(self, rock_position, image_width):
        try:
            relative_x = (rock_position[0] - image_width/2) / (image_width/2)
    
            linear_speed = self.get_parameter('turtle_linear_speed').value
            angular_scale = self.get_parameter('turtle_angular_scale').value
            
            cmd = Twist()
            cmd.linear.x = linear_speed
            cmd.angular.z = -relative_x * angular_scale
            
            angle_factor = 1.0 - min(abs(relative_x), 0.8) 
            cmd.linear.x *= angle_factor
            safe_cmd = self.adjust_movement_for_safety(cmd)
            
            self.current_theta += safe_cmd.angular.z * 0.1 
            self.current_x += safe_cmd.linear.x * math.cos(self.current_theta) * 0.1
            self.current_y += safe_cmd.linear.x * math.sin(self.current_theta) * 0.1

            self.turtle_cmd_pub.publish(safe_cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error moving turtle: {str(e)}')

    def video_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            detection_image = cv_image.copy()
            results = self.model(cv_image)
            
            largest_rock = None
            max_area = 0
            conf_thresh = self.get_parameter('confidence_threshold').value
            
            for result in results:
                boxes = result.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    confidence = box.conf[0]
                    
                    if confidence > conf_thresh:
                        area = (x2 - x1) * (y2 - y1)
                        
                        if area > max_area:
                            max_area = area
                            largest_rock = (int((x1 + x2)/2), int((y1 + y2)/2))
                        
                        cv2.rectangle(detection_image, 
                                    (int(x1), int(y1)), 
                                    (int(x2), int(y2)), 
                                    (0, 255, 0), 2)
                        cv2.putText(detection_image, 
                                  f'Rock: {confidence:.2f}', 
                                  (int(x1), int(y1-10)), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.5, 
                                  (0, 255, 0), 
                                  2)
            
            if largest_rock:
                cv2.circle(detection_image, largest_rock, 10, (0, 0, 255), -1)
                self.move_turtle_towards_rock(largest_rock, cv_image.shape[1])
            
            detection_msg = self.bridge.cv2_to_imgmsg(detection_image, encoding='bgr8')
            self.detection_pub.publish(detection_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = RockDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()