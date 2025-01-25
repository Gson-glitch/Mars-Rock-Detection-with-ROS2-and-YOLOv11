import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher_node')
        
        self.declare_parameter('video_source', '0')  
        self.video_source = self.get_parameter('video_source').value
        
        if self.video_source.isdigit():
            self.video_source = int(self.video_source)
            
        self.publisher = self.create_publisher(Image, 'video_stream', 10)
        self.bridge = CvBridge()
        
        self.cap = cv2.VideoCapture(self.video_source)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open video source: {self.video_source}')
            return
            
        self.timer = self.create_timer(1/30.0, self.timer_callback)
        
        self.get_logger().info(f'Video publisher initialized with source: {self.video_source}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(msg)
        else:
            if isinstance(self.video_source, str):  
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0) 
                self.get_logger().info('Restarting video from beginning')
            else:
                self.get_logger().error('Failed to read from video source')

    def __del__(self):
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()