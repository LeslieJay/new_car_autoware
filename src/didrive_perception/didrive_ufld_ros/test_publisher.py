#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 100)
        self.timer = self.create_timer(1.0, self.publish_image)  # 1Hz
        self.bridge = CvBridge()
        
        # 加载图像
        self.image = cv2.imread('./img_for_test/1746689220133.jpg')
        if self.image is None:
            self.get_logger().error('无法加载图像！')
        else:
            self.get_logger().info('图像加载成功，开始发布')
        
    def publish_image(self):
        if self.image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.image, 'bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('发布图像')

def main():
    rclpy.init()
    node = TestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()