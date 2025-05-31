import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Migeran(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/holoocean/agent0/ProfilingSonar0', self.sonar_cb, 1)

    def sonar_cb(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv2.imshow('Sonar Image', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
        


def main(args=None):
    rclpy.init(args=args)
    node = Migeran()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()