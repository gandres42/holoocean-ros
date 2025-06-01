import rclpy
import rclpy.clock
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import json
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header




class Migeran(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.declare_parameter('intensity_threshold', 100)
        self.declare_parameter('config_path', "")

        if self.get_parameter('config_path').get_parameter_value().string_value == "":
            raise FileNotFoundError('no config file provided')
        with open(self.get_parameter('config_path').get_parameter_value().string_value) as f:
            config = json.load(f)
        
        config = config['agents'][0]['sensors'][-1]["configuration"]
        self.azi = config['Azimuth']
        self.minR = config['RangeMin']
        self.maxR = config['RangeMax']
        self.binsR = config['RangeBins']
        self.binsA = config['AzimuthBins']

        self.bridge = CvBridge()
        self.create_subscription(Image, '/holoocean/auv0/ImagingSonar', self.sonar_cb, 1)
        self.cloud_pub = self.create_publisher(PointCloud2, '/holoocean/auv0/SonarPoints', 10)

    def sonar_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            h, w = img.shape
            center = (w // 2, h // 2)
            max_radius = np.sqrt((w / 2)**2 + (h / 2)**2)

            points = []

            for r in range(img.shape[0]):
                for c in range(img.shape[1]):
                    if img[r, c] < self.get_parameter('intensity_threshold').get_parameter_value().integer_value:
                        continue

                    current_distance = (r * (self.maxR - self.minR)) / img.shape[0] + self.minR
                    current_angle = (self.azi / 2) - ((self.azi * c) / img.shape[1])
                    theta = current_angle * (np.pi / 180)

                    point_vector = np.array([current_distance, 0.0, 0.0])

                    point_vector = point_vector @ R.from_euler('xyz', [0, 0, theta]).as_matrix()
                    point_vector[1] *= -1
                    points.append(point_vector)
            points = np.array(points)
            header = Header()
            # header.stamp = self.get_clock().
            header.frame_id = "map"
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
            

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