import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import json
from scipy.spatial.transform import Rotation as R
import numpy as np
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

class Migeran(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.declare_parameter('intensity_threshold', 85)
        self.declare_parameter('config_path', "")
        self.declare_parameter('agent_name', "auv0")
        self.declare_parameter('frame_id', "map")
        self.declare_parameter('sensor_name', "ImagingSonar")

        if self.get_parameter('config_path').get_parameter_value().string_value == "":
            raise FileNotFoundError('no config file provided')
        with open(self.get_parameter('config_path').get_parameter_value().string_value) as f:
            config = json.load(f)
        
        base_config = {}
        for agent in config['agents']:
            if agent['agent_name'] == self.get_parameter('agent_name').get_parameter_value().string_value:
                for sensor in agent['sensors']:
                    if 'sensor_name' in sensor.keys():
                        if sensor['sensor_name'] == self.get_parameter('sensor_name').get_parameter_value().string_value:
                            base_config = sensor['configuration']
        
        config = base_config
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
            header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
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