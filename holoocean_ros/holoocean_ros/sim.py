from threading import Thread
import rclpy
from rclpy.node import Node
import holoocean
import numpy as np
from pynput import keyboard
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32
from cv_bridge import CvBridge
from copy import deepcopy
from geometry_msgs.msg import Twist, Point
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from threading import Lock
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from holoocean_msgs.msg import DVL # type: ignore
import json

# TODO
# multiagent support
# launchfiles and config options
# migeran compatibility



class AgentNode(Node):
    def __init__(self, cfg, num, env, callback_group):
        super().__init__(f"holoocean_agent{num}")
        
        # save config and env
        self.cfg = deepcopy(cfg)
        self.env = env

        # create bridge for image conversion
        self.bridge = CvBridge()

        # create sensor topic names and publishers based
        self.topic_root = f"/holoocean/agent{num}/"
        typecount = {}
        for i, sensor in enumerate(self.cfg['sensors']):
            if sensor['sensor_type'] not in typecount:
                typecount[sensor['sensor_type']] = 0
            else:
                typecount[sensor['sensor_type']] += 1

            self.cfg['sensors'][i]['topic'] = f"{sensor['sensor_type']}{typecount[sensor['sensor_type']]}"
            match sensor['sensor_type']:
                case "RGBCamera":
                    self.cfg['sensors'][i]['pub'] = self.create_publisher(Image, self.topic_root + self.cfg['sensors'][i]['topic'], 1)
                case "ViewportCapture":
                    self.cfg['sensors'][i]['pub'] = self.create_publisher(Image, self.topic_root + self.cfg['sensors'][i]['topic'], 1)
                case "ImagingSonar":
                    self.cfg['sensors'][i]['pub'] = self.create_publisher(Image, self.topic_root + self.cfg['sensors'][i]['topic'], 1)
                case "ProfilingSonar":
                    self.cfg['sensors'][i]['pub'] = self.create_publisher(Image, self.topic_root + self.cfg['sensors'][i]['topic'], 1)
                case "DVLSensor":
                    self.cfg['sensors'][i]['pub'] = self.create_publisher(DVL, self.topic_root + self.cfg['sensors'][i]['topic'], 1)
                case "GPSSensor":
                    self.cfg['sensors'][i]['pub'] = self.create_publisher(Point, self.topic_root + self.cfg['sensors'][i]['topic'], 1)
                case "DepthSensor":
                    self.cfg['sensors'][i]['pub'] = self.create_publisher(Float32, self.topic_root + self.cfg['sensors'][i]['topic'], 1)

        # create empty base command
        self.command = np.zeros((8))
        self.command_lock = Lock()
        
        # subscribe to control topic
        self.create_subscription(Twist, f"{self.topic_root}cmd_vel", self.command_callback, 1, callback_group=callback_group)
        self.get_logger().info(f'made subscriber at {self.topic_root}cmd_vel')

    def command_callback(self, msg):
        command = np.zeros(8)
        
        # UP/DOWN
        command[0:4] += msg.linear.z

        # YAW
        command[[4,7]] -= msg.angular.z
        command[[5,6]] += msg.angular.z

        # FORWARD/BACKWARD
        command[4:8] += msg.linear.y

        # STRAFE LEFT
        command[[4,6]] -= msg.linear.x
        command[[5,7]] += msg.linear.x
        
        with self.command_lock:
            self.command = command
        
    def camera_to_image(self, np_img: np.ndarray, frame_id: str = "camera") -> Image:
        msg = self.bridge.cv2_to_imgmsg(np_img, encoding='bgra8')
        msg.header = Header()
        msg.header.stamp = rclpy.time.Time().to_msg() # type: ignore
        msg.header.frame_id = frame_id
        return msg
    
    def sonar_to_image(self, array_2d):
        arr = np.array(array_2d, dtype=np.float32)
        arr_uint8 = (arr * 255).astype(np.uint8)
        bridge = CvBridge()
        return bridge.cv2_to_imgmsg(arr_uint8, encoding='mono8')
    
    def tick(self, state):
        # update ros topics
        for i, (sensor, val) in enumerate(state.items()):
            new_msg = None
            match sensor:
                case "RGBCamera":
                    new_msg = self.camera_to_image(val)
                case "ViewportCapture":
                    new_msg = self.camera_to_image(val)
                case "ImagingSonar":
                    new_msg = self.sonar_to_image(val)
                case "ProfilingSonar":
                    new_msg = self.sonar_to_image(val)
                case "DVLSensor":
                    new_msg = DVL()
                    new_msg.velocity_x = float(val[0])
                    new_msg.velocity_y = float(val[1])
                    new_msg.velocity_z = float(val[2])
                    new_msg.range_x_forw = float(val[3])
                    new_msg.range_y_forw = float(val[4])
                    new_msg.range_x_back = float(val[5])
                    new_msg.range_y_back = float(val[6])
                case "GPSSensor":
                    new_msg = Point()
                    new_msg.x = val[0]
                    new_msg.y = val[1]
                    new_msg.z = val[2]
                case "DepthSensor":
                    new_msg = Float32()
                    new_msg.data = float(val[0])
            self.cfg['sensors'][i]['pub'].publish(new_msg)
        
        with self.command_lock:
            self.env.act(self.cfg['agent_name'], self.command)
        

class EnvNode(Node):
    def __init__(self, agent_callback_group):
        super().__init__(f"holoocean_driver")

        self.declare_parameter('config_path', "")

        # load config json
        if self.get_parameter('config_path').get_parameter_value().string_value == "":
            raise FileNotFoundError('no config file provided')
        with open(self.get_parameter('config_path').get_parameter_value().string_value) as f:
            cfg = json.load(f)
        self.cfg = cfg
        
        # create sim environment
        self.env = holoocean.make(scenario_cfg=self.cfg, show_viewport=True, verbose=True)
        # self.env.should_render_viewport(True)
        # self.env.set_render_quality(3)
        self.env.weather.set_fog_density(0.5)
        
        # create agent nodes
        self.agents = []
        for i, agent_cfg in enumerate(cfg['agents']):
            self.agents.append(AgentNode(agent_cfg, i, self.env, agent_callback_group))
        
        # update environment at provided tick rate
        self.create_timer(1 / cfg['ticks_per_sec'], self.tick)

    def tick(self):
        # one simulation tick
        state = self.env.tick()

        # update state format to be consistent with single agent
        if len(self.cfg['agents']) == 1:
            new_state = {
                self.cfg['agents'][0]['agent_name']: state,
                't': state['t']
            }
            del new_state[self.cfg['agents'][0]['agent_name']]['t']
            state = new_state

        # send state to all agents
        for i, agent in enumerate(self.agents):
            agent.tick(state[self.cfg['agents'][i]['agent_name']])
                

def main(args=None):
    rclpy.init(args=args)

    # with open('config.json') as f:
    #     cfg = json.load(f)

    # make env
    env_node = EnvNode(ReentrantCallbackGroup())

    # add agents to shared executor
    executor = MultiThreadedExecutor(num_threads=len(env_node.agents) + 1)
    for agent in env_node.agents:
        executor.add_node(agent)

    # spin until the heat death of the universe
    try:
        while True:
            rclpy.spin_once(env_node)
            executor.spin_once(timeout_sec=0)
    except KeyboardInterrupt:
        env_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
