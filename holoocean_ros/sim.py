from threading import Thread
import rclpy
from rclpy.node import Node
import holoocean
import numpy as np
from pynput import keyboard
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge
from copy import deepcopy
from geometry_msgs.msg import Twist
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from threading import Lock
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

test_cfg = {
    "name": "test_rgb_camera",
    "world": "SimpleUnderwater",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 60,
    "frames_per_sec": False,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "BlueROV2",
            "sensors": [
                {
                    "sensor_type": "RGBCamera",
                    "name": "cam0",
                    "socket": "CameraSocket",
                    "configuration": {
                        "CaptureWidth": 720,
                        "CaptureHeight": 480
                    }
                },
                # {
                #     "sensor_type": "ViewportCapture"
                # }
                # {
                #     "sensor_type": "ProfilingSonar",
                #     "socket": "SonarSocket",
                #     "octree_min": 1,
                #     "octree_max": 2,
                #     # "Hz": 1,
                #     # "configuration": {
                #     #     "RangeBins": 512,
                #     #     "AzimuthBins": 512,
                #     #     "RangeMin": 1,
                #     #     "RangeMax": 40,
                #     #     "InitOctreeRange": 50,
                #     #     "Elevation": 20,
                #     #     "Azimuth": 120,
                #     #     "AzimuthStreaks": -1,
                #     #     "ScaleNoise": True,
                #     #     "AddSigma": 0.15,
                #     #     "MultSigma": 0.2,
                #     #     "RangeSigma": 0.1,
                #     #     "MultiPath": True
                #     # }
                # }
            ],
            "control_scheme": 0,
            # "location": [486.0, -632.0, -12.0],
            "rotation": [0.0, 0.0, 135.0]
        },
    ]
}

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
            match sensor:
                case "RGBCamera":
                    self.cfg['sensors'][i]['pub'].publish(self.camera_to_image(val))
                case "ViewportCapture":
                    self.cfg['sensors'][i]['pub'].publish(self.camera_to_image(val))
                case "ImagingSonar":
                    self.cfg['sensors'][i]['pub'].publish(self.sonar_to_image(val))
                case "ProfilingSonar":
                    self.cfg['sensors'][i]['pub'].publish(self.sonar_to_image(val))

        with self.command_lock:
            self.env.act(self.cfg['agent_name'], self.command)
        

class EnvNode(Node):
    def __init__(self, cfg, agent_callback_group):
        self.cfg = cfg
        super().__init__(f"holoocean_driver")
        
        # create sim environment
        self.env = holoocean.make(scenario_cfg=self.cfg, show_viewport=True, verbose=True)
        # self.env.should_render_viewport(True)
        # self.env.set_render_quality(3)
        # self.env.weather.set_fog_density(1)
        
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

    # make env
    env_node = EnvNode(test_cfg, ReentrantCallbackGroup())

    # add agents to shared executor
    executor = MultiThreadedExecutor()
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
