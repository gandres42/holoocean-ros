from threading import Thread
import rclpy
from rclpy.node import Node
import holoocean
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Float32
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Lock
from holoocean_msgs.msg import DVL # type: ignore
import json

# TODO
# launchfiles and config options
# USB gamepad node

class ConfigurationError(Exception):
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

class AgentNode(Node):
    def __init__(self, cfg, num, env, callback_group):
        super().__init__(f"holoocean_agent{num}")
        
        # instance vars
        self.env = env
        self.cfg = cfg
        self.bridge = CvBridge()

        # sensors
        self.topic_root = f"/holoocean/{self.cfg['agent_name']}/"
        self.sensors = {}
        self.sensor_topic_mapping = {
            "RGBCamera": Image,
            "ViewportCapture": Image,
            "ImagingSonar": Image,
            "ProfilingSonar": Image,
            "DVLSensor": DVL,
            "GPSSensor": Point,
            "DepthSensor": Float32
        }
        for sensor_cfg in self.cfg['sensors']:
            self.sensors[sensor_cfg['sensor_name']] = {
                'pub': self.create_publisher(self.sensor_topic_mapping[sensor_cfg['sensor_type']], f"{self.topic_root}{sensor_cfg['sensor_name']}", 1),
                'type': sensor_cfg['sensor_type']
            }
        
        # commands
        print(self.cfg['agent_type'])
        if self.cfg['agent_type'] in ['HoveringAUV', 'BlueROV2']:
            self.command = np.zeros((8))
        elif self.cfg['agent_type'] in ['TorpedoAUV']:
            self.command = np.zeros((5))
        elif self.cfg['agent_type'] in ['SurfaceVessel']:
            self.command = np.zeros((2))
        self.command_lock = Lock()
        self.create_subscription(Twist, f"{self.topic_root}cmd_vel", self.command_callback, 1, callback_group=callback_group)

    def command_callback(self, msg):
        if self.cfg['agent_type'] in ['HoveringAUV', 'BlueROV2']:
            command = np.zeros(8)
            
            # UP/DOWN
            command[0:4] += msg.linear.z * 10.0

            # YAW
            command[[4,7]] -= msg.angular.z * 10.0
            command[[5,6]] += msg.angular.z * 10.0

            # FORWARD/BACKWARD
            command[4:8] += msg.linear.y * 10.0

            # STRAFE LEFT
            command[[4,6]] -= msg.linear.x * 10.0
            command[[5,7]] += msg.linear.x * 10.0
            with self.command_lock:
                self.command = command
        elif self.cfg['agent_type'] in ['TorpedoAUV']:
            command = np.zeros(5)
            # [left_fin, top_fin, right_fin, bottom_fin, thrust]
            
            # FORWARD
            command[4] = msg.linear.y * 100.0

            # YAW
            command[1] = -100.0 * msg.linear.x
            command[3] = 100.0 * msg.linear.x

            # ROLL
            command[0] = 100.0 * msg.linear.z
            command[2] = 100.0 * -msg.linear.z

            with self.command_lock:
                self.command = command
        elif self.cfg['agent_type'] in ['SurfaceVessel']:
            command = np.zeros(2)
            # [left thruster, right thruster]
            command[0] = msg.linear.y * 10000.0
            command[1] = msg.linear.y * 10000.0

            if msg.angular.z != 0:
                command[0] *= msg.angular.z
                command[1] *= -msg.angular.z

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
        # topics
        for (sensor_name, val) in state.items():
            sensor_type = self.sensors[sensor_name]['type']
            msg = None
            match sensor_type:
                case "RGBCamera":
                    msg = self.camera_to_image(val)
                case "ViewportCapture":
                    msg = self.camera_to_image(val)
                case "ImagingSonar":
                    msg = self.sonar_to_image(val)
                case "ProfilingSonar":
                    msg = self.sonar_to_image(val)
                case "DVLSensor":
                    msg = DVL()
                    msg.velocity_x = float(val[0])
                    msg.velocity_y = float(val[1])
                    msg.velocity_z = float(val[2])
                    msg.range_x_forw = float(val[3])
                    msg.range_y_forw = float(val[4])
                    msg.range_x_back = float(val[5])
                    msg.range_y_back = float(val[6])
                case "GPSSensor":
                    msg = Point()
                    msg.x = val[0]
                    msg.y = val[1]
                    msg.z = val[2]
                case "DepthSensor":
                    msg = Float32()
                    msg.data = float(val[0])
            self.sensors[sensor_name]['pub'].publish(msg)
        
        # latest command
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
            self.cfg = json.load(f)
        
        # create sim environment
        self.env = holoocean.make(scenario_cfg=self.cfg, show_viewport=True, verbose=True)
        # self.env.should_render_viewport(True)
        # self.env.set_render_quality(3)
        self.env.weather.set_fog_density(0.5)
        
        # create agent nodes using cfg updated by holoocean
        self.agents = []
        for i, agent_cfg in enumerate(self.cfg['agents']):
            self.agents.append(AgentNode(agent_cfg, i, self.env, agent_callback_group))
        
        # update environment at provided tick rate
        self.create_timer(1 / self.cfg['ticks_per_sec'], self.tick)

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
    env_node = EnvNode(ReentrantCallbackGroup())

    # add agents to shared executor
    # executor = MultiThreadedExecutor(num_threads=len(env_node.agents) + 1)
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
