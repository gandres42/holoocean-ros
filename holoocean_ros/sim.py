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
                {
                    "sensor_type": "ViewportCapture"
                }
            ],
            "control_scheme": 0,
            # "location": [486.0, -632.0, -12.0],
            "rotation": [0.0, 0.0, 135.0]
        },
    ]
}

class AgentNode(Node):
    def __init__(self, agent_cfg, agent_num):
        super().__init__(f"holoocean_agent{agent_num}")
        self.bridge = CvBridge()
        self.topic_root = f"/holoocean/agent{agent_num}/"
        agent_cfg = deepcopy(agent_cfg)

        # create sensor topic names and publishers based
        typecount = {}
        for i, sensor in enumerate(agent_cfg['sensors']):
            if sensor['sensor_type'] not in typecount:
                typecount[sensor['sensor_type']] = 0
            else:
                typecount[sensor['sensor_type']] += 1

            agent_cfg['sensors'][i]['topic'] = f"{sensor['sensor_type']}{typecount[sensor['sensor_type']]}"
            match sensor['sensor_type']:
                case "RGBCamera":
                    agent_cfg['sensors'][i]['pub'] = self.create_publisher(Image, self.topic_root + agent_cfg['sensors'][i]['topic'], 1)
                case "ViewportCapture":
                    agent_cfg['sensors'][i]['pub'] = self.create_publisher(Image, self.topic_root + agent_cfg['sensors'][i]['topic'], 1)

        self.cfg = agent_cfg
        
        # subscribe to control topic
        self.create_subscription(Twist, f"{self.topic_root}cmd_vel", self.vel_callback, 10)
        print(f"{self.topic_root}cmd_vel")
        self.get_logger().info(f'made subscriber at {self.topic_root}cmd_vel')

    def vel_callback(self, msg):
        command = np.zeros(8)
        # if 'i' in keys:
        #     command[0:4] += val
        # if 'k' in keys:
        #     command[0:4] -= val
        # if 'j' in keys:
        #     command[[4,7]] += val
        #     command[[5,6]] -= val
        # if 'l' in keys:
        #     command[[4,7]] -= val
        #     command[[5,6]] += val

        # if 'w' in keys:
        #     command[4:8] += val
        # if 's' in keys:
        #     command[4:8] -= val
        # if 'a' in keys:
        #     command[[4,6]] += val
        #     command[[5,7]] -= val
        # if 'd' in keys:
        #     command[[4,6]] -= val
        #     command[[5,7]] += val
    
    def numpy_to_image(self, np_img: np.ndarray, frame_id: str = "camera") -> Image:
        msg = self.bridge.cv2_to_imgmsg(np_img)
        msg.header = Header()
        msg.header.stamp = rclpy.time.Time().to_msg() # type: ignore
        msg.header.frame_id = frame_id
        return msg

    def tick(self, state):
        for i, (sensor, val) in enumerate(state.items()):
            match sensor:
                case "RGBCamera":
                    self.cfg['sensors'][i]['pub'].publish(self.numpy_to_image(val))
                case "ViewportCapture":
                    self.cfg['sensors'][i]['pub'].publish(self.numpy_to_image(val))


class EnvNode(Node):
    def __init__(self, global_cfg):
        self.cfg = global_cfg
        super().__init__(f"holoocean_driver")
        self.agents = []
        for i, agent_cfg in enumerate(global_cfg['agents']):
            self.agents.append(AgentNode(agent_cfg, i))
        self.env = holoocean.make(scenario_cfg=self.cfg, show_viewport=True, verbose=False)
        self.create_timer(1 / global_cfg['ticks_per_sec'], self.tick)
        self.env.should_render_viewport(True)
        # self.env.set_render_quality(3)
        # self.env.weather.set_fog_density(1)

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
        for i in range(len(self.agents)):
            self.agents[i].tick(state[self.cfg['agents'][i]['agent_name']])

        # update agent movements
        # command = parse_keys(pressed_keys, force)
        # env.act("auv0", command)
                

def main(args=None):
    rclpy.init(args=args)
    node = EnvNode(test_cfg)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0)
            for agent in node.agents:
                rclpy.spin_once(agent, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    # rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
