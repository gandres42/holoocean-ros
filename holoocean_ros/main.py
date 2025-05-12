import holoocean.sensors
import rclpy
from rclpy.node import Node
import holoocean
import numpy as np
from pynput import keyboard
import json
import cv2
import time

cfg = {
    "name": "test_rgb_camera",
    "world": "PierHarbor",
    "package_name": "Ocean",
    "main_agent": "auv0",
    "ticks_per_sec": 60,
    "agents": [
        {
            "agent_name": "auv0",
            "agent_type": "BlueROV2",
            "sensors": [
                {
                    "sensor_type": "RGBCamera",
                    "socket": "CameraSocket",
                    "configuration": {
                        "CaptureWidth": 512,
                        "CaptureHeight": 512
                    }
                },
                {
                    "sensor_type": "ViewportCapture",
                }
            ],
            "control_scheme": 0,
            "location": [486.0, -632.0, -12.0],
            "rotation": [0.0, 0.0, 135.0]
        }
    ]
}

pressed_keys = list()
force = 25

def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))

def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

def parse_keys(keys, val):
    command = np.zeros(8)
    if 'i' in keys:
        command[0:4] += val
    if 'k' in keys:
        command[0:4] -= val
    if 'j' in keys:
        command[[4,7]] += val
        command[[5,6]] -= val
    if 'l' in keys:
        command[[4,7]] -= val
        command[[5,6]] += val

    if 'w' in keys:
        command[4:8] += val
    if 's' in keys:
        command[4:8] -= val
    if 'a' in keys:
        command[[4,6]] += val
        command[[5,7]] -= val
    if 'd' in keys:
        command[[4,6]] -= val
        command[[5,7]] += val

    return command

def main():
    with holoocean.make(scenario_cfg=cfg, show_viewport=True, verbose=True) as env:
        env.should_render_viewport(True)
        while True:
            if 'q' in pressed_keys:
                break
            command = parse_keys(pressed_keys, force)

            #send to holoocean
            env.act("auv0", command)
            state = env.tick()
            cv2.imshow("viewport", state['RGBCamera'])
            # cv2.imshow("viewport", state['ViewportCapture'])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

# class MinimalNode(Node):
#     def __init__(self):
#         super().__init__('minimal_node')
#         self.get_logger().info('Node started')

# def main(args=None):
#     rclpy.init(args=args)
#     node = MinimalNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

if __name__ == '__main__':
    main()
