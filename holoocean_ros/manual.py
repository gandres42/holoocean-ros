import holoocean
import numpy as np
from pynput import keyboard
import time
from threading import Thread
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64


class KeyboardListener:
    def __init__(self):
        self.pressed_keys = list()
        self.force = 25
        listener = keyboard.Listener(on_press=self.__on_press, on_release=self.__on_release)
        listener.start()
        self.command = None
        Thread(target=self.__key_thread, daemon=True).start()

    def __key_thread(self):
        while True:
            if 'q' in self.pressed_keys:
                break
            self.command = self.__parse_keys(self.pressed_keys, self.force)

    def __on_press(self, key):
        if hasattr(key, 'char'):
            self.pressed_keys.append(key.char)
            self.pressed_keys = list(set(self.pressed_keys))

    def __on_release(self, key):
        if hasattr(key, 'char'):
            self.pressed_keys.remove(key.char)

    def __parse_keys(self, keys, val):
        command = {}
        for i, key in enumerate('wasdijkl'):
            if key in keys:
                command[key] = True
            else:
                command[key] = False
        return command
    
    def get_command(self) -> dict:
        return self.command # type: ignore

class ControlNode(Node):
    def __init__(self, agent_num):
        super().__init__('holoocean_controller')
        self.control_timer = self.create_timer(1/60, self.control_timer_callback)
        self.cmdvel_pub = self.create_publisher(Twist, f"/holoocean/agent{agent_num}/cmd_vel", 10)
        self.listener = KeyboardListener()

    def control_timer_callback(self):
        cmd = self.listener.get_command()
        cmd_lin = Vector3()
        cmd_ang = Vector3()
        if cmd['w']:
            cmd_lin.y = 10.0
        if cmd['s']:
            cmd_lin.y = -10.0
        if cmd['a']:
            cmd_lin.x = -10.0
        if cmd['d']:
            cmd_lin.x = 10.0
        if cmd['i']:
            cmd_lin.z = 10.0
        if cmd['j']:
            cmd_ang.z = -10.0
        if cmd['k']:
            cmd_lin.z = -10.0
        if cmd['l']:
            cmd_ang.z = 10.0
        
        cmd_vel = Twist()
        cmd_vel.linear = cmd_lin
        cmd_vel.angular = cmd_ang
        self.cmdvel_pub.publish(cmd_vel)
        


def main(args=None):

    rclpy.init(args=args)
    node = ControlNode(0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()