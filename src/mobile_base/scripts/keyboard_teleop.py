#!/usr/bin/env python3
from pynput import keyboard
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from math import tan, radians

class KeyboardController(Node):
    def __init__(self):
        super().__init__('keyboard_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 30)
        self.velocity = 0.65
        self.angular = 0.0
        self.active_keys = set()
        self.wheelbase_length = 0.55
        self.track_width = 0.62
        self.phi_max_degrees = 30  # Maximum steering angle in degrees

    def on_press(self, key):
        try:
            char = key.char
        except AttributeError:
            char = key.name
        self.active_keys.add(char)
        self.update_motion()

    def on_release(self, key):
        try:
            char = key.char
        except AttributeError:
            char = key.name
        if char in self.active_keys:
            self.active_keys.remove(char)
        self.update_motion()
    def update_motion(self):
        phi_max = radians(self.phi_max_degrees)
        l = self.wheelbase_length
        w = self.track_width
        r = float('inf')
        temp_velocity = 0

        if 'space' in self.active_keys:
            temp_velocity = 0  
        elif 'w' in self.active_keys:
            temp_velocity = self.velocity  # forward
        elif 's' in self.active_keys:
            temp_velocity = -self.velocity  # backward

        if 'a' in self.active_keys:
            r = l / tan(phi_max) + 0.5 * w  # turn left
        elif 'd' in self.active_keys:
            r = -l / tan(phi_max) - 0.5 * w  # turn right

        omega = temp_velocity / r if r != float('inf') else 0

        twist = Twist()
        twist.linear.x = float(temp_velocity)
        twist.angular.z = float(omega)
        self.publisher_.publish(twist)



    def run(self):
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        listener.start()  # start to listen on a separate thread
        rclpy.spin(self)  # keep spinning
        listener.stop()

def main(args=None):
    rclpy.init(args=args)
    keyboard_controller = KeyboardController()
    keyboard_controller.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
