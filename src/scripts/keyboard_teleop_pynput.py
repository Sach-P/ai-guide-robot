#!/usr/bin/env python3
"""Keyboard teleoperation for RC car using pynput for continuous key detection."""
import rclpy
import time
import threading

from geometry_msgs.msg import Twist
from pynput import keyboard


class KeyboardTeleop:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('keyboard_teleop')
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Control parameters
        self.speed = 0.0
        self.steer = 0.0
        self.max_speed = 3.0
        self.max_steer = 1.0

        # The pynput listener updates this set from a background thread.
        self.keys_pressed = set()
        self._keys_lock = threading.Lock()
        self._quit_requested = threading.Event()

        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        """Called when a key is pressed."""
        try:
            char = key.char
        except AttributeError:
            char = None

        if char is not None:
            char = char.lower()
            with self._keys_lock:
                self.keys_pressed.add(char)
            if char == 'q':
                self._quit_requested.set()
                return False
        else:
            with self._keys_lock:
                self.keys_pressed.add(key)

    def on_release(self, key):
        """Called when a key is released."""
        try:
            char = key.char
        except AttributeError:
            char = None

        with self._keys_lock:
            self.keys_pressed.discard(char.lower() if char is not None else key)

    def update_controls(self):
        """Update speed and steering based on currently held keys."""
        with self._keys_lock:
            keys_pressed = self.keys_pressed.copy()

        self.speed = 0.0
        self.steer = 0.0

        if 'w' in keys_pressed or keyboard.Key.up in keys_pressed:
            self.speed = self.max_speed
        elif 's' in keys_pressed or keyboard.Key.down in keys_pressed:
            self.speed = -self.max_speed

        if 'a' in keys_pressed or keyboard.Key.left in keys_pressed:
            self.steer = self.max_steer
        elif 'd' in keys_pressed or keyboard.Key.right in keys_pressed:
            self.steer = -self.max_steer

        if ' ' in keys_pressed or keyboard.Key.space in keys_pressed:
            self.speed = 0.0
            self.steer = 0.0

    def publish_command(self):
        """Publish the current command."""
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.steer
        self.pub.publish(msg)

    def print_status(self):
        """Print current status."""
        print(
            f"\rSpeed: {self.speed:5.2f} m/s | Turn: {self.steer:5.2f} rad/s",
            end='',
            flush=True,
        )

    def run(self):
        """Main teleop loop."""
        print("╔════════════════════════════════════════╗")
        print("║   RC Car Keyboard Teleoperation (Py)   ║")
        print("╠════════════════════════════════════════╣")
        print("║  W / ↑     : forward                   ║")
        print("║  S / ↓     : reverse                   ║")
        print("║  A / ←     : turn left                 ║")
        print("║  D / →     : turn right                ║")
        print("║  SPACE     : emergency stop            ║")
        print("║  Q         : quit                      ║")
        print("╚════════════════════════════════════════╝")
        print("Speed:  0.00 m/s | Turn:  0.00 rad/s")

        try:
            while rclpy.ok() and not self._quit_requested.is_set():
                self.update_controls()
                self.publish_command()
                self.print_status()

                time.sleep(0.01)

        except KeyboardInterrupt:
            pass
        finally:
            self.listener.stop()
            self.speed = 0.0
            self.steer = 0.0
            if rclpy.ok():
                self.publish_command()
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            print("\nTeleop stopped.")


def main():
    try:
        teleop = KeyboardTeleop()
        teleop.run()
    except ImportError:
        print("\nError: pynput not installed!")
        print("Install it with: pip3 install pynput")
        return 1


if __name__ == '__main__':
    main()
