#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA
import time
import numpy as np

class SquareControllerNode(DTROS):
    def __init__(self, node_name):
        super(SquareControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        self.veh_name = os.environ.get('VEHICLE_NAME', 'duck')
        
        car_cmd_topic = f"/{self.veh_name}/car_cmd_switch_node/cmd"
        self._car_cmd_publisher = rospy.Publisher(
            car_cmd_topic, 
            Twist2DStamped, 
            queue_size=1
        )
        
        #referced Duckietown: https://github.com/duckietown/dt-gui-tools/tree/daffy/packages/led_widget
        led_topic = f"/{self.veh_name}/led_emitter_node/led_pattern"
        self._led_publisher = rospy.Publisher(
            led_topic,
            LEDPattern,
            queue_size=1
        )
        self._linear_velocity = 0.2   # m/s - good speed for real bot
        self._angular_velocity = 2.0  # rad/s - good turning speed
        self._edge_distance = 1.0     # meters
        
        self._move_time = self._edge_distance / self._linear_velocity  # time to move 1 meter
        self._turn_time = (np.pi / 2) / self._angular_velocity         # time for 90-degree turn
        
        self._corner_colors = [
            [1.0, 0.0, 0.0],  # Red
            [0.0, 1.0, 0.0],  # Green  
            [0.0, 0.0, 1.0],  # Blue
            [1.0, 1.0, 1.0]   # White
        ]
        
        self._intensity = 1.0
        
        self.log(f"Square controller initialized. Move time: {self._move_time:.2f}s, Turn time: {self._turn_time:.2f}s")
        self.log(f"Vehicle name: {self.veh_name}")
        # self.log(f"Publishing movement commands to: {car_cmd_topic}")
        # self.log(f"Publishing LED patterns to: {led_topic}")

    def set_led_color(self, r, g, b, color_name):
        pattern = LEDPattern()
        
        for _ in range(5):
            led_color = ColorRGBA()
            led_color.r = r
            led_color.g = g
            led_color.b = b
            led_color.a = self._intensity
            pattern.rgb_vals.append(led_color)
        
        self._led_publisher.publish(pattern)
        self.log(f"LED pattern published: RGB[{r}, {g}, {b}]")

    def turn_off_leds(self):
        self.set_led_color(0.0, 0.0, 0.0, "OFF")

    def publish_car_cmd(self, linear_v, angular_v):
        """Publish a car command (adapted from EncoderPoseNode)"""
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = linear_v      # linear velocity
        car_control_msg.omega = angular_v # angular velocity
        self._car_cmd_publisher.publish(car_control_msg)
        
        # # Debug output to confirm commands are being sent
        # if linear_v != 0 or angular_v != 0:
        #     self.log(f"Sending cmd: v={linear_v:.2f}, ω={angular_v:.2f}")

    def stop_robot(self):
        self.publish_car_cmd(0.0, 0.0)

    def move_forward(self, duration):
        """Move forward for specified duration"""
        # self.log(f"Moving forward for {duration:.2f} seconds")
        rate = rospy.Rate(10)  # 10 Hz
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.publish_car_cmd(self._linear_velocity, 0.0)
            rate.sleep()
        
        self.stop_robot()

    def turn_left(self, duration):
        # self.log(f"Turning left for {duration:.2f} seconds")
        rate = rospy.Rate(10)  # 10 Hz
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.publish_car_cmd(0.0, self._angular_velocity)
            rate.sleep()
        
        self.stop_robot()

    def execute_square(self):
        self.log("Starting square movement...")
        
        for corner in range(4):
            if rospy.is_shutdown():
                break
            rgb = self._corner_colors[corner]
            color_names = ["RED", "GREEN", "BLUE", "WHITE"]
            color_name = color_names[corner]
            
            self.set_led_color(rgb[0], rgb[1], rgb[2], color_name)
            self.log(f"Corner {corner + 1}: LED set to {color_name} [{rgb[0]}, {rgb[1]}, {rgb[2]}]")
            
            rospy.sleep(1.0)
            self.move_forward(self._move_time)
            rospy.sleep(0.5)

            if corner < 3:  # Don't turn after the 4th edge
                self.turn_left(self._turn_time)
                rospy.sleep(0.5)
        
        self.turn_off_leds()
        self.log("LEDs turned off")
        self.log("Square movement completed!")

    def run(self):
        # Wait 2s for everything to initialize
        rospy.sleep(2.0)
    
        self.execute_square()

        rospy.sleep(2.0)

    def on_shutdown(self):
        self.stop_robot()
        self.turn_off_leds()
        self.log(" LEDs turned off")
        self.log("Square controller shutting down...")

if __name__ == '__main__':
    node = SquareControllerNode(node_name='square_controller_node')
    
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()