#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA
import time
import numpy as np

class SquareControllerNode(DTROS):
    """
    Open-loop square controller that moves the duckiebot in a 1m x 1m square
    with different LED colors at each corner using RGB LEDPattern messages.
    """
    
    def __init__(self, node_name):
        super(SquareControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        
        # Get vehicle name from namespace (like in the reference code)
        self.veh_name = rospy.get_namespace().strip("/")
        
        # If no vehicle name from namespace, use environment variable as fallback
        if not self.veh_name:
            self.veh_name = os.environ.get('VEHICLE_NAME', 'duck')
        
        # Publisher for movement commands - using the car command switch node topic
        car_cmd_topic = f"/{self.veh_name}/car_cmd_switch_node/cmd"
        self._car_cmd_publisher = rospy.Publisher(
            car_cmd_topic, 
            Twist2DStamped, 
            queue_size=1
        )
        
        # Publisher for LED control - using RGB pattern approach like the widget
        led_topic = f"/{self.veh_name}/led_emitter_node/led_pattern"
        self._led_publisher = rospy.Publisher(
            led_topic,
            LEDPattern,
            queue_size=1
        )
        
        # Movement parameters (calibrated for real Duckiebot)
        self._linear_velocity = 0.4   # m/s - good speed for real bot
        self._angular_velocity = 2.0  # rad/s - good turning speed
        self._edge_distance = 1.0     # meters
        
        # Calculate timing for 1-meter movement and 90-degree turn
        self._move_time = self._edge_distance / self._linear_velocity  # time to move 1 meter
        self._turn_time = (np.pi / 2) / self._angular_velocity         # time for 90-degree turn
        
        # Colors for each corner of the square (RGB values like in the widget)
        self._corner_colors = [
            [1.0, 0.0, 0.0],  # Red
            [0.0, 1.0, 0.0],  # Green  
            [0.0, 0.0, 1.0],  # Blue
            [1.0, 1.0, 1.0]   # White
        ]
        
        # LED intensity
        self._intensity = 1.0
        
        self.log(f"Square controller initialized. Move time: {self._move_time:.2f}s, Turn time: {self._turn_time:.2f}s")
        self.log(f"Vehicle name: {self.veh_name}")
        self.log(f"Publishing movement commands to: {car_cmd_topic}")
        self.log(f"Publishing LED patterns to: {led_topic}")

    def set_led_color(self, r, g, b, color_name):
        """Set LED color using RGB values like the widget"""
        try:
            # Create LED pattern message
            pattern = LEDPattern()
            
            # Set all 5 LEDs to the same color (like in the widget)
            for i in range(5):
                led_color = ColorRGBA()
                led_color.r = r
                led_color.g = g
                led_color.b = b
                led_color.a = self._intensity
                pattern.rgb_vals.append(led_color)
            
            # Publish the pattern
            self._led_publisher.publish(pattern)
            self.log(f"🎨 LED pattern published: RGB[{r}, {g}, {b}]")
            
        except Exception as e:
            self.logwarn(f"Failed to publish LED pattern: {str(e)}")

    def turn_off_leds(self):
        """Turn off all LEDs"""
        self.set_led_color(0.0, 0.0, 0.0, "OFF")

    def publish_car_cmd(self, linear_v, angular_v):
        """Publish a car command (adapted from EncoderPoseNode)"""
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = linear_v      # linear velocity
        car_control_msg.omega = angular_v # angular velocity
        self._car_cmd_publisher.publish(car_control_msg)
        
        # Debug output to confirm commands are being sent
        if linear_v != 0 or angular_v != 0:
            self.log(f"🚗 Sending cmd: v={linear_v:.2f}, ω={angular_v:.2f}")

    def stop_robot(self):
        """Stop the robot"""
        self.publish_car_cmd(0.0, 0.0)

    def move_forward(self, duration):
        """Move forward for specified duration"""
        self.log(f"Moving forward for {duration:.2f} seconds")
        rate = rospy.Rate(10)  # 10 Hz
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.publish_car_cmd(self._linear_velocity, 0.0)
            rate.sleep()
        
        self.stop_robot()

    def turn_left(self, duration):
        """Turn left (counterclockwise) for specified duration"""
        self.log(f"Turning left for {duration:.2f} seconds")
        rate = rospy.Rate(10)  # 10 Hz
        end_time = rospy.Time.now() + rospy.Duration(duration)
        
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self.publish_car_cmd(0.0, self._angular_velocity)
            rate.sleep()
        
        self.stop_robot()

    def execute_square(self):
        """Execute one complete square movement"""
        self.log("Starting square movement...")
        
        for corner in range(4):
            if rospy.is_shutdown():
                break
                
            # Set LED color for this corner
            rgb = self._corner_colors[corner]
            color_names = ["RED", "GREEN", "BLUE", "WHITE"]
            color_name = color_names[corner]
            
            self.set_led_color(rgb[0], rgb[1], rgb[2], color_name)
            self.log(f"Corner {corner + 1}: LED set to {color_name} [{rgb[0]}, {rgb[1]}, {rgb[2]}]")
            
            # Small pause to see the color change
            rospy.sleep(1.0)
            
            # Move forward along the edge
            self.move_forward(self._move_time)
            
            # Small pause between movement and turn
            rospy.sleep(0.5)
            
            # Turn left 90 degrees (except after the last edge)
            if corner < 3:  # Don't turn after the 4th edge
                self.turn_left(self._turn_time)
                rospy.sleep(0.5)
        
        # Turn off LEDs when done
        self.turn_off_leds()
        self.log("LEDs turned off")
        self.log("Square movement completed!")

    def run(self):
        """Main run loop"""
        # Wait a bit for everything to initialize
        rospy.sleep(2.0)
        
        # Execute the square movement
        self.execute_square()
        
        # Keep node alive for a bit
        rospy.sleep(2.0)

    def on_shutdown(self):
        """Cleanup when shutting down"""
        self.stop_robot()
        self.turn_off_leds()
        self.log("LEDs turned off")
        self.log("Square controller shutting down...")

if __name__ == '__main__':
    node = SquareControllerNode(node_name='square_controller_node')
    
    # Register shutdown hook
    rospy.on_shutdown(node.on_shutdown)
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
