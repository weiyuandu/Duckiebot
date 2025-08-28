#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, LEDPattern
from std_msgs.msg import ColorRGBA
import time
import numpy as np
from enum import Enum

class SquareState(Enum):
    """States for the square movement state machine"""
    INIT = 0
    SET_CORNER_LED = 1
    MOVING_FORWARD = 2
    TURNING_LEFT = 3
    COMPLETED = 4
    SHUTDOWN = 5

class SquareControllerStateMachineNode(DTROS):
    """
    State machine-based square controller that moves the duckiebot in a 1m x 1m square
    with different LED colors at each corner using RGB LEDPattern messages.
    """
    
    def __init__(self, node_name):
        super(SquareControllerStateMachineNode, self).__init__(
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
        
        # State machine variables
        self._current_state = SquareState.INIT
        self._current_corner = 0
        self._action_start_time = None
        self._state_machine_rate = rospy.Rate(10)  # 10 Hz state machine update rate
        
        # Color names for logging
        self._color_names = ["RED", "GREEN", "BLUE", "WHITE"]
        
        self.log(f"Square controller state machine initialized. Move time: {self._move_time:.2f}s, Turn time: {self._turn_time:.2f}s")
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
            self.log(f"LED pattern published: {color_name} RGB[{r}, {g}, {b}]")
            
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

    def stop_robot(self):
        """Stop the robot"""
        self.publish_car_cmd(0.0, 0.0)

    def start_action(self):
        """Record the start time of current action"""
        self._action_start_time = rospy.Time.now()

    def action_duration_elapsed(self, target_duration):
        """Check if the specified duration has elapsed since action start"""
        if self._action_start_time is None:
            return False
        current_time = rospy.Time.now()
        elapsed = (current_time - self._action_start_time).to_sec()
        return elapsed >= target_duration

    def transition_to_state(self, new_state):
        """Transition to a new state"""
        self.log(f"State transition: {self._current_state.name} -> {new_state.name}")
        self._current_state = new_state
        self._action_start_time = None

    def state_machine_step(self):
        """Execute one step of the state machine"""
        
        if self._current_state == SquareState.INIT:
            self.log("Starting square movement...")
            self.transition_to_state(SquareState.SET_CORNER_LED)
            
        elif self._current_state == SquareState.SET_CORNER_LED:
            # Start timing for LED display if we just entered this state
            if self._action_start_time is None:
                # Set LED color for current corner
                rgb = self._corner_colors[self._current_corner]
                color_name = self._color_names[self._current_corner]
                
                self.set_led_color(rgb[0], rgb[1], rgb[2], color_name)
                self.log(f"Corner {self._current_corner + 1}: LED set to {color_name} [{rgb[0]}, {rgb[1]}, {rgb[2]}]")
                
                # Start timing for LED display
                self.start_action()
            
            # Wait 1 second to see the color change, then move to forward movement
            if self.action_duration_elapsed(1.0):
                self.transition_to_state(SquareState.MOVING_FORWARD)
                
        elif self._current_state == SquareState.MOVING_FORWARD:
            # Start moving forward if we just entered this state
            if self._action_start_time is None:
                self.log(f"Moving forward for {self._move_time:.2f} seconds")
                self.start_action()
            
            # Keep publishing forward movement command
            self.publish_car_cmd(self._linear_velocity, 0.0)
            
            # Check if movement duration is complete
            if self.action_duration_elapsed(self._move_time):
                self.stop_robot()
                self.log("Forward movement completed")
                
                # Check if this was the last edge (4th corner)
                if self._current_corner >= 3:
                    self.transition_to_state(SquareState.COMPLETED)
                else:
                    # Small pause before turning
                    rospy.sleep(0.5)
                    self.transition_to_state(SquareState.TURNING_LEFT)
                    
        elif self._current_state == SquareState.TURNING_LEFT:
            # Start turning if we just entered this state
            if self._action_start_time is None:
                self.log(f"Turning left for {self._turn_time:.2f} seconds")
                self.start_action()
            
            # Keep publishing turn command
            self.publish_car_cmd(0.0, self._angular_velocity)
            
            # Check if turn duration is complete
            if self.action_duration_elapsed(self._turn_time):
                self.stop_robot()
                self.log("Turn completed")
                
                # Move to next corner
                self._current_corner += 1
                
                # Small pause before next edge
                rospy.sleep(0.5)
                self.transition_to_state(SquareState.SET_CORNER_LED)
                
        elif self._current_state == SquareState.COMPLETED:
            # Turn off LEDs and complete the square
            self.turn_off_leds()
            self.log("LEDs turned off")
            self.log("Square movement completed!")
            self.transition_to_state(SquareState.SHUTDOWN)
            
        elif self._current_state == SquareState.SHUTDOWN:
            # Final cleanup
            self.stop_robot()
            self.log("State machine shutting down...")
            return False  # Signal to stop the state machine loop
            
        return True  # Continue state machine execution

    def run(self):
        """Main run loop with state machine"""
        # Wait a bit for everything to initialize
        rospy.sleep(2.0)
        
        # Run the state machine
        while not rospy.is_shutdown():
            # Execute one step of the state machine
            continue_execution = self.state_machine_step()
            
            if not continue_execution:
                break
                
            # Sleep according to state machine rate
            self._state_machine_rate.sleep()
        
        # Keep node alive for a bit after completion
        rospy.sleep(2.0)

    def on_shutdown(self):
        """Cleanup when shutting down"""
        self.stop_robot()
        self.turn_off_leds()
        self.log("LEDs turned off")
        self.log("Square controller state machine shutting down...")

if __name__ == '__main__':
    node = SquareControllerStateMachineNode(node_name='square_controller_state_machine_node')
    
    # Register shutdown hook
    rospy.on_shutdown(node.on_shutdown)
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()
