#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, LEDPattern, WheelEncoderStamped, Pose2DStamped
from std_msgs.msg import ColorRGBA
from typing import Optional
import time
import numpy as np
from enum import Enum
import odometry_activity

class SquareState(Enum):
    """States for the square movement state machine"""
    INIT = 0
    WAIT_FOR_ENCODERS = 1
    SET_CORNER_LED = 2
    MOVING_FORWARD = 3
    TURNING_LEFT = 4
    COMPLETED = 5
    SHUTDOWN = 6

class SquareControllerStateMachineNode(DTROS):
    """
    State machine-based square controller that moves the duckiebot in a 1m x 1m square
    with different LED colors at each corner using RGB LEDPattern messages.
    Uses wheel encoder feedback for precise positioning based on differential drive kinematics.
    """
    
    # Type hints for encoder data
    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float
    
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
        
        # Load kinematic parameters like in the encoder_pose_node
        self.log("Loading kinematics calibration...")
        self.R = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius',
            0.0318,  # meters, default value of wheel radius
        )
        self.baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline', 
            0.1  # meters, default value of baseline
        )
        
        # Initialize odometry variables
        self.reset_odometry()
        
        # Publisher for movement commands - using the car command switch node topic
        car_cmd_topic = f"/{self.veh_name}/car_cmd_switch_node/cmd"
        self._car_cmd_publisher = rospy.Publisher(
            car_cmd_topic, 
            Twist2DStamped, 
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )
        
        # Publisher for LED control - using RGB pattern approach like the widget
        led_topic = f"/{self.veh_name}/led_emitter_node/led_pattern"
        self._led_publisher = rospy.Publisher(
            led_topic,
            LEDPattern,
            queue_size=1
        )
        
        # Encoder subscribers (like in encoder_pose_node)
        left_encoder_topic = f"/{self.veh_name}/left_wheel_encoder_node/tick"
        rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped,
            self.cb_left_encoder,
            queue_size=1,
        )

        right_encoder_topic = f"/{self.veh_name}/right_wheel_encoder_node/tick"
        rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped,
            self.cb_right_encoder,
            queue_size=1,
        )
        
        # Control parameters (calibrated for real Duckiebot)
        self._linear_velocity = 0.4   # m/s - good speed for real bot
        self._angular_velocity = 2.0  # rad/s - good turning speed
        self._edge_distance = 1.0     # meters - side length of square
        
        # Control gains for position and orientation control
        self.speed_gain = 0.4
        self.steer_gain = 5.5
        
        # Thresholds for position and angle control
        self.position_threshold = 0.05  # meters
        self.angle_threshold = np.deg2rad(5)  # 5 degrees
        
        # For encoders synchronization:
        self.RIGHT_RECEIVED = False
        self.LEFT_RECEIVED = False
        
        # Movement state tracking
        self.duckiebot_is_moving = False
        
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
        
        # Target positions for each corner of the square
        self._corner_positions = [
            (0.0, 0.0),     # Start position
            (1.0, 0.0),     # First corner - move forward 1m
            (1.0, 1.0),     # Second corner - turn left and move forward 1m
            (0.0, 1.0),     # Third corner - turn left and move forward 1m
            (0.0, 0.0)      # Back to start - turn left and move forward 1m
        ]
        
        # Target orientations for each corner (in radians)
        self._corner_orientations = [
            0.0,            # Start facing right (0°)
            np.pi/2,        # After first edge, face up (90°)
            np.pi,          # After second edge, face left (180°)
            3*np.pi/2,      # After third edge, face down (270°)
            0.0             # After fourth edge, face right again (0°)
        ]
        
        # Color names for logging
        self._color_names = ["RED", "GREEN", "BLUE", "WHITE"]
        
        self.log(f"Square controller state machine initialized.")
        self.log(f"Wheel radius: {self.R:.4f} m, Baseline: {self.baseline:.4f} m")
        self.log(f"Vehicle name: {self.veh_name}")
        self.log(f"Publishing movement commands to: {car_cmd_topic}")
        self.log(f"Publishing LED patterns to: {led_topic}")
        self.log(f"Subscribing to encoders: {left_encoder_topic}, {right_encoder_topic}")

    def reset_odometry(self):
        """Reset odometry parameters like in encoder_pose_node"""
        # Change in left/right wheel (radians) since previous reading
        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

        # Last number of wheel encoder ticks read
        self.left_tick_prev = None
        self.right_tick_prev = None

        # Initialize the odometry - start at origin facing right
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0  # Start facing right (0 degrees)

        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0
        
        # Total distance traveled
        self.total_distance = 0.0

    def cb_left_encoder(self, encoder_msg):
        """Left wheel encoder callback (adapted from encoder_pose_node)"""
        # If we have not yet read ticks for the wheel, read them now
        if self.left_tick_prev is None:
            ticks = encoder_msg.data
            self.left_tick_prev = ticks
            return

        # Calculate the rotation of the left wheel
        delta_phi_left, self.left_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.left_tick_prev,
        )
        self.delta_phi_left += delta_phi_left

        # Compute the new pose
        self.LEFT_RECEIVED = True
        self.calculate_pose()

    def cb_right_encoder(self, encoder_msg):
        """Right wheel encoder callback (adapted from encoder_pose_node)"""
        # If we have not yet read ticks for the wheel, read them now
        if self.right_tick_prev is None:
            ticks = encoder_msg.data
            self.right_tick_prev = ticks
            return

        # Calculate the rotation of the right wheel
        delta_phi_right, self.right_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.right_tick_prev,
        )
        self.delta_phi_right += delta_phi_right

        # Compute the new pose
        self.RIGHT_RECEIVED = True
        self.calculate_pose()

    def calculate_pose(self):
        """Calculate pose using encoder data (adapted from encoder_pose_node)"""
        if not (self.LEFT_RECEIVED and self.RIGHT_RECEIVED):
            return

        # Sync incoming messages from encoders
        self.LEFT_RECEIVED = self.RIGHT_RECEIVED = False

        # Estimate new pose
        self.x_curr, self.y_curr, theta_curr, dA = \
            odometry_activity.estimate_pose(
                self.R,
                self.baseline,
                self.x_prev,
                self.y_prev,
                self.theta_prev,
                self.delta_phi_left,
                self.delta_phi_right,
            )

        self.total_distance += np.abs(dA)

        # Clamp the angle to stay in [0, 2π)
        self.theta_curr = self.angle_clamp(theta_curr)

        # Check if robot is moving
        self.duckiebot_is_moving = (
            abs(self.delta_phi_left) > 0 or
            abs(self.delta_phi_right) > 0
        )

        # Reset the change in wheels
        self.delta_phi_left = self.delta_phi_right = 0

        # Update previous pose
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr

    def angle_clamp(self, theta):
        """Clamp angle to [0, 2π) (from encoder_pose_node)"""
        if theta >= 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < 0:
            return theta + 2 * np.pi
        else:
            return theta

    def angle_diff(self, target_angle, current_angle):
        """Calculate the shortest angular difference between two angles"""
        diff = target_angle - current_angle
        # Normalize to [-π, π]
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

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

    def calculate_position_error(self, target_x, target_y):
        """Calculate distance to target position"""
        dx = target_x - self.x_curr
        dy = target_y - self.y_curr
        return np.sqrt(dx*dx + dy*dy)

    def calculate_position_control(self, target_x, target_y):
        """Calculate control commands to reach target position (like encoder_pose_node)"""
        # Calculate position error
        dx = target_x - self.x_curr
        dy = target_y - self.y_curr
        distance_error = np.sqrt(dx*dx + dy*dy)
        
        # Calculate desired heading to target
        desired_theta = np.arctan2(dy, dx)
        
        # Calculate angular error
        angle_error = self.angle_diff(desired_theta, self.theta_curr)
        
        # Control gains (like in encoder_pose_node)
        v = self.speed_gain if distance_error > self.position_threshold else 0.0
        omega = self.steer_gain * angle_error
        
        # Limit angular velocity to prevent instability
        max_omega = 2.0
        omega = np.clip(omega, -max_omega, max_omega)
        
        return v, omega, distance_error, angle_error

    def calculate_orientation_control(self, target_theta):
        """Calculate control commands to reach target orientation"""
        angle_error = self.angle_diff(target_theta, self.theta_curr)
        
        # Pure rotation control
        v = 0.0
        omega = self.steer_gain * angle_error
        
        # Limit angular velocity
        max_omega = 2.0
        omega = np.clip(omega, -max_omega, max_omega)
        
        return v, omega, angle_error

    def state_machine_step(self):
        """Execute one step of the state machine with encoder feedback control"""
        
        if self._current_state == SquareState.INIT:
            self.log("Starting square movement with encoder feedback...")
            self.log(f"Initial position: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")
            self.transition_to_state(SquareState.WAIT_FOR_ENCODERS)
            
        elif self._current_state == SquareState.WAIT_FOR_ENCODERS:
            # Wait until we have encoder data before starting
            if self.left_tick_prev is not None and self.right_tick_prev is not None:
                self.log("Encoder data received, starting square movement")
                self.transition_to_state(SquareState.SET_CORNER_LED)
            else:
                self.log("Waiting for encoder data...")
                rospy.sleep(0.1)
                
        elif self._current_state == SquareState.SET_CORNER_LED:
            # Set LED color for current corner
            rgb = self._corner_colors[self._current_corner]
            color_name = self._color_names[self._current_corner]
            
            self.set_led_color(rgb[0], rgb[1], rgb[2], color_name)
            self.log(f"Corner {self._current_corner + 1}: LED set to {color_name}")
            self.log(f"Current pose: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")
            
            # Transition to movement
            self.transition_to_state(SquareState.MOVING_FORWARD)
                
        elif self._current_state == SquareState.MOVING_FORWARD:
            # Get target position for this edge
            target_x, target_y = self._corner_positions[self._current_corner + 1]
            
            # Calculate control commands using position feedback
            v, omega, distance_error, angle_error = self.calculate_position_control(target_x, target_y)
            
            # Log current state
            if self._action_start_time is None:
                self.log(f"Moving to position ({target_x:.2f}, {target_y:.2f})")
                self.start_action()
            
            # Publish control commands
            self.publish_car_cmd(v, omega)
            
            # Log every 2 seconds for debugging
            if self.action_duration_elapsed(2.0):
                self.log(f"Progress: pos=({self.x_curr:.3f}, {self.y_curr:.3f}), "
                        f"θ={np.rad2deg(self.theta_curr):.1f}°, "
                        f"dist_err={distance_error:.3f}m, ang_err={np.rad2deg(angle_error):.1f}°")
                self.start_action()  # Reset timer for next log
            
            # Check if we reached the target position
            if distance_error < self.position_threshold:
                self.stop_robot()
                self.log(f"Reached position ({target_x:.2f}, {target_y:.2f})")
                
                # Check if this was the last edge (back to start)
                if self._current_corner >= 3:
                    self.transition_to_state(SquareState.COMPLETED)
                else:
                    # Move to turning phase
                    rospy.sleep(0.5)  # Brief pause
                    self.transition_to_state(SquareState.TURNING_LEFT)
                    
        elif self._current_state == SquareState.TURNING_LEFT:
            # Get target orientation for next edge
            target_theta = self._corner_orientations[self._current_corner + 1]
            
            # Calculate control commands using orientation feedback
            v, omega, angle_error = self.calculate_orientation_control(target_theta)
            
            # Log start of turn
            if self._action_start_time is None:
                self.log(f"Turning to orientation {np.rad2deg(target_theta):.1f}°")
                self.start_action()
            
            # Publish control commands
            self.publish_car_cmd(v, omega)
            
            # Log every 1 second for debugging
            if self.action_duration_elapsed(1.0):
                self.log(f"Turn progress: current_θ={np.rad2deg(self.theta_curr):.1f}°, "
                        f"target_θ={np.rad2deg(target_theta):.1f}°, "
                        f"ang_err={np.rad2deg(angle_error):.1f}°")
                self.start_action()  # Reset timer for next log
            
            # Check if we reached the target orientation
            if abs(angle_error) < self.angle_threshold:
                self.stop_robot()
                self.log(f"Reached orientation {np.rad2deg(target_theta):.1f}°")
                
                # Move to next corner
                self._current_corner += 1
                
                # Brief pause before next edge
                rospy.sleep(0.5)
                self.transition_to_state(SquareState.SET_CORNER_LED)
                
        elif self._current_state == SquareState.COMPLETED:
            # Turn off LEDs and complete the square
            self.turn_off_leds()
            self.log("LEDs turned off")
            self.log(f"Square movement completed!")
            self.log(f"Final position: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")
            self.log(f"Total distance traveled: {self.total_distance:.3f} m")
            self.transition_to_state(SquareState.SHUTDOWN)
            
        elif self._current_state == SquareState.SHUTDOWN:
            # Final cleanup
            self.stop_robot()
            self.log("State machine shutting down...")
            return False  # Signal to stop the state machine loop
            
        return True  # Continue state machine execution

    def run(self):
        """Main run loop with state machine and encoder feedback"""
        # Wait a bit for everything to initialize
        self.log("Waiting for system initialization...")
        rospy.sleep(2.0)
        
        # Run the state machine
        while not rospy.is_shutdown():
            # Only run state machine if we have encoder data or we're still waiting for it
            if (self.left_tick_prev is not None and self.right_tick_prev is not None) or \
               self._current_state in [SquareState.INIT, SquareState.WAIT_FOR_ENCODERS]:
                
                # Execute one step of the state machine
                continue_execution = self.state_machine_step()
                
                if not continue_execution:
                    break
            else:
                # Still waiting for encoder initialization
                self.log("Waiting for encoder initialization...")
                rospy.sleep(0.5)
                
            # Sleep according to state machine rate
            self._state_machine_rate.sleep()
        
        # Keep node alive for a bit after completion
        self.log("Square controller state machine completed. Keeping node alive for 2 seconds...")
        rospy.sleep(2.0)

    def on_shutdown(self):
        """Cleanup when shutting down"""
        self.stop_robot()
        self.turn_off_leds()
        self.log("LEDs turned off")
        self.log("Square controller state machine shutting down...")
        self.log(f"Final odometry: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")
        self.log(f"Total distance traveled: {self.total_distance:.3f} m")


if __name__ == '__main__':
    node = SquareControllerStateMachineNode(node_name='square_controller_state_machine_node')
    
    # Register shutdown hook
    rospy.on_shutdown(node.on_shutdown)
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        node.logerr(f"Unexpected error: {str(e)}")
    
    # Final cleanup
    node.stop_robot()
    node.turn_off_leds()
