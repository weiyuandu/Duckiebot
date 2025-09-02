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

    INIT = 0
    WAIT_FOR_ENCODERS = 1
    SET_CORNER_LED = 2
    MOVING_FORWARD = 3
    TURNING_LEFT = 4
    COMPLETED = 5
    SHUTDOWN = 6

class SquareControllerStateMachineNode(DTROS):

    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int]
    delta_phi_left: float
    delta_phi_right: float
    
    def __init__(self, node_name):
        super(SquareControllerStateMachineNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        
        self.veh_name = rospy.get_namespace().strip("/")

        if not self.veh_name:
            self.veh_name = os.environ.get('VEHICLE_NAME', 'duck')
        
        self.log("Loading kinematics calibration...")
        self.R = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius',
            0.0318,  # meters, default value of wheel radius
        )
        self.baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline', 
            0.1  # meters, default value of baseline
        )
        
        self.reset_odometry()
        
        car_cmd_topic = f"/{self.veh_name}/car_cmd_switch_node/cmd"
        self._car_cmd_publisher = rospy.Publisher(
            car_cmd_topic, 
            Twist2DStamped, 
            queue_size=1,
            dt_topic_type=TopicType.CONTROL
        )
        
        led_topic = f"/{self.veh_name}/led_emitter_node/led_pattern"
        self._led_publisher = rospy.Publisher(
            led_topic,
            LEDPattern,
            queue_size=1
        )
        
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
        
        self._linear_velocity = 0.4   # m/s - good speed for real bot
        self._angular_velocity = 2.0  # rad/s - good turning speed
        self._edge_distance = 1.0     # meters - side length of square
        
        self.speed_gain = 0.4
        self.steer_gain = 5.5
        
        self.position_threshold = 0.05  # meters
        self.angle_threshold = np.deg2rad(5)  # 5 degrees
        
        self.RIGHT_RECEIVED = False
        self.LEFT_RECEIVED = False
        

        self.duckiebot_is_moving = False
        
        self._corner_colors = [
            [1.0, 0.0, 0.0],  # Red
            [0.0, 1.0, 0.0],  # Green  
            [0.0, 0.0, 1.0],  # Blue
            [1.0, 1.0, 1.0]   # White
        ]
        

        self._intensity = 1.0
        

        self._current_state = SquareState.INIT
        self._current_corner = 0
        self._action_start_time = None
        self._state_machine_rate = rospy.Rate(10)  # 10 Hz state machine update rate
        
        self._corner_positions = [
            (0.0, 0.0),  
            (1.0, 0.0),    
            (1.0, 1.0),    
            (0.0, 1.0),    
            (0.0, 0.0)      
        ]
        
        # Target orientations for each corner (in radians)
        self._corner_orientations = [
            0.0,            # Start facing right (0°)
            np.pi/2,        # After first edge, face up (90°)
            np.pi,          # After second edge, face left (180°)
            3*np.pi/2,      # After third edge, face down (270°)
            0.0             # After fourth edge, face right again (0°)
        ]
        
        self._color_names = ["RED", "GREEN", "BLUE", "WHITE"]
        
        self.log(f"Square controller state machine initialized.")
        self.log(f"Wheel radius: {self.R:.4f} m, Baseline: {self.baseline:.4f} m")
        self.log(f"Vehicle name: {self.veh_name}")
        self.log(f"Publishing movement commands to: {car_cmd_topic}")
        self.log(f"Publishing LED patterns to: {led_topic}")
        self.log(f"Subscribing to encoders: {left_encoder_topic}, {right_encoder_topic}")

    def reset_odometry(self):
        """Reset odometry parameters like in encoder_pose_node"""
        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0


        self.left_tick_prev = None
        self.right_tick_prev = None


        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0 

        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0
        

        self.total_distance = 0.0

    def cb_left_encoder(self, encoder_msg):
        """Left wheel encoder callback (adapted from encoder_pose_node)"""
        if self.left_tick_prev is None:
            ticks = encoder_msg.data
            self.left_tick_prev = ticks
            return

        delta_phi_left, self.left_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.left_tick_prev,
        )
        self.delta_phi_left += delta_phi_left

        # Compute the new pose
        self.LEFT_RECEIVED = True
        self.calculate_pose()

    def cb_right_encoder(self, encoder_msg):
        """Right wheel encoder callback (adapted from encoder_pose_node)"""
        if self.right_tick_prev is None:
            ticks = encoder_msg.data
            self.right_tick_prev = ticks
            return

        delta_phi_right, self.right_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.right_tick_prev,
        )
        self.delta_phi_right += delta_phi_right

        self.RIGHT_RECEIVED = True
        self.calculate_pose()

    def calculate_pose(self):
        if not (self.LEFT_RECEIVED and self.RIGHT_RECEIVED):
            return
        self.LEFT_RECEIVED = self.RIGHT_RECEIVED = False

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

        self.theta_curr = self.angle_clamp(theta_curr)

        self.duckiebot_is_moving = (
            abs(self.delta_phi_left) > 0 or
            abs(self.delta_phi_right) > 0
        )

        self.delta_phi_left = self.delta_phi_right = 0

        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr

    def angle_clamp(self, theta):
        if theta >= 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < 0:
            return theta + 2 * np.pi
        else:
            return theta

    def angle_diff(self, target_angle, current_angle):
        diff = target_angle - current_angle
        # Normalize to [-π, π]
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def set_led_color(self, r, g, b, color_name):

        try:
            pattern = LEDPattern()
            
            for i in range(5):
                led_color = ColorRGBA()
                led_color.r = r
                led_color.g = g
                led_color.b = b
                led_color.a = self._intensity
                pattern.rgb_vals.append(led_color)
            
            self._led_publisher.publish(pattern)
            self.log(f"LED pattern published: {color_name} RGB[{r}, {g}, {b}]")
            
        except Exception as e:
            self.logwarn(f"Failed to publish LED pattern: {str(e)}")

    def turn_off_leds(self):
        self.set_led_color(0.0, 0.0, 0.0, "OFF")

    def publish_car_cmd(self, linear_v, angular_v):
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = linear_v      # linear velocity
        car_control_msg.omega = angular_v # angular velocity
        self._car_cmd_publisher.publish(car_control_msg)

    def stop_robot(self):
        self.publish_car_cmd(0.0, 0.0)

    def start_action(self):
        self._action_start_time = rospy.Time.now()

    def action_duration_elapsed(self, target_duration):
        if self._action_start_time is None:
            return False
        current_time = rospy.Time.now()
        elapsed = (current_time - self._action_start_time).to_sec()
        return elapsed >= target_duration

    def transition_to_state(self, new_state):
        self.log(f"State transition: {self._current_state.name} -> {new_state.name}")
        self._current_state = new_state
        self._action_start_time = None

    def calculate_position_error(self, target_x, target_y):
        dx = target_x - self.x_curr
        dy = target_y - self.y_curr
        return np.sqrt(dx*dx + dy*dy)

    def calculate_position_control(self, target_x, target_y):
        dx = target_x - self.x_curr
        dy = target_y - self.y_curr
        distance_error = np.sqrt(dx*dx + dy*dy)
        
        desired_theta = np.arctan2(dy, dx)
        
        angle_error = self.angle_diff(desired_theta, self.theta_curr)
        
        v = self.speed_gain if distance_error > self.position_threshold else 0.0
        omega = self.steer_gain * angle_error
        
        max_omega = 2.0
        omega = np.clip(omega, -max_omega, max_omega)
        
        return v, omega, distance_error, angle_error

    def calculate_orientation_control(self, target_theta):
        angle_error = self.angle_diff(target_theta, self.theta_curr)
        
        v = 0.0
        omega = self.steer_gain * angle_error
        
        max_omega = 2.0
        omega = np.clip(omega, -max_omega, max_omega)
        
        return v, omega, angle_error

    def state_machine_step(self):
        
        if self._current_state == SquareState.INIT:
            self.log("Starting square movement with encoder feedback...")
            self.log(f"Initial position: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")
            self.transition_to_state(SquareState.WAIT_FOR_ENCODERS)
            
        elif self._current_state == SquareState.WAIT_FOR_ENCODERS:
            if self.left_tick_prev is not None and self.right_tick_prev is not None:
                self.log("Encoder data received, starting square movement")
                self.transition_to_state(SquareState.SET_CORNER_LED)
            else:
                self.log("Waiting for encoder data...")
                rospy.sleep(0.1)
                
        elif self._current_state == SquareState.SET_CORNER_LED:
            rgb = self._corner_colors[self._current_corner]
            color_name = self._color_names[self._current_corner]
            
            self.set_led_color(rgb[0], rgb[1], rgb[2], color_name)
            self.log(f"Corner {self._current_corner + 1}: LED set to {color_name}")
            self.log(f"Current pose: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")

            self.transition_to_state(SquareState.MOVING_FORWARD)
                
        elif self._current_state == SquareState.MOVING_FORWARD:
            target_x, target_y = self._corner_positions[self._current_corner + 1]

            v, omega, distance_error, angle_error = self.calculate_position_control(target_x, target_y)
            
            if self._action_start_time is None:
                self.log(f"Moving to position ({target_x:.2f}, {target_y:.2f})")
                self.start_action()
            
            self.publish_car_cmd(v, omega)
            
            if self.action_duration_elapsed(2.0):
                self.log(f"Progress: pos=({self.x_curr:.3f}, {self.y_curr:.3f}), "
                        f"θ={np.rad2deg(self.theta_curr):.1f}°, "
                        f"dist_err={distance_error:.3f}m, ang_err={np.rad2deg(angle_error):.1f}°")
                self.start_action()  # Reset timer for next log
            
            if distance_error < self.position_threshold:
                self.stop_robot()
                self.log(f"Reached position ({target_x:.2f}, {target_y:.2f})")
                
                if self._current_corner >= 3:
                    self.transition_to_state(SquareState.COMPLETED)
                else:
                    rospy.sleep(0.5)  # Brief pause
                    self.transition_to_state(SquareState.TURNING_LEFT)
                    
        elif self._current_state == SquareState.TURNING_LEFT:
            target_theta = self._corner_orientations[self._current_corner + 1]
            
            v, omega, angle_error = self.calculate_orientation_control(target_theta)
            
            if self._action_start_time is None:
                self.log(f"Turning to orientation {np.rad2deg(target_theta):.1f}°")
                self.start_action()
            
            self.publish_car_cmd(v, omega)

            if self.action_duration_elapsed(1.0):
                self.log(f"Turn progress: current_θ={np.rad2deg(self.theta_curr):.1f}°, "
                        f"target_θ={np.rad2deg(target_theta):.1f}°, "
                        f"ang_err={np.rad2deg(angle_error):.1f}°")
                self.start_action()  # Reset timer for next log
            
            if abs(angle_error) < self.angle_threshold:
                self.stop_robot()
                self.log(f"Reached orientation {np.rad2deg(target_theta):.1f}°")
                
                self._current_corner += 1
                
                rospy.sleep(0.5)
                self.transition_to_state(SquareState.SET_CORNER_LED)
                
        elif self._current_state == SquareState.COMPLETED:
            self.turn_off_leds()
            self.log("LEDs turned off")
            self.log(f"Square movement completed!")
            self.log(f"Final position: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")
            self.log(f"Total distance traveled: {self.total_distance:.3f} m")
            self.transition_to_state(SquareState.SHUTDOWN)
            
        elif self._current_state == SquareState.SHUTDOWN:
            self.stop_robot()
            self.log("State machine shutting down...")
            return False  
            
        return True  

    def run(self):
        self.log("Waiting for system initialization...")
        rospy.sleep(2.0)
        
        while not rospy.is_shutdown():
            if (self.left_tick_prev is not None and self.right_tick_prev is not None) or \
               self._current_state in [SquareState.INIT, SquareState.WAIT_FOR_ENCODERS]:
                
                continue_execution = self.state_machine_step()
                
                if not continue_execution:
                    break
            else:
                self.log("Waiting for encoder initialization...")
                rospy.sleep(0.5)
                
            self._state_machine_rate.sleep()
        
        self.log("Square controller state machine completed. Keeping node alive for 2 seconds...")
        rospy.sleep(2.0)

    def on_shutdown(self):
        self.stop_robot()
        self.turn_off_leds()
        self.log("LEDs turned off")
        self.log("Square controller state machine shutting down...")
        self.log(f"Final odometry: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°")
        self.log(f"Total distance traveled: {self.total_distance:.3f} m")


if __name__ == '__main__':
    node = SquareControllerStateMachineNode(node_name='square_controller_state_machine_node')
    
    rospy.on_shutdown(node.on_shutdown)
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        node.logerr(f"Unexpected error: {str(e)}")
        
    node.stop_robot()
    node.turn_off_leds()