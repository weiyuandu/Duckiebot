#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped, LEDPattern, WheelEncoderStamped, Pose2DStamped
from duckietown_msgs.srv import ChangePattern
from std_msgs.msg import ColorRGBA, String, Empty
from enum import Enum
from typing import Optional
import time
import numpy as np
import rosbag

import odometry_activity


class _SquareState(Enum):
    """
    States for the square controller state machine
    """
    STOP = 0
    SIDE1 = 1      # First side - move forward
    TURN1 = 2      # First turn - 90° left
    SIDE2 = 3      # Second side - move forward  
    TURN2 = 4      # Second turn - 90° left
    SIDE3 = 5      # Third side - move forward
    TURN3 = 6      # Third turn - 90° left
    SIDE4 = 7      # Fourth side - move forward
    TURN4 = 8      # Final turn - 90° left
    COMPLETE = 9   # Square completed

class SquareControllerNode(DTROS):
    """
    State machine square controller with encoder-based odometry
    for precise 1m x 1m square path execution
    """
    
    right_tick_prev: Optional[int]
    left_tick_prev: Optional[int] 
    delta_phi_left: float
    delta_phi_right: float
    
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(SquareControllerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL
        )
        
        self._state = None
        self.log(f"Initializing Square Controller State Machine...")
        
        # Get vehicle name from namespace
        self.veh_name = rospy.get_namespace().strip("/")
        if not self.veh_name:
            self.veh_name = os.environ.get('VEHICLE_NAME', 'duck')
        
        # Initialize parameters
        self.resetParameters()
        
        # Square path parameters
        self.side_length = 1.0  # 1 meter sides
        self.turn_angle = np.pi/2  # 90 degree turns
        
        # State tracking variables
        self.segment_start_dist = 0
        self.segment_start_angle = 0
        
        # Record start time
        self.start_time = time.time()
        
        # Write world frame info to rosbag
        self.world_frame_bag = rosbag.Bag(
            f"/data/bags/square_world_frame_{self.start_time}.bag", "w",
        )
        
        # Load kinematics calibration
        self.log("Loading kinematics calibration...")
        self.R = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius',
            0.318,
        )  # meters, wheel radius
        self.baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline', 0.1
        )  # meters, baseline
        
        # Activities flags
        self.ODOMETRY_ACTIVITY = True
        self.ACTIVITY = True
        
        # Control parameters
        self.speed_gain = 0.4  # Linear velocity
        self.steer_gain = 2.0  # Angular velocity for turns
        self.correction_gain = 3.0  # Correction gain for straight line
        
        # Thresholds
        self.dist_threshold = 0.05  # Distance threshold (5cm)
        self.angle_threshold = np.deg2rad(5)  # Angle threshold (5 degrees)
        
        # Wait until encoders data is received
        self.duckiebot_is_moving = False
        self.STOP = False
        
        # For encoders synchronization
        self.RIGHT_RECEIVED = False
        self.LEFT_RECEIVED = False
        
        # #####################################
        # Command publishers
        # #####################################
        # For shutting down nodes
        shutdown_cmd_topic = f"/{self.veh_name}/shutdown"
        self.pub_shutdown_cmd = rospy.Publisher(
            shutdown_cmd_topic, Empty, queue_size=1,
        )
        
        # Publisher for movement commands
        car_cmd_topic = f"/{self.veh_name}/car_cmd_switch_node/cmd"
        self._car_cmd_publisher = rospy.Publisher(
            car_cmd_topic, 
            Twist2DStamped, 
            queue_size=1,
            dt_topic_type=TopicType.CONTROL,
        )

        # Publisher for LED control
        led_topic = f"/{self.veh_name}/led_emitter_node/led_pattern"
        self._led_publisher = rospy.Publisher(
            led_topic,
            LEDPattern,
            queue_size=1
        )
        
        # ####################################
        # Defining subscribers
        # ####################################
        # Wheel encoder subscribers
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
        
        # ########################################
        # Set the service for changing LED colour
        # ########################################
        try:
            self._svc_name = f"/{self.veh_name}/light_service_node/light_change"
            rospy.wait_for_service(self._svc_name, timeout=5.0)
            self._light_service = rospy.ServiceProxy(self._svc_name, ChangePattern)
            self.use_led_service = True
        except rospy.ROSException:
            self.log("LED service not available, using LED pattern publisher")
            self.use_led_service = False
        
        # ########################################
        # Set the colours for different states
        # ########################################
        self._light_colour = {
            _SquareState.STOP: String("RED"),
            _SquareState.SIDE1: String("GREEN"),
            _SquareState.TURN1: String("BLUE"),
            _SquareState.SIDE2: String("GREEN"),
            _SquareState.TURN2: String("BLUE"),
            _SquareState.SIDE3: String("GREEN"),
            _SquareState.TURN3: String("BLUE"),
            _SquareState.SIDE4: String("GREEN"),
            _SquareState.TURN4: String("BLUE"),
            _SquareState.COMPLETE: String("WHITE"),
        }

        # RGB colors for LED pattern
        self._rgb_colours = {
            _SquareState.STOP: [1.0, 0.0, 0.0],      # Red
            _SquareState.SIDE1: [0.0, 1.0, 0.0],     # Green
            _SquareState.TURN1: [0.0, 0.0, 1.0],     # Blue
            _SquareState.SIDE2: [0.0, 1.0, 0.0],     # Green
            _SquareState.TURN2: [0.0, 0.0, 1.0],     # Blue
            _SquareState.SIDE3: [0.0, 1.0, 0.0],     # Green
            _SquareState.TURN3: [0.0, 0.0, 1.0],     # Blue
            _SquareState.SIDE4: [0.0, 1.0, 0.0],     # Green
            _SquareState.TURN4: [0.0, 0.0, 1.0],     # Blue
            _SquareState.COMPLETE: [1.0, 1.0, 1.0],  # White
        }
        
        # Start the state machine
        self._stop()
        rospy.sleep(2.0)  # Wait for initialization
        self._set_state(_SquareState.SIDE1)

        self.log(f"Square Controller State Machine initialized.")

    def resetParameters(self):
        """
        Reset parameters at the beginning
        """
        # Change in left/right wheel (metres) since previous reading
        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

        # Last number of wheel encoder ticks read
        self.left_tick_prev = None
        self.right_tick_prev = None

        # Initialize the odometry and total distance travelled
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0  # Start facing 0 degrees (positive x-axis)
        self.dist = 0

        self.x_curr = 0.0
        self.y_curr = 0.0
        self.theta_curr = 0.0

    def cb_left_encoder(self, encoder_msg):
        """
        Left wheel encoder callback
        """
        if not (self.ODOMETRY_ACTIVITY or self.ACTIVITY):
            return

        if self.left_tick_prev is None:
            ticks = encoder_msg.data
            self.left_tick_prev = ticks
            return

        delta_phi_left, self.left_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.left_tick_prev,
        )
        self.delta_phi_left += delta_phi_left

        self.LEFT_RECEIVED = True
        self.calculate_pose()

    def cb_right_encoder(self, encoder_msg):
        """
        Right wheel encoder callback
        """
        if not (self.ODOMETRY_ACTIVITY or self.ACTIVITY):
            return

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
        """
        Calculate the pose using kinematics model and encoders
        """
        if self.STOP or not (self.LEFT_RECEIVED and self.RIGHT_RECEIVED):
            return

        # Sync incoming messages from encoders
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

        self.dist += np.abs(dA)

        # Clamp the angle to stay in [0, 2π)
        self.theta_curr = self.angle_clamp(theta_curr)

        # Save world frame data to rosbag
        pose = Pose2DStamped()
        pose.header.stamp = rospy.Time.now()
        pose.x = self.x_curr
        pose.y = self.y_curr
        pose.theta = self.theta_curr
        self.world_frame_bag.write("world_frame", pose)

        # Log current pose
        self.log(f"Pose: x={self.x_curr:.3f}, y={self.y_curr:.3f}, θ={np.rad2deg(self.theta_curr):.1f}°, dist={self.dist:.3f}")

        self.duckiebot_is_moving = (
            abs(self.delta_phi_left) > 0 or
            abs(self.delta_phi_right) > 0
        )

        # Reset the change in wheels
        self.delta_phi_left = self.delta_phi_right = 0

        # Update pose estimates
        self.x_prev = self.x_curr
        self.y_prev = self.y_curr
        self.theta_prev = self.theta_curr

        # Run the controller
        if self.ACTIVITY and self._state is not None:
            self.Controller()

    def Controller(self):
        """
        State machine controller - main logic
        """
        rospy.loginfo(f"=== State: {self._state}")

        if self._state == _SquareState.SIDE1:
            self._do_straight_side(1)
        elif self._state == _SquareState.TURN1:
            self._do_turn(1)
        elif self._state == _SquareState.SIDE2:
            self._do_straight_side(2)
        elif self._state == _SquareState.TURN2:
            self._do_turn(2)
        elif self._state == _SquareState.SIDE3:
            self._do_straight_side(3)
        elif self._state == _SquareState.TURN3:
            self._do_turn(3)
        elif self._state == _SquareState.SIDE4:
            self._do_straight_side(4)
        elif self._state == _SquareState.TURN4:
            self._do_turn(4)
        elif self._state == _SquareState.COMPLETE:
            self._do_complete()
        elif self._state == _SquareState.STOP:
            self.publishCmd((0, 0))
        else:
            rospy.logwarn(f"Unknown state: {self._state}")

    def _do_straight_side(self, side_num):
        """
        Execute straight line movement for one side of the square
        """
        # Initialize segment tracking
        if not hasattr(self, f'_side{side_num}_started'):
            self.segment_start_dist = self.dist
            self.segment_start_angle = self.theta_curr
            setattr(self, f'_side{side_num}_started', True)
            rospy.loginfo(f"Starting side {side_num} at distance {self.dist:.3f}m, angle {np.rad2deg(self.theta_curr):.1f}°")

        # Calculate distance traveled on this side
        distance_traveled = self.dist - self.segment_start_dist

        # Check if side is completed
        if distance_traveled >= self.side_length - self.dist_threshold:
            self.publishCmd((0, 0))
            rospy.loginfo(f"Side {side_num} completed! Traveled: {distance_traveled:.3f}m")
            
            # Remove the side started flag
            delattr(self, f'_side{side_num}_started')
            
            # Transition to next turn state
            next_states = {
                1: _SquareState.TURN1,
                2: _SquareState.TURN2,
                3: _SquareState.TURN3,
                4: _SquareState.TURN4
            }
            self._set_state(next_states[side_num])
            return

        # Continue moving forward with angle correction
        v = self.speed_gain
        omega = self._calculate_straight_correction()
        self.publishCmd((v, omega))
        
        rospy.loginfo(f"Side {side_num}: {distance_traveled:.3f}/{self.side_length:.3f}m, correction: {omega:.3f}")

    def _do_turn(self, turn_num):
        """
        Execute 90-degree left turn
        """
        # Initialize turn tracking
        if not hasattr(self, f'_turn{turn_num}_started'):
            self.segment_start_angle = self.theta_curr
            setattr(self, f'_turn{turn_num}_started', True)
            rospy.loginfo(f"Starting turn {turn_num} at angle {np.rad2deg(self.theta_curr):.1f}°")

        # Calculate target angle (90 degrees counterclockwise)
        target_angle = self.segment_start_angle + self.turn_angle
        target_angle = self.angle_clamp(target_angle)

        # Calculate angle difference
        angle_diff = self._calculate_angle_diff(self.theta_curr, target_angle)

        # Check if turn is completed
        if abs(angle_diff) <= self.angle_threshold:
            self.publishCmd((0, 0))
            rospy.loginfo(f"Turn {turn_num} completed! Final angle: {np.rad2deg(self.theta_curr):.1f}°")
            
            # Remove the turn started flag
            delattr(self, f'_turn{turn_num}_started')
            
            # Transition to next state
            if turn_num < 4:
                next_states = {
                    1: _SquareState.SIDE2,
                    2: _SquareState.SIDE3,
                    3: _SquareState.SIDE4
                }
                self._set_state(next_states[turn_num])
            else:
                self._set_state(_SquareState.COMPLETE)
            return

        # Continue turning
        v = 0.0  # No forward movement during turn
        omega = self.steer_gain if angle_diff > 0 else -self.steer_gain
        self.publishCmd((v, omega))
        
        rospy.loginfo(f"Turn {turn_num}: angle diff {np.rad2deg(angle_diff):.1f}°")

    def _do_complete(self):
        """
        Square path completed
        """
        self.publishCmd((0, 0))
        total_time = time.time() - self.start_time
        
        rospy.loginfo("🎉 Square path completed!")
        rospy.loginfo(f"Total execution time: {total_time:.2f} seconds")
        rospy.loginfo(f"Final position: x={self.x_curr:.3f}m, y={self.y_curr:.3f}m, θ={np.rad2deg(self.theta_curr):.1f}°")
        rospy.loginfo(f"Total distance traveled: {self.dist:.3f}m")
        
        self.world_frame_bag.close()
        
        # Signal shutdown
        self.pub_shutdown_cmd.publish(Empty())
        rospy.signal_shutdown("Square path completed!")

    def _calculate_straight_correction(self):
        """
        Calculate angular velocity correction to maintain straight line
        """
        if hasattr(self, 'segment_start_angle'):
            target_angle = self.segment_start_angle
            angle_error = self._calculate_angle_diff(self.theta_curr, target_angle)
            return -self.correction_gain * angle_error
        else:
            return 0.0

    def _calculate_angle_diff(self, current, target):
        """
        Calculate the smallest angle difference between current and target (-π, π)
        """
        diff = target - current
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff

    def publishCmd(self, u):
        """
        Publish car command message
        """
        car_control_msg = Twist2DStamped()
        car_control_msg.header.stamp = rospy.Time.now()
        car_control_msg.v = u[0]      # linear velocity
        car_control_msg.omega = u[1]  # angular velocity
        self._car_cmd_publisher.publish(car_control_msg)

    def angle_clamp(self, theta):
        """
        Clamp angle to [0, 2π)
        """
        if theta >= 2 * np.pi:
            return theta - 2 * np.pi
        elif theta < 0:
            return theta + 2 * np.pi
        else:
            return theta

    def _stop(self):
        """
        Stop state: change LEDs to red and stop
        """
        self._set_state(_SquareState.STOP)

    def _set_state(self, state: _SquareState):
        """
        Set the state and change LED color
        """
        rospy.loginfo(f"=== Setting state to {state}")
        self._state = state

        # Set LED color
        try:
            if self.use_led_service and state in self._light_colour:
                msg = self._light_colour[state]
                self._light_service(msg)
            elif state in self._rgb_colours:
                self._set_led_pattern(self._rgb_colours[state])
        except Exception as e:
            rospy.logwarn(f"Failed to set LED: {e}")

        if state == _SquareState.STOP:
            rospy.sleep(2.0)

    def _set_led_pattern(self, rgb):
        """
        Set LED pattern using RGB values
        """
        try:
            pattern = LEDPattern()
            for i in range(5):  # 5 LEDs
                led_color = ColorRGBA()
                led_color.r = rgb[0]
                led_color.g = rgb[1]
                led_color.b = rgb[2]
                led_color.a = 1.0
                pattern.rgb_vals.append(led_color)
            self._led_publisher.publish(pattern)
        except Exception as e:
            rospy.logwarn(f"Failed to publish LED pattern: {e}")

    def run(self):
        """
        Main run loop for state machine
        """
        # State machine runs through encoder callbacks
        # Just keep the node alive
        rospy.spin()

    def on_shutdown(self):
        """Cleanup when shutting down"""
        self.publishCmd((0, 0))
        try:
            self._set_led_pattern([0.0, 0.0, 0.0])  # Turn off LEDs
        except:
            pass
        try:
            self.world_frame_bag.close()
        except:
            pass
        self.log("Square controller shutting down...")


if __name__ == '__main__':
    node = SquareControllerNode(node_name='square_controller_node')
    
    # Register shutdown hook
    rospy.on_shutdown(node.on_shutdown)
    
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
