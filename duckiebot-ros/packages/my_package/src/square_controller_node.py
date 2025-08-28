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
            self.log(f" LED pattern published: RGB[{r}, {g}, {b}]")
            
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
