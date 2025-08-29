#!/usr/bin/env python3

"""
Odometry Node for Duckiebot

This node computes the pose estimate of the Duckiebot using wheel encoder data
and publishes the odometry information. It's compatible with the existing
Duckiebot ROS infrastructure.

Author: Based on Duckietown MOOC examples
"""

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Pose2DStamped, WheelEncoderStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from typing import Optional
import numpy as np
import rospy
import tf2_ros
import geometry_msgs.msg
import odometry_activity


class OdometryNode(DTROS):
    """
    Computes an estimate of the Duckiebot pose using wheel encoders and
    publishes odometry information.

    Subscribers:
        ~/left_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            left wheel encoder ticks
        ~/right_wheel_encoder_node/tick (:obj:`WheelEncoderStamped`):
            right wheel encoder ticks
            
    Publishers:
        ~/odometry (:obj:`Odometry`): 
            robot odometry information
        ~/pose (:obj:`Pose2DStamped`): 
            simplified 2D pose information
    """

    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(
            node_name=node_name, 
            node_type=NodeType.PERCEPTION
        )

        self.log(f"Initializing {node_name}...")

        # Get the vehicle name from namespace
        self.veh_name = rospy.get_namespace().strip("/")
        if not self.veh_name:
            import os
            self.veh_name = os.environ.get('VEHICLE_NAME', 'duck')

        # Load kinematic parameters
        self.log("Loading kinematics calibration...")
        self.R = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/radius', 
            0.0318  # meters, wheel radius
        )
        self.baseline = rospy.get_param(
            f'/{self.veh_name}/kinematics_node/baseline', 
            0.1  # meters, distance between wheels
        )

        self.log(f"Wheel radius: {self.R:.4f} m")
        self.log(f"Baseline: {self.baseline:.4f} m")

        # Initialize odometry variables
        self.reset_odometry()

        # Previous encoder tick counts
        self.left_tick_prev: Optional[int] = None
        self.right_tick_prev: Optional[int] = None

        # Accumulated wheel rotations since last pose update
        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

        # Encoder synchronization flags
        self.left_received = False
        self.right_received = False

        # Publishers
        self.pub_odom = rospy.Publisher(
            f"/{self.veh_name}/odometry_node/odometry",
            Odometry,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )
        
        self.pub_pose = rospy.Publisher(
            f"/{self.veh_name}/odometry_node/pose",
            Pose2DStamped,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )

        # TF broadcaster for publishing transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribers
        left_encoder_topic = f"/{self.veh_name}/left_wheel_encoder_node/tick"
        rospy.Subscriber(
            left_encoder_topic,
            WheelEncoderStamped,
            self.cb_left_encoder,
            queue_size=1
        )

        right_encoder_topic = f"/{self.veh_name}/right_wheel_encoder_node/tick"
        rospy.Subscriber(
            right_encoder_topic,
            WheelEncoderStamped,
            self.cb_right_encoder,
            queue_size=1
        )

        self.log(f"Subscribing to: {left_encoder_topic}")
        self.log(f"Subscribing to: {right_encoder_topic}")
        self.log(f"Publishing odometry to: /{self.veh_name}/odometry_node/odometry")
        self.log(f"Publishing pose to: /{self.veh_name}/odometry_node/pose")
        self.log(f"{node_name} initialized successfully.")

    def reset_odometry(self):
        """Reset the odometry to initial values"""
        # Initial pose (starting at origin, facing forward)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous pose for calculation
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0
        
        # Velocity estimates
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        # Total distance traveled
        self.total_distance = 0.0
        
        # Last update time for velocity calculation
        self.last_update_time = rospy.Time.now()

    def cb_left_encoder(self, encoder_msg):
        """
        Callback for left wheel encoder messages
        
        Args:
            encoder_msg (WheelEncoderStamped): Left encoder message
        """
        # Initialize previous tick count if this is the first message
        if self.left_tick_prev is None:
            self.left_tick_prev = encoder_msg.data
            return

        # Calculate wheel rotation since last message
        delta_phi_left, self.left_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.left_tick_prev
        )
        
        # Accumulate rotation
        self.delta_phi_left += delta_phi_left

        # Mark that we received left encoder data
        self.left_received = True
        
        # Try to compute pose if both encoders have new data
        self.compute_odometry()

    def cb_right_encoder(self, encoder_msg):
        """
        Callback for right wheel encoder messages
        
        Args:
            encoder_msg (WheelEncoderStamped): Right encoder message
        """
        # Initialize previous tick count if this is the first message
        if self.right_tick_prev is None:
            self.right_tick_prev = encoder_msg.data
            return

        # Calculate wheel rotation since last message
        delta_phi_right, self.right_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.right_tick_prev
        )
        
        # Accumulate rotation
        self.delta_phi_right += delta_phi_right

        # Mark that we received right encoder data
        self.right_received = True
        
        # Try to compute pose if both encoders have new data
        self.compute_odometry()

    def compute_odometry(self):
        """
        Compute and publish odometry when both encoder measurements are available
        """
        # Only compute if we have data from both encoders
        if not (self.left_received and self.right_received):
            return

        # Reset synchronization flags
        self.left_received = False
        self.right_received = False

        # Skip if no movement detected
        if abs(self.delta_phi_left) < 1e-6 and abs(self.delta_phi_right) < 1e-6:
            self.delta_phi_left = 0.0
            self.delta_phi_right = 0.0
            return

        # Store previous pose
        self.x_prev = self.x
        self.y_prev = self.y
        self.theta_prev = self.theta

        # Compute new pose using odometry
        self.x, self.y, self.theta, distance_traveled = odometry_activity.estimate_pose(
            self.R,
            self.baseline,
            self.x_prev,
            self.y_prev,
            self.theta_prev,
            self.delta_phi_left,
            self.delta_phi_right
        )

        # Update total distance
        self.total_distance += abs(distance_traveled)

        # Normalize angle to [-π, π]
        self.theta = self.normalize_angle(self.theta)

        # Calculate velocities
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        
        if dt > 0:
            self.linear_velocity = distance_traveled / dt
            self.angular_velocity = self.normalize_angle(self.theta - self.theta_prev) / dt
        else:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        self.last_update_time = current_time

        # Publish odometry information
        self.publish_odometry(current_time)

        # Reset accumulated wheel rotations
        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

        # Log pose information
        self.log(f"Pose: x={self.x:.3f}, y={self.y:.3f}, θ={np.rad2deg(self.theta):.1f}°")

    def normalize_angle(self, angle):
        """
        Normalize angle to [-π, π]
        
        Args:
            angle (float): Input angle in radians
            
        Returns:
            float: Normalized angle in radians
        """
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def publish_odometry(self, timestamp):
        """
        Publish odometry information
        
        Args:
            timestamp (rospy.Time): Current timestamp
        """
        # Create and publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = f"{self.veh_name}/base_link"

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (convert from 2D angle to quaternion)
        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.theta)

        # Velocity
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        # Covariance (simple diagonal covariance matrix)
        # Position covariance increases with distance traveled
        pos_variance = 0.001 + 0.0001 * self.total_distance
        odom_msg.pose.covariance[0] = pos_variance  # x
        odom_msg.pose.covariance[7] = pos_variance  # y
        odom_msg.pose.covariance[35] = 0.01  # theta

        # Velocity covariance
        odom_msg.twist.covariance[0] = 0.001  # linear x
        odom_msg.twist.covariance[35] = 0.01  # angular z

        self.pub_odom.publish(odom_msg)

        # Create and publish Pose2DStamped message
        pose_msg = Pose2DStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "odom"
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta

        self.pub_pose.publish(pose_msg)

        # Publish transform
        self.publish_transform(timestamp)

    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle to quaternion
        
        Args:
            yaw (float): Yaw angle in radians
            
        Returns:
            geometry_msgs.msg.Quaternion: Quaternion representation
        """
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = np.sin(yaw / 2.0)
        quat.w = np.cos(yaw / 2.0)
        return quat

    def publish_transform(self, timestamp):
        """
        Publish transform from odom to base_link
        
        Args:
            timestamp (rospy.Time): Current timestamp
        """
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = f"{self.veh_name}/base_link"

        # Translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        # Rotation
        t.transform.rotation = self.yaw_to_quaternion(self.theta)

        self.tf_broadcaster.sendTransform(t)

    def on_shutdown(self):
        """Cleanup when shutting down"""
        self.log("Odometry node shutting down...")
        self.log(f"Final pose: x={self.x:.3f}, y={self.y:.3f}, θ={np.rad2deg(self.theta):.1f}°")
        self.log(f"Total distance traveled: {self.total_distance:.3f} m")


if __name__ == '__main__':
    # Create and run the odometry node
    odometry_node = OdometryNode(node_name='odometry_node')
    
    # Register shutdown hook
    rospy.on_shutdown(odometry_node.on_shutdown)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
