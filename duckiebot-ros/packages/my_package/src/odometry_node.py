#!/usr/bin/env python3

"""
This node computes the pose estimate of the Duckiebot using wheel encoder data
and publishes the odometry information. 
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

    def __init__(self, node_name):
        super(OdometryNode, self).__init__(
            node_name=node_name, 
            node_type=NodeType.PERCEPTION
        )

        self.log(f"Initializing {node_name}...")

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

        self.reset_odometry()

        self.left_tick_prev: Optional[int] = None
        self.right_tick_prev: Optional[int] = None

        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

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
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.x_prev = 0.0
        self.y_prev = 0.0
        self.theta_prev = 0.0
        
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.total_distance = 0.0
        
        self.last_update_time = rospy.Time.now()

    def cb_left_encoder(self, encoder_msg):

        if self.left_tick_prev is None:
            self.left_tick_prev = encoder_msg.data
            return

        delta_phi_left, self.left_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.left_tick_prev
        )
        
        self.delta_phi_left += delta_phi_left

        self.left_received = True
        
        self.compute_odometry()

    def cb_right_encoder(self, encoder_msg):

        if self.right_tick_prev is None:
            self.right_tick_prev = encoder_msg.data
            return

        delta_phi_right, self.right_tick_prev = odometry_activity.delta_phi(
            encoder_msg, self.right_tick_prev
        )
        
        self.delta_phi_right += delta_phi_right

        self.right_received = True
        
        self.compute_odometry()

    def compute_odometry(self):
        if not (self.left_received and self.right_received):
            return

        self.left_received = False
        self.right_received = False

        if abs(self.delta_phi_left) < 1e-6 and abs(self.delta_phi_right) < 1e-6:
            self.delta_phi_left = 0.0
            self.delta_phi_right = 0.0
            return

        self.x_prev = self.x
        self.y_prev = self.y
        self.theta_prev = self.theta

        self.x, self.y, self.theta, distance_traveled = odometry_activity.estimate_pose(
            self.R,
            self.baseline,
            self.x_prev,
            self.y_prev,
            self.theta_prev,
            self.delta_phi_left,
            self.delta_phi_right
        )

        self.total_distance += abs(distance_traveled)

        self.theta = self.normalize_angle(self.theta)

        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        
        if dt > 0:
            self.linear_velocity = distance_traveled / dt
            self.angular_velocity = self.normalize_angle(self.theta - self.theta_prev) / dt
        else:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        self.last_update_time = current_time


        self.publish_odometry(current_time)

        self.delta_phi_left = 0.0
        self.delta_phi_right = 0.0

        self.log(f"Pose: x={self.x:.3f}, y={self.y:.3f}, θ={np.rad2deg(self.theta):.1f}°")

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def publish_odometry(self, timestamp):

        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = f"{self.veh_name}/base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0


        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.theta)

        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = self.angular_velocity

        pos_variance = 0.001 + 0.0001 * self.total_distance
        odom_msg.pose.covariance[0] = pos_variance  # x
        odom_msg.pose.covariance[7] = pos_variance  # y
        odom_msg.pose.covariance[35] = 0.01  # theta

        odom_msg.twist.covariance[0] = 0.001  # linear x
        odom_msg.twist.covariance[35] = 0.01  # angular z

        self.pub_odom.publish(odom_msg)

        pose_msg = Pose2DStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = "odom"
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta

        self.pub_pose.publish(pose_msg)


        self.publish_transform(timestamp)

    def yaw_to_quaternion(self, yaw):

        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = np.sin(yaw / 2.0)
        quat.w = np.cos(yaw / 2.0)
        return quat

    def publish_transform(self, timestamp):

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = f"{self.veh_name}/base_link"


        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation = self.yaw_to_quaternion(self.theta)

        self.tf_broadcaster.sendTransform(t)

    def on_shutdown(self):
        """Cleanup when shutting down"""
        self.log("Odometry node shutting down...")
        self.log(f"Final pose: x={self.x:.3f}, y={self.y:.3f}, θ={np.rad2deg(self.theta):.1f}°")
        self.log(f"Total distance traveled: {self.total_distance:.3f} m")


if __name__ == '__main__':
    odometry_node = OdometryNode(node_name='odometry_node')
    
    rospy.on_shutdown(odometry_node.on_shutdown)
    
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
