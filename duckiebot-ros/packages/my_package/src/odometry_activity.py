#!/usr/bin/env python3

# Functions in this file are taken and adapted from the Duckietown MOOC:
# https://github.com/duckietown/mooc-exercises/tree/daffy/modcon/solution/04-Odometry

from duckietown_msgs.msg import WheelEncoderStamped
import numpy as np


def delta_phi(encoder_msg: WheelEncoderStamped, prev_ticks):
    """
    Calculate the change in wheel rotation based on the incoming message and
    the previous number of ticks read.
    
    Parameters
    ----------
    encoder_msg : WheelEncoderStamped
        The encoder message containing current tick count and resolution
    prev_ticks : int
        The previous tick count
        
    Returns
    -------
    delta_phi : float
        The change in wheel rotation in radians
    ticks : int
        The current tick count
    """
    ticks = encoder_msg.data
    delta_ticks = ticks - prev_ticks
    n_total = encoder_msg.resolution  # ticks per full rotation
    alpha = 2 * np.pi / n_total
    delta_phi = alpha * delta_ticks

    return delta_phi, ticks


def estimate_pose(
    R,
    baseline_wheel2wheel,
    x_prev,
    y_prev,
    theta_prev,
    delta_phi_left,
    delta_phi_right,
):
    """
    Estimate the pose of the Duckiebot using differential drive kinematics

    Parameters
    ----------
    R : float
        The radius of the wheels (meters)
    baseline_wheel2wheel : float
        The distance between the two wheels (meters)
    x_prev : float
        The previous x position of the Duckiebot (meters)
    y_prev : float
        The previous y position of the Duckiebot (meters)
    theta_prev : float
        The previous orientation angle of the Duckiebot (radians)
    delta_phi_left : float
        The change in the left wheel rotation (radians)
    delta_phi_right : float
        The change in the right wheel rotation (radians)
        
    Returns
    -------
    x : float
        The new x position (meters)
    y : float
        The new y position (meters)
    theta : float
        The new orientation angle (radians)
    dA : float
        The distance traveled (meters)
    """
    # Assume both wheels have the same radius
    R_left = R_right = R

    # Calculate distance traveled by each wheel
    d_left = R_left * delta_phi_left
    d_right = R_right * delta_phi_right
    
    # Calculate the distance traveled by the center of the robot
    dA = (d_left + d_right) / 2

    # Calculate the change in orientation
    dtheta = (d_right - d_left) / baseline_wheel2wheel

    # Calculate the change in position using the previous orientation
    # This assumes the robot moves in a straight line during the small time interval
    dx = dA * np.cos(theta_prev)
    dy = dA * np.sin(theta_prev)

    # Update pose estimate
    x = x_prev + dx
    y = y_prev + dy
    theta = theta_prev + dtheta

    return x, y, theta, dA
