#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

class TrafficLightDetectorNode(object):
    def __init__(self):
        self.use_compressed = rospy.get_param("~use_compressed", True)
        self.roi_top_frac   = rospy.get_param("~roi_top_frac", 2.0/5.0)  # top 2/5
        self.roi_x_margin   = rospy.get_param("~roi_x_margin", 2.0/5.0)  # middle 2/5
        self.print_only_on_change = rospy.get_param("~print_only_on_change", True)

        self.prev_state = None
        self.pub_state = rospy.Publisher("~state", String, queue_size=1)

        if self.use_compressed:
            rospy.Subscriber("~image", CompressedImage, self.cb_comp, queue_size=1, buff_size=2**22)
        else:
            rospy.Subscriber("~image", Image, self.cb_raw, queue_size=1)

        rospy.loginfo("[tl_det] Started. use_compressed=%s", self.use_compressed)

    def cb_comp(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame  = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is not None:
            self.process(frame)

    def cb_raw(self, msg):
        h = msg.height; w = msg.width
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, -1)
        self.process(frame)

    def process(self, bgr):
        h, w, _ = bgr.shape
        x0 = int(w * self.roi_x_margin)
        x1 = int(w * (1.0 - self.roi_x_margin))
        y1 = int(h * self.roi_top_frac)
        roi = bgr[0:y1, x0:x1]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([  0, 120, 120], dtype=np.uint8)
        upper_red1 = np.array([ 10, 255, 255], dtype=np.uint8)
        lower_red2 = np.array([160, 120, 120], dtype=np.uint8)
        upper_red2 = np.array([179, 255, 255], dtype=np.uint8)

        lower_green = np.array([ 40, 120, 120], dtype=np.uint8)
        upper_green = np.array([ 90, 255, 255], dtype=np.uint8)

        red_mask   = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20,
            param1=50, param2=15, minRadius=5, maxRadius=30
        )

        state = None
        if circles is not None:
            circles = np.uint16(np.around(circles))
            h_roi, w_roi = red_mask.shape
            for (x, y, r) in circles[0, :]:
                if 0 <= x < w_roi and 0 <= y < h_roi:
                    if red_mask[y, x] > 0:
                        state = "RED"; break
                    elif green_mask[y, x] > 0:
                        state = "GREEN"; break

        if self.print_only_on_change:
            if state != self.prev_state and state is not None:
                rospy.loginfo("[tl_det] %s", state)
                self.pub_state.publish(state)
        else:
            rospy.loginfo("[tl_det] %s", state if state else "NONE")
            if state: self.pub_state.publish(state)

        self.prev_state = state


if __name__ == "__main__":
    rospy.init_node("traffic_light_detector")
    TrafficLightDetectorNode()
    rospy.spin()
