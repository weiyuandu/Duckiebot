#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge

class CameraReaderNode(DTROS):

   def __init__(self, node_name):
       super(CameraReaderNode, self).__init__(
           node_name=node_name,
           node_type=NodeType.VISUALIZATION
       )
       self._vehicle_name = os.environ['VEHICLE_NAME']
       self._camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
       self._bridge = CvBridge()
       self._window = "camera-reader"
       cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)
       self.sub = rospy.Subscriber(self._camera_topic, CompressedImage, self.callback)

   def callback(self, msg):
       image = self._bridge.compressed_imgmsg_to_cv2(msg)
       hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

       lower_green = (40, 70, 70)
       upper_green = (80, 255, 255)
       # Red (two ranges due to HSV wrap-around)
       lower_red1 = (0, 70, 50)
       upper_red1 = (10, 255, 255)
       lower_red2 = (170, 70, 50)
       upper_red2 = (180, 255, 255)
       # Blue
       lower_blue = (100, 150, 0)
       upper_blue = (140, 255, 255)

       # Create masks
       mask_green = cv2.inRange(hsv, lower_green, upper_green)
       mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
       mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
       mask_red = cv2.bitwise_or(mask_red1, mask_red2)
       mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

       # Find contours and draw them
       for mask, color, name in zip(
           [mask_green, mask_red, mask_blue],
           [(0,255,0), (0,0,255), (255,0,0)],
           ["Green", "Red", "Blue"]
       ):
           contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
           for cnt in contours:
               area = cv2.contourArea(cnt)
               if area > 500:  # filter small areas
                   cv2.drawContours(image, [cnt], -1, color, 3)
                   # Compute contour center
                   M = cv2.moments(cnt)
                   if M["m00"] != 0:
                       cX = int(M["m10"] / M["m00"])
                       cY = int(M["m01"] / M["m00"])
                       cv2.putText(
                           image, name, (cX-30, cY),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA
                       )
       cv2.imshow(self._window, image)
       cv2.waitKey(1)

if __name__ == '__main__':
   node = CameraReaderNode(node_name='camera_reader_node')
   rospy.spin()