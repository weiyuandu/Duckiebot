#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
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
        
        # add publisher for led
        led_topic = f"/{self._vehicle_name}/led_emitter_node/led_pattern"
        self._led_publisher = rospy.Publisher(
            led_topic,
            LEDPattern,
            queue_size=1
        )
        self._intensity = 1.0
        self._current_led_color = None

    def set_led_color(self, r, g, b, color_name):
        pattern = LEDPattern()
        for _ in range(5):
            led_color = ColorRGBA()
            led_color.r = r
            led_color.g = g
            led_color.b = b
            led_color.a = self._intensity
            pattern.rgb_vals.append(led_color)
        
        self._led_publisher.publish(pattern)
        self.log(f"LED color set to {color_name}: RGB[{r}, {g}, {b}]")

    def turn_off_leds(self):
        self.set_led_color(0.0, 0.0, 0.0, "OFF")

    def update_led_based_on_color(self, dominant_color):
        """Update LED color based on the dominant detected color (Green, Blue, or Yellow)"""
        # Only update if the color changed to avoid constant publishing
        if dominant_color != self._current_led_color:
            if dominant_color == "Green":
                self.set_led_color(0.0, 1.0, 0.0, "Green")
            elif dominant_color == "Blue":
                self.set_led_color(0.0, 0.0, 1.0, "Blue")
            elif dominant_color == "Yellow":
                self.set_led_color(1.0, 1.0, 0.0, "Yellow")
            else:
                # No dominant color detected, turn off LEDs
                self.turn_off_leds()
            
            self._current_led_color = dominant_color

    def callback(self, msg):
        image = self._bridge.compressed_imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_green = (40, 70, 70)
        upper_green = (80, 255, 255)

        lower_blue = (100, 150, 0)
        upper_blue = (140, 255, 255)

        lower_yellow = (20, 100, 100)
        upper_yellow = (30, 255, 255)

        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        color_areas = {"Green": 0, "Blue": 0, "Yellow": 0}

        # Find contours and draw them
        for mask, color, name in zip(
            [mask_green, mask_blue, mask_yellow],
            [(0,255,0), (255,0,0), (0,255,255)],
            ["Green", "Blue", "Yellow"]
        ):
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            total_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 500:  # filter small areas
                    cv2.drawContours(image, [cnt], -1, color, 3)
                    total_area += area
                    # Compute contour center
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        cv2.putText(
                            image, name, (cX-30, cY),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA
                        )
            color_areas[name] = total_area
        
        # find the dominant color
        dominant_color = None
        max_area = max(color_areas.values())
        if max_area > 1000:
            for color_name, area in color_areas.items():
                if area == max_area:
                    dominant_color = color_name
                    break
        self.update_led_based_on_color(dominant_color)

        cv2.imshow(self._window, image)

    def on_shutdown(self):
        self.turn_off_leds()
        cv2.destroyAllWindows()
        self.log("Camera reader shutting down, LEDs turned off")

    def run(self):
        try:
            rate = rospy.Rate(30)  # 30 Hz
            while not rospy.is_shutdown():
                rate.sleep()
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    rospy.signal_shutdown("User pressed 'q' to quit")
                    break
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
        finally:
            self.on_shutdown()

if __name__ == '__main__':
    node = CameraReaderNode(node_name='camera_reader_node')
    
    rospy.on_shutdown(node.on_shutdown)
    node.run()