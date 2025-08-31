#!/usr/bin/env python3
"""
Lane keeping in gym-duckietown using visual servoing,
with optional debug overlay for lane detection.
"""

import argparse
from typing import Tuple
import time
import numpy as np
import cv2

from gym_duckietown.simulator import Simulator


# ---------- Perception ----------

def detect_lane_markings(image_rgb: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Segment left (yellow) and right (white) lane markings.
    Returns binary masks normalized to [0,1].
    """
    h, w, _ = image_rgb.shape

    # Region of interest: bottom ~60%
    roi = np.zeros((h, w), np.uint8)
    y0 = int(h * 0.4)
    cv2.rectangle(roi, (0, y0), (w, h), 255, -1)

    hsv = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2HSV)

    # Yellow mask
    lower_yellow = np.array([15, 80, 80], dtype=np.uint8)
    upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
    mask_y = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # White mask
    lower_white = np.array([0, 0, 190], dtype=np.uint8)
    upper_white = np.array([180, 60, 255], dtype=np.uint8)
    mask_w = cv2.inRange(hsv, lower_white, upper_white)

    # Apply ROI
    mask_y = cv2.bitwise_and(mask_y, roi)
    mask_w = cv2.bitwise_and(mask_w, roi)

    # Morph cleanup
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, k, iterations=1)
    mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_CLOSE, k, iterations=1)
    mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, k, iterations=1)
    mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_CLOSE, k, iterations=1)

    return (mask_y.astype(np.float32) / 255.0,
            mask_w.astype(np.float32) / 255.0)


# ---------- Steering matrices ----------

def _xy_weights(shape: Tuple[int, int]):
    h, w = shape
    x = np.linspace(-1.0, 1.0, w, dtype=np.float32)[None, :]
    y = np.linspace(0.0, 1.0, h, dtype=np.float32)[:, None]
    return x, y ** 1.5


def get_steer_matrix_left_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    x, y = _xy_weights(shape)
    return (-x) * y


def get_steer_matrix_right_lane_markings(shape: Tuple[int, int]) -> np.ndarray:
    x, y = _xy_weights(shape)
    return (x) * y


# ---------- Controller ----------

class LaneServoController:
    def __init__(self, img_shape: Tuple[int, int, int]):
        h, w, _ = img_shape
        self.steer_left = get_steer_matrix_left_lane_markings((h, w))
        self.steer_right = get_steer_matrix_right_lane_markings((h, w))

        self.k_yellow = 0.9
        self.k_white = 0.9
        self.turn_gain_to_wheels = 0.5
        self.base_speed = 0.30

        # State variables for stop sign handling
        self.stop_sign_state = "NORMAL"  # NORMAL, STOPPING, CROSSING
        self.stop_start_time = 0
        self.cross_start_time = 0

        # State variables for slow sign handling
        self.slow_sign_state = "NORMAL"  # NORMAL, SLOWING
        self.slow_start_time = 0
        self.slow_speed = 0.15  # Reduced speed for slow sign

        # State variables for traffic light handling
        self.traffic_light_state = "NORMAL"  # NORMAL, RED_LIGHT, GREEN_LIGHT
        self.traffic_light_start_time = 0
        self.last_light_detection = None

    def compute_action(self, obs_rgb: np.ndarray):
        mask_left, mask_right = detect_lane_markings(obs_rgb)

        def mean_weight(m, s):
            total = m.sum()
            return 0.0 if total < 1e-6 else float((m * s).sum() / total)

        left_mean = mean_weight(mask_left, self.steer_left)
        right_mean = mean_weight(mask_right, self.steer_right)

        omega = self.k_yellow * left_mean - self.k_white * right_mean
        omega = np.clip(omega, -1.0, 1.0)

        speed = self.base_speed * (1.0 - 0.5 * abs(omega))
        speed = float(np.clip(speed, 0.05, 0.8))

        left_wheel = speed - self.turn_gain_to_wheels * omega
        right_wheel = speed + self.turn_gain_to_wheels * omega
        return np.array([np.clip(left_wheel, -1, 1),
                         np.clip(right_wheel, -1, 1)], dtype=np.float32)

    @staticmethod
    def make_debug_frame(obs_rgb, mask_left, mask_right):
        """Stack RGB, yellow mask, and white mask side by side for debugging."""
        h, w, _ = obs_rgb.shape
        rgb_bgr = cv2.cvtColor(obs_rgb, cv2.COLOR_RGB2BGR)
        # Convert masks to 3-channel color images
        mask_y_vis = cv2.applyColorMap((mask_left * 255).astype(np.uint8), cv2.COLORMAP_AUTUMN)
        mask_w_vis = cv2.applyColorMap((mask_right * 255).astype(np.uint8), cv2.COLORMAP_BONE)
        return np.hstack([rgb_bgr, mask_y_vis, mask_w_vis])

    def detect_light(self, image):
        h, w, _ = image.shape
        roi = image[0:int(h / 3), int(w / 3):int(2 * w / 3)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 120])
        upper_red2 = np.array([179, 255, 255])
        lower_green = np.array([40, 120, 120])
        upper_green = np.array([90, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        gray = cv2.cvtColor(roi, cv2.COLOR_RGB2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=20, param1=50, param2=15, minRadius=5,
                                   maxRadius=30)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            h_roi, w_roi = red_mask.shape
            for (x, y, r) in circles[0, :]:
                if 0 <= x < w_roi and 0 <= y < h_roi:
                    if red_mask[y, x] > 0:
                        return "RED"
                    elif green_mask[y, x] > 0:
                        return "GREEN"

        return None

    def detect_stop_line(self, image_rgb: np.ndarray) -> bool:
        """
        Detect red stop line at the bottom of the image
        """
        h, w, _ = image_rgb.shape
        # Look at bottom 20% of image
        roi_bottom = image_rgb[int(0.8 * h):h, :]

        hsv = cv2.cvtColor(roi_bottom, cv2.COLOR_RGB2HSV)

        # Red color range
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        # Count red pixels
        red_pixels = cv2.countNonZero(red_mask)
        total_pixels = roi_bottom.shape[0] * roi_bottom.shape[1]

        # If more than 5% of bottom area is red, consider it a stop line
        return (red_pixels / total_pixels) > 0.05

    def detect_sign(self, image):
        h, w, _ = image.shape
        roi = image[int(h / 3):int(2 * h / 3), int(2 * w / 3):w]
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        """STOP"""
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

        """SLOW"""
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([35, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours_red:
            area = cv2.contourArea(cnt)
            if area > 300:
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                if len(approx) >= 6:
                    x, y, w, h = cv2.boundingRect(cnt)
                    cv2.putText(image, "STOP", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                    return "STOP"

        for cnt in contours_yellow:
            area = cv2.contourArea(cnt)
            if area > 300:
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                if len(approx) == 4:
                    x, y, w_box, h_box = cv2.boundingRect(cnt)

                    aspect_r = float(w_box) / h_box
                    if not (0.8 < aspect_r < 1.2):
                        continue

                    rect = cv2.minAreaRect(cnt)
                    angle = rect[2]

                    if not (20 < abs(angle) < 70):
                        continue

                    if y + h_box > 0.8 * h:
                        continue

                    cv2.putText(image, "SLOW", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                    return "SLOW"
        return None

    def handle_stop_sign(self, obs: np.ndarray, action: np.ndarray) -> Tuple[np.ndarray, str]:
        """
        Handle stop sign state machine
        Returns modified action and new state
        """
        current_time = time.time()

        if self.stop_sign_state == "NORMAL":
            # Check if we see a stop sign and are at a stop line
            sign = self.detect_sign(obs)
            stop_line_detected = self.detect_stop_line(obs)

            if sign == "STOP" and stop_line_detected:
                print("Stop sign detected at stop line - stopping for 3 seconds")
                self.stop_sign_state = "STOPPING"
                self.stop_start_time = current_time
                return np.array([0.0, 0.0]), self.stop_sign_state

        elif self.stop_sign_state == "STOPPING":
            # Stop for 3 seconds
            if current_time - self.stop_start_time < 3.0:
                return np.array([0.0, 0.0]), self.stop_sign_state
            else:
                print("3 seconds passed - proceeding through intersection")
                self.stop_sign_state = "CROSSING"
                self.cross_start_time = current_time
                # Go straight
                return np.array([self.base_speed, self.base_speed]), self.stop_sign_state

        elif self.stop_sign_state == "CROSSING":
            # Go straight for 2 seconds
            if current_time - self.cross_start_time < 2.0:
                return np.array([self.base_speed, self.base_speed]), self.stop_sign_state
            else:
                print("Intersection crossed - resuming normal operation")
                self.stop_sign_state = "NORMAL"

        return action, self.stop_sign_state

    def handle_slow_sign(self, obs: np.ndarray, action: np.ndarray) -> Tuple[np.ndarray, str]:
        """
        Handle slow sign state machine
        Returns modified action and new state
        """
        current_time = time.time()

        if self.slow_sign_state == "NORMAL":
            # Check if we see a slow sign
            sign = self.detect_sign(obs)

            if sign == "SLOW":
                print("Slow sign detected - reducing speed for 3 seconds")
                self.slow_sign_state = "SLOWING"
                self.slow_start_time = current_time
                # Reduce speed but keep moving forward
                slow_action = self.compute_slow_action(obs)
                return slow_action, self.slow_sign_state

        elif self.slow_sign_state == "SLOWING":
            # Move slowly for 3 seconds
            if current_time - self.slow_start_time < 3.0:
                slow_action = self.compute_slow_action(obs)
                return slow_action, self.slow_sign_state
            else:
                print("3 seconds passed - resuming normal speed")
                self.slow_sign_state = "NORMAL"

        return action, self.slow_sign_state

    def handle_traffic_light(self, obs: np.ndarray, action: np.ndarray) -> Tuple[np.ndarray, str]:
        """
        Handle traffic light state machine
        Returns modified action and new state
        """
        current_time = time.time()
        light = self.detect_light(obs)

        # Only update detection if we actually see a light
        if light is not None:
            self.last_light_detection = light

        if self.traffic_light_state == "NORMAL":
            if self.last_light_detection == "RED":
                print("Red light detected - stopping")
                self.traffic_light_state = "RED_LIGHT"
                self.traffic_light_start_time = current_time
                return np.array([0.0, 0.0]), self.traffic_light_state
            elif self.last_light_detection == "GREEN":
                print("Green light detected - proceeding with caution")
                self.traffic_light_state = "GREEN_LIGHT"
                self.traffic_light_start_time = current_time
                # Continue with normal action for green light
                return action, self.traffic_light_state

        elif self.traffic_light_state == "RED_LIGHT":
            # Check if light has turned green
            if self.last_light_detection == "GREEN":
                print("Light turned green - proceeding")
                self.traffic_light_state = "GREEN_LIGHT"
                self.traffic_light_start_time = current_time
                return action, self.traffic_light_state

            # Stay stopped for red light
            return np.array([0.0, 0.0]), self.traffic_light_state

        elif self.traffic_light_state == "GREEN_LIGHT":
            # Check if light has turned red
            if self.last_light_detection == "RED":
                print("Light turned red - stopping")
                self.traffic_light_state = "RED_LIGHT"
                self.traffic_light_start_time = current_time
                return np.array([0.0, 0.0]), self.traffic_light_state

            # Continue with normal action for green light
            # Reset to normal if no light is detected for a while
            if current_time - self.traffic_light_start_time > 5.0 and self.last_light_detection is None:
                print("No light detected for 5 seconds - resuming normal operation")
                self.traffic_light_state = "NORMAL"

        return action, self.traffic_light_state

    def compute_slow_action(self, obs_rgb: np.ndarray):
        """
        Compute action with reduced speed for slow sign
        """
        mask_left, mask_right = detect_lane_markings(obs_rgb)

        def mean_weight(m, s):
            total = m.sum()
            return 0.0 if total < 1e-6 else float((m * s).sum() / total)

        left_mean = mean_weight(mask_left, self.steer_left)
        right_mean = mean_weight(mask_right, self.steer_right)

        omega = self.k_yellow * left_mean - self.k_white * right_mean
        omega = np.clip(omega, -1.0, 1.0)

        # Use reduced speed for slow sign
        speed = self.slow_speed * (1.0 - 0.5 * abs(omega))
        speed = float(np.clip(speed, 0.05, self.slow_speed))

        left_wheel = speed - self.turn_gain_to_wheels * omega
        right_wheel = speed + self.turn_gain_to_wheels * omega
        return np.array([np.clip(left_wheel, -1, 1),
                         np.clip(right_wheel, -1, 1)], dtype=np.float32)


# ---------- Main ----------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="loop_empty")
    ap.add_argument("--steps", type=int, default=200000)
    ap.add_argument("--no-gui", action="store_true")
    ap.add_argument("--debug", action="store_true", help="Show OpenCV debug overlay")
    ap.add_argument("--seed", type=int, default=123)
    ap.add_argument("--domain-rand", type=int, default=0)
    args = ap.parse_args()

    env = Simulator(
        seed=args.seed,
        map_name=args.map,
        max_steps=args.steps + 1,
        domain_rand=int(bool(args.domain_rand)),
        camera_width=640,
        camera_height=480,
        accept_start_angle_deg=4,
        full_transparency=True,
        distortion=False,
    )

    obs = env.reset()
    controller = LaneServoController(img_shape=obs.shape)

    step = 0

    try:
        while True:
            action = controller.compute_action(obs)

            """Handle traffic light with state machine"""
            action, light_state = controller.handle_traffic_light(obs, action)

            """Handle stop sign with state machine"""
            action, stop_state = controller.handle_stop_sign(obs, action)

            """Handle slow sign with state machine"""
            action, slow_state = controller.handle_slow_sign(obs, action)

            mask_left, mask_right = detect_lane_markings(obs)

            obs, reward, done, info = env.step(action)
            if not args.no_gui:
                env.render()

            if args.debug:
                dbg = controller.make_debug_frame(obs, mask_left, mask_right)
                cv2.imshow("Lane Servo Debug", dbg)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC quits
                    break

            if done:
                obs = env.reset()
                # Reset state machines when episode resets
                controller.stop_sign_state = "NORMAL"
                controller.slow_sign_state = "NORMAL"
                controller.traffic_light_state = "NORMAL"
                controller.last_light_detection = None
            step += 1
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        if args.debug:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()