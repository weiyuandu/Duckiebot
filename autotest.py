#!/usr/bin/env python3
import argparse, time
import numpy as np
import cv2
from gym_duckietown.envs import DuckietownEnv

# Tuned PID parameters
Kp = 0.008
Ki = 0.0
Kd = 0.002

integral = 0.0
last_error = 0.0
last_steer = 0.0
HUG_LEFT = None
LAST_LINE_SIDE = 0
LAST_LINE_SLOPE = 0.0

STEER_BLEND = 0.8
DEADBAND = 12       # increase deadband to tolerate small deviations
MIN_CORRECTION = 0.03
MAX_STEER = 0.7
BASE_SPEED = 0.3
MIN_SPEED = 0.1

def detect_yellow_lane(obs):
    global integral, last_error, last_steer, HUG_LEFT, LAST_LINE_SIDE, LAST_LINE_SLOPE

    hsv = cv2.cvtColor(obs, cv2.COLOR_RGB2HSV)
    mask_yellow = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([40, 255, 255]))

    h, w = mask_yellow.shape
    roi_start = int(h*0.3)
    roi_end = h

    slices = 5
    lane_centers = []
    weights = np.linspace(0.5, 1.0, slices)
    vis = obs.copy()

    for i, weight in enumerate(weights):
        y1 = roi_start + i*(roi_end - roi_start)//slices
        y2 = roi_start + (i+1)*(roi_end - roi_start)//slices
        roi_slice = mask_yellow[y1:y2, :]
        roi_slice = cv2.GaussianBlur(roi_slice, (5,5), 0)
        roi_slice = cv2.dilate(roi_slice, np.ones((3,3), np.uint8), iterations=1)
        edges = cv2.Canny(roi_slice, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=10, minLineLength=5, maxLineGap=5)

        line_x = []
        if lines is not None:
            for x1l, y1l, x2l, y2l in lines[:,0]:
                slope = (y2l - y1l) / (x2l - x1l + 1e-6)
                if abs(slope) < 0.3:
                    continue
                line_x.append((x1l + x2l)/2)
                cv2.line(vis, (x1l, y1+y1l), (x2l, y1+y2l), (0,255,255), 1)

        if line_x:
            lane_centers.append(weight * np.mean(line_x))

    if lane_centers:
        yellow_x = np.sum(lane_centers)/np.sum(weights)

        if HUG_LEFT is None:
            HUG_LEFT = yellow_x < w/2

        # Reduced offset to stay closer to the line
        offset = w * 0.08
        lane_center = yellow_x - offset if HUG_LEFT else yellow_x + offset

        # Lookahead for curvature
        curv_factor = 1.0
        if len(lane_centers) > 1:
            diffs = np.diff(lane_centers)
            LAST_LINE_SLOPE = np.mean(diffs)
            avg_slope_abs = np.mean(np.abs(diffs))
            curv_factor += 5.0 * avg_slope_abs / (w/2)
            curv_factor = min(curv_factor, 5.0)
        else:
            curv_factor = 1.0

        error_px = lane_center - w/2
        LAST_LINE_SIDE = -1 if error_px < 0 else 1

        if abs(error_px) < DEADBAND:
            steer = np.sign(error_px) * MIN_CORRECTION * curv_factor
        else:
            integral += error_px
            derivative = error_px - last_error
            steer = (Kp*error_px + Ki*integral + Kd*derivative) * curv_factor
            steer = np.clip(steer, -MAX_STEER, MAX_STEER)

        last_error = error_px
        speed = BASE_SPEED

    else:
        # Line lost → spin in place if never seen, or steer toward last seen direction
        if HUG_LEFT is None:
            steer = MAX_STEER  # spin clockwise to find line
        else:
            steer = MAX_STEER * np.sign(LAST_LINE_SLOPE)
        speed = MIN_SPEED

    steer = STEER_BLEND * last_steer + (1 - STEER_BLEND) * steer
    last_steer = steer

    # Visualization
    if lane_centers:
        cv2.line(vis, (int(lane_center), roi_start), (int(lane_center), h-1), (0,0,255), 2)
    steer_px = int(steer * w/2)
    cv2.arrowedLine(vis, (w//2, h), (w//2 + steer_px, h-50), (255,0,0), 3)

    return float(steer), float(speed), vis

def main():
    global HUG_LEFT
    ap = argparse.ArgumentParser()
    ap.add_argument("--env-name", default="small_loop")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    env = DuckietownEnv(
        map_name=args.env_name,
        domain_rand=False, draw_curve=False, draw_bbox=False
    )
    obs = env.reset()

    dt = 1.0 / max(1e-6, args.fps)
    steps = 0

    try:
        while True:
            steer_correction, speed, vis = detect_yellow_lane(obs)
            action = np.array([speed, -steer_correction], dtype=np.float32)

            obs, reward, done, info = env.step(action)

            if not args.headless:
                env.render()
                cv2.imshow("Yellow Lane Detection", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
                cv2.waitKey(1)

            time.sleep(dt)

            if done:
                print("[INFO] Resetting environment")
                obs = env.reset()
                HUG_LEFT = None
                steps = 0
            steps += 1

    finally:
        env.close()
        cv2.destroyAllWindows()
        print(f"[INFO] Finished after {steps} steps")

if __name__ == "__main__":
    main()
