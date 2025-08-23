#!/usr/bin/env python3
#
# ROS wrapper for the gym-based lane keeping controller:
# - Subscribes to camera_node/image/raw
# - Publishes wheels_driver_node/car_cmd (WheelsCmdStamped)
# - Optionally publishes an annotated debug image
#

import os
import cv2
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from duckietown_msgs.msg import WheelsCmdStamped


# =============================== Configs =================================== #

@dataclass
class PIDGains:
    kp: float = 0.010
    ki: float = 0.0003
    kd: float = 0.003
    integral_clamp: float = 5000.0


@dataclass
class SpeedCfg:
    base: float = 0.35
    min_: float = 0.08
    max_: float = 0.55


@dataclass
class VisionCfg:
    roi_top: float = 0.35           # analyze lower 65% of the image
    slices: int = 6
    center_ema_alpha: float = 0.35  # smoothing on detected lane center (px)
    steer_blend: float = 0.70       # low-pass on steering output
    deadband_px: int = 8
    min_correction: float = 0.03
    max_steer: float = 0.75
    white_right_gate: float = 0.58  # fraction of width (search white to right of this)


@dataclass
class RecoveryCfg:
    lost_frames_to_search: int = 10  # frames before entering active search
    sweep_gain: float = 0.55         # steer amplitude during search
    sweep_period_frames: int = 12    # ~0.6s at 20 FPS


@dataclass
class TurnCfg:
    # Enter turn mode if one side is missing this many consecutive frames
    disappear_hysteresis: int = 3
    # Hold turn mode at least this many frames once engaged
    hold_frames: int = 30
    # How strongly to bias steering during turn mode (added to PID)
    steer_bias: float = 0.40
    # When only one line is visible, estimate lane center at
    #   x_line +/- (lane_width_ema * single_line_center_factor)
    single_line_center_factor: float = 0.30  # < 0.50 hugs inside of turn
    # Extra speed reduction factor while in turn mode (multiply base speed)
    speed_scale: float = 0.70
    # Optional ramp factor: up to +ramp_gain at the beginning of turn mode
    ramp_gain: float = 0.40  # 0.40 => up to +40% stronger at the start


# =============================== Controller ================================= #

class LaneKeeperPIDTurnAware:
    """
    PID lane keeping centered on lane, with turn-awareness and robust vision.
    """
    def __init__(
        self,
        pid: PIDGains,
        speed: SpeedCfg,
        vis: VisionCfg,
        recov: RecoveryCfg,
        turn: TurnCfg,
        use_white_fallback: bool = True,
        show_masks: bool = False,
    ):
        self.pid = pid
        self.speed = speed
        self.vis = vis
        self.recov = recov
        self.turn = turn
        self.use_white_fallback = use_white_fallback
        self.show_masks = show_masks

        # state
        self.integral = 0.0
        self.last_error = 0.0
        self.last_steer = 0.0
        self.last_center_ema = None
        self.last_lane_width_ema = None
        self.last_line_side = 0
        self.lost_counter = 0
        self.search_phase = 1
        self.search_phase_frames = 0

        # turn awareness state
        self.turn_mode = None            # None | "left" | "right"
        self.turn_hold = 0               # frames left to hold current turn mode
        self.miss_yellow_frames = 0
        self.miss_white_frames = 0

    # ------------------------------ Vision -------------------------------- #

    @staticmethod
    def _preprocess(rgb):
        """CLAHE on LAB L; return HSV and LAB from enhanced image."""
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        lab0 = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        L, A, B = cv2.split(lab0)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        Lc = clahe.apply(L)
        lab_enh = cv2.merge([Lc, A, B])
        bgr_enh = cv2.cvtColor(lab_enh, cv2.COLOR_LAB2BGR)
        hsv = cv2.cvtColor(bgr_enh, cv2.COLOR_BGR2HSV)
        lab = cv2.cvtColor(bgr_enh, cv2.COLOR_BGR2LAB)
        return hsv, lab

    @staticmethod
    def _adaptive_yellow_mask(hsv):
        """Brightness-aware yellow mask (HSV)."""
        v = hsv[..., 2]
        v_med = float(np.median(v))
        s_lo = max(60, int(0.9 * (80 - (v_med - 128) * 0.15)))
        v_lo = max(60, int(0.9 * (80 - (v_med - 128) * 0.10)))
        lower = np.array([15, s_lo, v_lo], dtype=np.uint8)
        upper = np.array([40, 255, 255], dtype=np.uint8)
        return cv2.inRange(hsv, lower, upper)

    def _white_mask(self, hsv, lab, right_gate_x=None, y1=None, y2=None):
        """
        LAB+HSV + adaptive-threshold white mask restricted to lower ROI and right half.
        """
        h, w = hsv.shape[:2]
        L, A, B = cv2.split(lab)

        # HSV gate: bright & low saturation (loosen V to 185 if needed)
        hsv_gate = cv2.inRange(hsv, (0, 0, 200), (180, 80, 255))

        # LAB gates: near-neutral chroma + bright L
        cA = np.full_like(A, 128, dtype=A.dtype)
        cB = np.full_like(B, 128, dtype=B.dtype)
        a_dev = cv2.absdiff(A, cA)
        b_dev = cv2.absdiff(B, cB)
        chroma_ok = cv2.threshold(a_dev + b_dev, 25, 255, cv2.THRESH_BINARY_INV)[1]
        L_hi = cv2.threshold(L, 190, 255, cv2.THRESH_BINARY)[1]

        # Adaptive L (handles uneven lighting)
        L_adapt = cv2.adaptiveThreshold(
            L, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 15, -7
        )

        white = cv2.bitwise_and(hsv_gate, L_hi)
        white = cv2.bitwise_or(white, cv2.bitwise_and(chroma_ok, L_adapt))

        # Lower-ROI
        if y1 is None: y1 = int(h * self.vis.roi_top)
        if y2 is None: y2 = h
        vmask = np.zeros_like(white); vmask[y1:y2, :] = 255
        white = cv2.bitwise_and(white, vmask)

        # Right-half gating
        if right_gate_x is None:
            right_gate_x = int(w * self.vis.white_right_gate)
        rmask = np.zeros_like(white); rmask[:, right_gate_x:] = 255
        white = cv2.bitwise_and(white, rmask)

        # Morph cleanup
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
        white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1)
        return white

    def _slice_centers(self, bin_mask, y1, y2, vis=None, color_bgr=(0, 255, 255)):
        """Per-slice Hough to get x-centers. Returns (centers, weights)."""
        h, w = bin_mask.shape
        centers = []
        weights = np.linspace(0.6, 1.0, self.vis.slices)
        for i, weight in enumerate(weights):
            ys = y1 + i * (y2 - y1) // self.vis.slices
            ye = y1 + (i + 1) * (y2 - y1) // self.vis.slices
            roi = bin_mask[ys:ye, :]
            roi = cv2.GaussianBlur(roi, (5, 5), 0)
            roi = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1)
            edges = cv2.Canny(roi, 50, 150)
            lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=12, minLineLength=6, maxLineGap=6)

            xs = []
            if lines is not None:
                for x1l, y1l, x2l, y2l in lines[:, 0]:
                    dx = (x2l - x1l); dy = (y2l - y1l)
                    slope = 1e6 if abs(dx) < 1 else dy / dx
                    if abs(slope) < 0.25:  # ignore near-horizontal clutter
                        continue
                    xs.append((x1l + x2l) * 0.5)
                    if vis is not None:
                        cv2.line(vis, (x1l, ys + y1l), (x2l, ys + y2l), color_bgr, 1)
            if xs:
                centers.append(weight * float(np.mean(xs)))
        return centers, weights

    @staticmethod
    def _edge_histogram_peak(bin_mask, y1, y2):
        """Rightmost 40% edge peak as a coarse white fallback."""
        h, w = bin_mask.shape
        xs = slice(int(w * 0.60), w)
        roi = bin_mask[y1:y2, xs]
        edges = cv2.Canny(roi, 50, 150)
        hist = edges.sum(axis=0)
        if hist.size == 0 or hist.max() < 200:
            return None
        peak = int(hist.argmax())
        return int(w * 0.60) + peak

    def _update_lane_width_ema(self, yellow_x, white_x):
        width = abs(white_x - yellow_x)
        if self.last_lane_width_ema is None:
            self.last_lane_width_ema = width
        else:
            self.last_lane_width_ema = 0.8 * self.last_lane_width_ema + 0.2 * width

    def _estimate_center_from_single(self, x_line, which, w, factor=None):
        """Estimate lane center given one line using lane-width EMA and factor."""
        if self.last_lane_width_ema is None:
            self.last_lane_width_ema = 0.25 * w  # bootstrap
        f = self.turn.single_line_center_factor if factor is None else factor
        if which == "yellow":  # white is to the right
            return x_line + f * self.last_lane_width_ema
        else:                  # which == "white"
            return x_line - f * self.last_lane_width_ema

    # ------------------------------- Step --------------------------------- #

    def step(self, obs):
        """Compute (steer, speed, debug_vis) for a single observation."""
        hsv, lab = self._preprocess(obs)
        h, w = hsv.shape[:2]
        y1 = int(h * self.vis.roi_top); y2 = h
        vis = obs.copy()

        # Masks
        mask_y = self._adaptive_yellow_mask(hsv)
        mask_w = self._white_mask(hsv, lab, right_gate_x=int(w * self.vis.white_right_gate), y1=y1, y2=y2)

        # Centers
        y_centers, weights = self._slice_centers(mask_y, y1, y2, vis, (0, 255, 255))
        w_centers, _ = self._slice_centers(mask_w, y1, y2, vis, (255, 255, 255))
        if self.use_white_fallback and not w_centers:
            peak_x = self._edge_histogram_peak(mask_w, y1, y2)
            if peak_x is not None:
                w_centers = [float(peak_x)]

        if self.show_masks:
            cv2.imshow("mask_y", mask_y)
            cv2.imshow("mask_w", mask_w)

        got_y = len(y_centers) > 0
        got_w = len(w_centers) > 0

        # Track disappearances with hysteresis
        self.miss_yellow_frames = 0 if got_y else (self.miss_yellow_frames + 1)
        self.miss_white_frames  = 0 if got_w else (self.miss_white_frames  + 1)

        yellow_x = float(np.sum(y_centers) / max(1e-6, np.sum(weights))) if got_y else None
        white_x  = float(np.sum(w_centers) / max(1e-6, np.sum(weights))) if got_w else None

        # Compute lane center target (pre-turn-mode)
        lane_center_x = None
        if got_y and got_w:
            lane_center_x = 0.5 * (yellow_x + white_x)
            self._update_lane_width_ema(yellow_x, white_x)
            self.lost_counter = 0
        elif got_y:
            lane_center_x = self._estimate_center_from_single(yellow_x, "yellow", w, factor=0.50)
            self.lost_counter = 0
        elif got_w:
            lane_center_x = self._estimate_center_from_single(white_x, "white", w, factor=0.50)
            self.lost_counter = 0
        else:
            self.lost_counter += 1

        # Enter/refresh turn mode based on disappearances
        if self.turn_mode is None:
            if self.miss_yellow_frames >= self.turn.disappear_hysteresis and got_w:
                # Yellow gone, white present -> RIGHT turn
                self.turn_mode = "right"
                self.turn_hold = self.turn.hold_frames
            elif self.miss_white_frames >= self.turn.disappear_hysteresis and got_y:
                # White gone, yellow present -> LEFT turn
                self.turn_mode = "left"
                self.turn_hold = self.turn.hold_frames
        else:
            # refresh / count down
            self.turn_hold -= 1
            if self.turn_hold <= 0 or (got_y and got_w):
                # exit turn mode if time elapsed or both lines back
                self.turn_mode = None
                self.turn_hold = 0

        # If in turn mode, bias target & steering toward inside of corner
        steer_bias = 0.0
        if self.turn_mode == "right":
            # only have white reliably; hug it (center closer than half-width)
            if got_w:
                lane_center_x = self._estimate_center_from_single(
                    white_x, "white", w, factor=self.turn.single_line_center_factor)
            # ramp the bias stronger at start of turn, taper as hold decreases
            ramp = 1.0 + self.turn.ramp_gain * (self.turn_hold / max(1, self.turn.hold_frames))
            steer_bias = -self.turn.steer_bias * ramp  # negative steer is right (we invert later for env)
        elif self.turn_mode == "left":
            if got_y:
                lane_center_x = self._estimate_center_from_single(
                    yellow_x, "yellow", w, factor=self.turn.single_line_center_factor)
            ramp = 1.0 + self.turn.ramp_gain * (self.turn_hold / max(1, self.turn.hold_frames))
            steer_bias = +self.turn.steer_bias * ramp

        # curvature factor from slice deltas (whichever side we have)
        curv_factor = 1.0
        seq = y_centers if got_y else (w_centers if got_w else [])
        if len(seq) > 1:
            diffs = np.diff(seq)
            avg_slope_abs = float(np.mean(np.abs(diffs)))
            curv_factor += 5.0 * avg_slope_abs / (w / 2)
            curv_factor = min(curv_factor, 5.0)

        # ------------------------------ Control ---------------------------- #
        in_recovery = self.lost_counter >= self.recov.lost_frames_to_search

        if (lane_center_x is not None) and not in_recovery:
            # EMA smoothing on target center
            if self.last_center_ema is None:
                self.last_center_ema = lane_center_x
            self.last_center_ema = (
                self.vis.center_ema_alpha * lane_center_x +
                (1 - self.vis.center_ema_alpha) * self.last_center_ema
            )

            error_px = self.last_center_ema - (w / 2)
            self.last_line_side = -1 if error_px < 0 else 1

            if abs(error_px) < self.vis.deadband_px:
                steer = np.sign(error_px) * self.vis.min_correction * curv_factor
                self.integral *= 0.95
            else:
                error_norm = error_px / (w / 2)  # normalize to [-1,1]
                self.integral += error_norm
                self.integral = np.clip(self.integral, -self.pid.integral_clamp, self.pid.integral_clamp)
                derivative = (error_norm - self.last_error)
                steer = (self.pid.kp * error_norm +
                         self.pid.ki * self.integral +
                         self.pid.kd * derivative) * curv_factor
                self.last_error = error_norm

            # Apply turn steer bias if any, then clamp
            steer += steer_bias
            steer = float(np.clip(steer, -self.vis.max_steer, self.vis.max_steer))

            # Speed modulation by steer + extra slowdown in turn mode
            speed_cmd = self.speed.base * (1.1 - 0.6 * min(1.0, abs(steer) / self.vis.max_steer))
            if self.turn_mode is not None:
                speed_cmd *= self.turn.speed_scale
            speed_cmd = float(np.clip(speed_cmd, self.speed.min_, self.speed.max_))
        else:
            # Recovery behavior
            if self.lost_counter < self.recov.lost_frames_to_search:
                steer = 0.35 * self.vis.max_steer * (self.last_line_side if self.last_line_side != 0 else 1)
                speed_cmd = max(self.speed.min_ * 0.7, 0.06)
            else:
                self.search_phase_frames += 1
                if self.search_phase_frames >= self.recov.sweep_period_frames:
                    self.search_phase *= -1
                    self.search_phase_frames = 0
                steer = self.search_phase * self.recov.sweep_gain * self.vis.max_steer
                speed_cmd = 0.10
            self.integral *= 0.90

        # Low-pass filter on steer to reduce jitter
        steer = self.vis.steer_blend * self.last_steer + (1 - self.vis.steer_blend) * steer
        self.last_steer = steer

        # ------------------------------ Viz -------------------------------- #
        if lane_center_x is not None:
            x_draw = int(self.last_center_ema if self.last_center_ema is not None else lane_center_x)
            cv2.line(vis, (x_draw, y1), (x_draw, h - 1), (0, 0, 255), 2)

        if got_y:
            cv2.circle(vis, (int(yellow_x), h - 6), 4, (0, 255, 255), -1)
        if got_w:
            cv2.circle(vis, (int(white_x),  h - 6), 4, (255, 255, 255), -1)

        steer_px = int(steer * w / 2)
        cv2.arrowedLine(vis, (w // 2, h - 1), (w // 2 + steer_px, h - 60), (255, 0, 0), 3)
        hud = f"steer={steer:+.3f} speed={speed_cmd:.2f} lost={self.lost_counter}"
        if self.turn_mode:
            hud += f" turn={self.turn_mode}:{self.turn_hold}"
        if self.last_lane_width_ema is not None:
            hud += f" width_ema={self.last_lane_width_ema:.1f}"
        cv2.putText(vis, hud, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (40, 220, 40), 1, cv2.LINE_AA)

        return float(steer), float(speed_cmd), vis


class LaneKeepingNode(object):
    def __init__(self):
        # --- Parameters (ROS param server) ---
        # Names are consistent with your argparse options when possible.
        # All of these can be set in a .launch file or via rosparam.
        self.camera_topic = rospy.get_param("~camera_topic", "camera_node/image/raw")
        self.cmd_topic = rospy.get_param("~cmd_topic", "wheels_driver_node/car_cmd")
        self.debug_topic = rospy.get_param("~debug_topic", "lane_keeping/debug_image")
        self.publish_debug_image = rospy.get_param("~publish_debug_image", True)

        # Controller config (PID + speed + vision + recovery + turn-awareness)
        pid = PIDGains(
            kp=rospy.get_param("~kp", PIDGains.kp),
            ki=rospy.get_param("~ki", PIDGains.ki),
            kd=rospy.get_param("~kd", PIDGains.kd),
            integral_clamp=rospy.get_param("~integral_clamp", PIDGains.integral_clamp),
        )
        speed = SpeedCfg(
            base=rospy.get_param("~base_speed", SpeedCfg.base),
            min_=rospy.get_param("~min_speed", SpeedCfg.min_),
            max_=rospy.get_param("~max_speed", SpeedCfg.max_),
        )
        vis = VisionCfg(
            roi_top=rospy.get_param("~roi_top", VisionCfg.roi_top),
            slices=rospy.get_param("~slices", VisionCfg.slices),
            center_ema_alpha=rospy.get_param("~center_ema_alpha", VisionCfg.center_ema_alpha),
            steer_blend=rospy.get_param("~steer_blend", VisionCfg.steer_blend),
            deadband_px=rospy.get_param("~deadband_px", VisionCfg.deadband_px),
            min_correction=rospy.get_param("~min_correction", VisionCfg.min_correction),
            max_steer=rospy.get_param("~max_steer", VisionCfg.max_steer),
            white_right_gate=rospy.get_param("~white_right_gate", VisionCfg.white_right_gate),
        )
        recov = RecoveryCfg(
            lost_frames_to_search=rospy.get_param("~lost_frames_to_search", RecoveryCfg.lost_frames_to_search),
            sweep_gain=rospy.get_param("~sweep_gain", RecoveryCfg.sweep_gain),
            sweep_period_frames=rospy.get_param("~sweep_period_frames", RecoveryCfg.sweep_period_frames),
        )
        turn = TurnCfg(
            disappear_hysteresis=rospy.get_param("~turn_hysteresis", TurnCfg.disappear_hysteresis),
            hold_frames=rospy.get_param("~turn_hold_frames", TurnCfg.hold_frames),
            steer_bias=rospy.get_param("~turn_steer_bias", TurnCfg.steer_bias),
            single_line_center_factor=rospy.get_param("~turn_center_factor", TurnCfg.single_line_center_factor),
            speed_scale=rospy.get_param("~turn_speed_scale", TurnCfg.speed_scale),
            ramp_gain=rospy.get_param("~turn_ramp_gain", TurnCfg.ramp_gain),
        )

        use_white_fallback = rospy.get_param("~use_white_fallback", True)
        show_masks = rospy.get_param("~show_masks", False)

        # Create controller
        self.controller = LaneKeeperPIDTurnAware(
            pid, speed, vis, recov, turn,
            use_white_fallback=use_white_fallback,
            show_masks=show_masks
        )

        # I/O
        self.bridge = CvBridge()
        self.pub_cmd = rospy.Publisher(self.cmd_topic, WheelsCmdStamped, queue_size=1)
        self.pub_debug = rospy.Publisher(self.debug_topic, Image, queue_size=1) if self.publish_debug_image else None
        self.sub_cam = rospy.Subscriber(self.camera_topic, Image, self.image_cb, queue_size=1, buff_size=2**22)

        # Execution rate hint (camera callback drives processing; this only affects sleep in visualization)
        self.target_hz = rospy.get_param("~target_hz", 20.0)

        # Scaling & safety
        self.wheel_gain = rospy.get_param("~wheel_gain", 1.0)      # scales overall speed
        self.steer_gain = rospy.get_param("~steer_gain", 1.0)      # scales steering contribution
        self.wheel_limit = rospy.get_param("~wheel_limit", 1.0)    # clamp wheel commands [-wheel_limit, +wheel_limit]

        # Optional imshow for on-laptop debugging
        self.use_imshow = rospy.get_param("~use_imshow", False)

        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("lane_keeping_pid_turnaware_node: initialized")

    # --- Camera callback drives one control step ---
    def image_cb(self, msg: Image):
        try:
            # Try RGB first, then BGR
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                rgb = frame
            except CvBridgeError:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        except CvBridgeError as e:
            rospy.logwarn("CvBridge error: %s", str(e))
            return

        # Run your controller
        steer, speed_cmd, vis = self.controller.step(rgb)

        # Map (speed, steer) -> wheel velocities
        # NOTE: In sim you did action=[speed, -steer].
        # Here we convert to differential wheel speeds; adjust gains if needed.
        left = (speed_cmd * self.wheel_gain) - (steer * self.steer_gain)
        right = (speed_cmd * self.wheel_gain) + (steer * self.steer_gain)

        # Clamp for safety
        left = float(np.clip(left, -self.wheel_limit, +self.wheel_limit))
        right = float(np.clip(right, -self.wheel_limit, +self.wheel_limit))

        # Publish wheel command
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = left
        cmd.vel_right = right
        self.pub_cmd.publish(cmd)

        # Optional visualization
        if self.pub_debug is not None:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(vis, cv2.COLOR_RGB2BGR), encoding="bgr8")
                dbg_msg.header.stamp = msg.header.stamp
                self.pub_debug.publish(dbg_msg)
            except CvBridgeError as e:
                rospy.logwarn("Debug image publish error: %s", str(e))

        if self.use_imshow:
            cv2.imshow("Lane Keeping (TURN-AWARE)", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

    def _on_shutdown(self):
        # Stop the robot on shutdown
        try:
            cmd = WheelsCmdStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.vel_left = 0.0
            cmd.vel_right = 0.0
            self.pub_cmd.publish(cmd)
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rospy.loginfo("lane_keeping_pid_turnaware_node: stopped")

def main():
    rospy.init_node("lane_keeping_pid_turnaware_node", anonymous=False)
    node = LaneKeepingNode()
    rospy.spin()

if __name__ == "__main__":
    main()
