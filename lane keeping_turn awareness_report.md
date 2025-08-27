# lane keeping_turn awareness_report

## **1. Purpose**

This project implements a **Lane Keeping with Turn Awareness Controller** for Duckiebot.

Its objectives are to:

- Keep the Duckiebot centered in its lane during normal driving;
- Handle sharp turns or temporary disappearance of one lane marking;
- Recover when both lane markings are lost, re-acquiring the lane automatically.

This controller enhances Duckiebot’s driving robustness beyond simple straight-line lane following, adding turn-awareness and recovery capabilities.

## **2. Core Functionality**

The code is structured into **Vision Processing**, **Control Logic**, and **Mode Switching (Hybrid Automaton)** .

### **2.1 Vision Processing**

Each camera frame is processed as follows:

1. **Preprocessing**: Contrast-Limited Adaptive Histogram Equalization (CLAHE) is applied to enhance brightness and mitigate lighting variations.
2. **Color Space Conversion**: Frames are converted to HSV and LAB color spaces for more robust lane detection.
3. **Lane Line Detection**:

    - **Yellow line**: Adaptive HSV thresholding.
    - **White line**: HSV + LAB thresholds, restricted to the lower-right region of the frame.
    - **Morphological operations**: Noise cleanup.
    - **Fallback method**: Edge histogram peak if no reliable white line is detected.
4. **Lane Center Estimation**:

    - Both lines visible → average their centers.
    - Only one line visible → estimate center using last known lane width.
    - Both lines missing → enter “lost” state.
5. **Curvature Factor**: Estimated from slice-based line slopes to scale steering effort.

---

### **2.2 Control Logic**

A **PID controller** regulates steering based on the lateral error between the vehicle’s center and the estimated lane center:

- **Proportional (P)** : corrects deviation quickly.
- **Integral (I)** : removes systematic steady-state error.
- **Derivative (D)** : damps oscillations.
- **Integral clamp**: prevents windup.
- **Output smoothing**: low-pass filtering on steering to reduce jitter.

**Speed modulation** is also applied:

- Larger steering angles → lower speed.
- Additional slowdown in Turn Mode for safety.

---

### **2.3 Mode Switching (Hybrid Automaton)**

The controller operates in three main modes:

1. **Normal Mode (Lane Following)**

    - Default mode.
    - Vehicle steers toward lane center via PID.
2. **Turn Mode (Left / Right)**

    - Triggered when one line disappears for several consecutive frames.
    - Vehicle follows the remaining line with inward steering bias.
    - Adds ramped steering bias at the beginning of turns.
    - Reduces speed until the mode’s hold time expires or both lines reappear.
3. **Recovery Mode**

    - Triggered when both lines are lost for too long.
    - Initially steers toward the last known side.
    - If unsuccessful, executes a sweeping search (alternating steering, low speed).
    - Exits when lane markings are reacquired.

---

## **3. Inputs and Outputs**

- **Input**: camera images (camera\_node/image/raw, type: sensor\_msgs/Image)
- **Output**: wheel velocity commands (wheels\_driver\_node/car\_cmd, type: duckietown\_msgs/WheelsCmdStamped)
- **Optional Output**: annotated debug images (lane\_keeping/debug\_image)

---

## **4. Parameters and Tunables**

Configurable via ROS param server or .launch files:

- **PID**: kp, ki, kd
- **Speed**: base\_speed, min\_speed, max\_speed
- **Vision**: roi\_top, slices, white\_right\_gate
- **Turn Awareness**: turn\_hysteresis, turn\_hold\_frames, turn\_steer\_bias, turn\_center\_factor, turn\_speed\_scale, turn\_ramp\_gain
- **Recovery**: lost\_frames\_to\_search, sweep\_gain, sweep\_period\_frames

## **5. Algorithm State Machine**

The **Lane Keeping with Turn Awareness Controller** is modeled as a **hybrid automaton** that switches between discrete operational modes based on lane visibility, while each mode applies continuous PID-based control.

The figure below illustrates the finite state machine (FSM) of the algorithm.

### **5.1 States**

1. **Normal Mode (Lane Following)**

    - Default state when at least one lane line is visible.
    - Lane center is estimated from visible lines (both or one).
    - Steering command:  
      ​![image](assets/image-20250827170848-yrg3yuo.png)with smoothing and curvature scaling.
    - Speed decreases slightly when steering magnitude is high.
2. **Turn Mode (LEFT / RIGHT)**

    - Activated when one lane marking (yellow or white) disappears for more than a hysteresis threshold.
    - LEFT turn mode: white missing, yellow present.
    - RIGHT turn mode: yellow missing, white present.
    - The lane center is estimated from the remaining line plus an inward bias using the lane width EMA.
    - Steering command includes an additional **steer bias with ramping**.
    - Vehicle speed is scaled down for safety.
    - Mode exits when both lines reappear or the hold timer expires.
3. **Recovery / Search Mode**

    - Triggered when both lane lines are missing for longer than lost\_frames.
    - Behavior:

      - If the loss is short, steer gently toward the last known side.
      - If the loss persists, perform sweeping search:

        ![image](assets/image-20250827170930-u7m6yys.png)at very low speed (≈0.10).
    - Mode exits immediately when one or both lines are reacquired.

### **5.2 Transitions**

- **Normal → Turn Mode**: triggered when one line is missing ≥ hysteresis frames.
- **Turn Mode → Normal**: when both lines are visible or hold timer reaches 0.
- **Normal → Recovery**: when both lines are missing ≥ lost\_frames.
- **Turn Mode → Recovery**: if the remaining line is also lost.
- **Recovery → Normal**: when any lane line is detected again

  ![image](assets/image-20250827171012-mxdx798.png)
