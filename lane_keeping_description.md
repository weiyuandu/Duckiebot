# Lab 2: Lane Keeping with Turn Awareness: Algorithm Description
Implementations: lane_keeping.py (Simulator), ROS (WIP)

The implemented controller is a hybrid automaton that combines discrete modes of operation with continuous PID control dynamics. It keeps the Duckiebot centered in its lane, even during sharp turns or when one or both lane markings temporarily disappear.

## Vision Processing

Each camera frame is preprocessed using contrast-limited adaptive histogram equalization (CLAHE) and converted into HSV and LAB color spaces. Two masks are extracted:

- Yellow mask: an adaptive HSV filter sensitive to lighting changes.
- White mask: a combination of HSV and LAB brightness thresholds, restricted to the lower right part of the image, with morphological cleanup. A fallback edge-histogram method is used if no strong white lines are found.

The lane line masks are sliced horizontally, and Hough line transforms estimate line centers per slice. From these detections, the algorithm computes:
- The lane center (average of yellow and white centers if both are visible).
- The lane width (tracked with an exponential moving average).
- A curvature factor (based on differences between slice centers), used to scale steering effort.

This image processing is easily adaptable to other execution environments, such as the physical Duckiebot, where the algorithm can take the image frame data published by he Duckiebot ROS nodes as input. The processing will remain essentially the same.
## Control Logic

When a lane center estimate is available, the lateral error is normalized and fed into a PID controller with integral clamping and derivative damping. The steering command is blended with the previous output to reduce jitter. Speed is modulated inversely with steering magnitude to slow down during sharp turns.

The controller switches between four modes in operation:
1. Normal Lane Following – Default mode when at least one line is visible. Steering is PID-driven toward the lane center.
2. Turn Mode (Left/Right) – Activated if one line disappears for several consecutive frames. The car biases its trajectory toward the inside of the corner by estimating the lane center from the remaining line, adding a fixed steer bias (with a ramp at the start of the turn), and reducing speed.
3. Recovery/Search – Entered when both lines disappear for too long. The car first steers gently toward the last known side; if no lane is recovered, it executes a sweeping search with alternating steering at very low speed.
4. Fallback Exit Conditions – Modes return to Normal when lane markings are reacquired, or after a turn hold timer expires.

A diagram illustration of the operations of this algorithm is provided in this repository as a hybrid automaton model.
