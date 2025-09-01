# Autonomous Driving System

---
## About
This project focuses on developing a self-driving vehicle system that combines lane-keeping, and road environment detection to mimic core autonomous driving behaviors.
An opencv-based lane detection module keeps the vehicle on course using PID control.
Road sign recognition adds to the autonomous capabilities of the vehicle, allowing it
to respond appropriately to road signage. The perception modules feed into a decision layer that keeps track of various states of the vehicle to plan safe and efficient
trajectories. Finally, a control layer translates high-level decisions into low-level motor commands for steering and speed. The autonomous driving system is written in
python, and tested in the gym-duckietown simulator, ensuring maximum similarities
with a real Duckietown environment. This project provides hands-on experience with
computer vision and cyber-physical systems, building a foundation for more advanced
autonomous vehicle research.

## Setup
One-time setup:
Windows OS should install WSL, install docker inside WSL   
Run：
```bash
docker pull duckietown/gym-duckietown
```
Create 
```bash
C:\duckiesim on Windows host
```
Run these comands:  
```bash
docker run -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/c/duckiesim:/gym_duckietown--env="QT_X11_NO_MITSHM=1" --name duckie duckietown/gym-duckietown bash  
apt update && apt install -y fontconfig libglib2.0-0  
pip install stable_baselines3
```
If you want to exit the container, run:
```bash
exit
```

If you want to save changes, run: 
```bash
docker commit duckie updated_duckietown_image
```
You can start the simulator with：
```bash
docker start -ai duckie
```

Test your setup with python `manual_control.py`

### Duckiebot-ros
If you want to run the code from the `duckiebot-ros` module, you will need:  
- A properly installed and calibrated Duckiebot **DB21J/M**  
- A local environment with **Duckietown Shell** configured  

For detailed instructions, please refer to the official Duckietown documentation.  

---
If the above conditions are met, you can proceed with the following steps:

1. Clone this repository:
```bash
git clone "<repository_url>"
```
2.	Navigate into the duckiebot-ros folder:
```bash
  cd duckiebot-ros
```
3. Build the Docker image using Duckietown Shell:
```bash
dts devel build -f
```
4.Run the desired node on your Duckiebot:
```bash
dts devel run -R <duckiebot_name> -L <node_name>
```
Replace `<duckiebot_name>` with the name of your Duckiebot and `<node_name>` with the name of the node you want to run.

5.For more details on dts devel run usage, you can check the help manual by running:
```bahs
dts devel run --help
```
## Overview of the functionalities of different modules

### autonomous_driving/main.py
Primary Function: Main entry point and orchestration layer

- Initializes the Duckietown simulation environment with configurable parameters
- Creates and manages the lane keeping controller instance
- Handles the main simulation loop and step execution
- Processes command-line arguments for runtime configuration
- Manages environment reset and graceful shutdown
- Provides debug visualization when enabled
- Coordinates between perception, control, and simulation components

### autonomous_driving/perception.py
Primary Function: Computer vision and environmental sensing
- Image Preprocessing: Convert image to HSV color space
- Lane Detection:  
Yellow Lane Detection: Use HSV thresholding with brightness-aware parameters for robust yellow line identification.   
White Lane Detection: Use HSV color space, restricted to right-side search area
- Robustness Features: Includes hysteresis tracking for lane disappearance and adaptive thresholding for varying lighting conditions

### autonomous_driving/controller.py
Primary Function: Decision making and control logic
- The lane marking masks are multiplied by a pre-set weighting matrix, giving a
value for each pixel in the masks
- The weighted results are converted into a steering vector and passed to the wheels.
- Essentially, this is a variation of the ”P” portion of PID Control

### autonomous_driving/utils.py
Primary Function: Utility functions and helper operations  
- Visualization Tools: Creates debug overlay displays showing lane detection results and mask visualizations
- Geometry Utilities:  
Estimates lane center from single detected line using exponential moving average of lane width  
Updates lane width estimation with smoothing
- Conversion Functions: Handles coordinate system transformations and normalization

### autonomous_driving/pid_control_standalone.py
Primary function: An alternate approach using full PID-based control and turn-awareness modifications. It is a standalone executable, and can be run via `python pid_control_standalone.py --env-name <map name>` 


## Duckiebot-ros

### `duckiebot-ros/packages/my_package/src`

- `my_publisher_node.py` / `my_subscriber_node.py`: Demonstrates basic ROS communication with publisher and subscriber examples.  
- `twist_control_node.py`: Controls robot motion using Twist velocity commands.  
- `wheel_control_node.py`: Provides independent velocity control for the left and right wheels.  
- `wheel_encoder_reader_node.py`: Reads and processes wheel encoder data for odometry input.  

### `duckiebot-ros/packages/my_package/src/camera_reader_node.py`

This node integrates computer vision and color recognition, receiving compressed images, converting them to HSV color space, detecting green/blue/yellow objects through thresholding, analyzing contours, dynamically controlling Duckiebot LEDs based on dominant colors, visualizing results in real time, and fully integrating with ROS topics and LED publishers.  

### `duckiebot-ros/packages/my_package/src/square_controller_node.py`

This node implements basic open-loop square trajectory control, moving forward and turning left in sequence, using fixed time intervals for switching motion states, while displaying different LED colors (red, green, blue, white) at each corner without relying on sensor feedback.  

### `duckiebot-ros/packages/my_package/src/square_controller_state_machine_node.py`

This node introduces an advanced state machine with closed-loop feedback, defining movement states through enums, integrating wheel encoder data, applying PID controllers for precise position and orientation tracking, navigating predefined waypoints, correcting errors in real time, and outputting detailed logs to visualize motion progress.  

### `duckiebot-ros/packages/my_package/src/odometry_activity.py`

This script implements the core odometry algorithm by processing wheel encoder displacements, applying differential-drive kinematics to estimate robot pose, providing position (x, y) and orientation updates, and exposing mathematical utilities for angle normalization and pose transformations.  

### `duckiebot-ros/packages/my_package/src/odometry_node.py`

This node wraps odometry into a ROS interface by subscribing to wheel encoder topics, continuously computing and publishing robot pose, loading calibration parameters (wheel radius, baseline) from the ROS parameter server, and maintaining pose consistency in the global coordinate frame.  

Presentation link:

Demo link:

Report link:  https://www.notion.so/CPS-Duckietown-Report-250192af408780bf9519c62ca5486e08?source=copy_link


