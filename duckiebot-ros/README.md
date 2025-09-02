## Duckiebot-ros

This project is modified from the template of duckiebot ros-template. Further information can either be find in the [offical documentations](https://docs.duckietown.com/daffy/opmanual-duckiebot/intro.html) or the github page of [duckietown](https://github.com/duckietown).

### TestNodes

- `my_publisher_node.py` / `my_subscriber_node.py`: Demonstrates basic ROS communication with publisher and subscriber examples.  
- `twist_control_node.py`: Controls robot motion using Twist velocity commands.  
- `wheel_control_node.py`: Provides independent velocity control for the left and right wheels.  
- `wheel_encoder_reader_node.py`: Reads and processes wheel encoder data for odometry input.  

### `camera_reader_node.py`

This node integrates computer vision and color recognition, receiving compressed images, converting them to HSV color space, detecting green/blue/yellow objects through thresholding, analyzing contours, dynamically controlling Duckiebot LEDs based on dominant colors, visualizing results in real time, and fully integrating with ROS topics and LED publishers.  

### `square_controller_node.py`

This node implements basic open-loop square trajectory control, moving forward and turning left in sequence, using fixed time intervals for switching motion states, while displaying different LED colors (red, green, blue, white) at each corner without relying on sensor feedback.  

### `square_controller_state_machine_node.py`

This node introduces an advanced state machine with closed-loop feedback, defining movement states through enums, integrating wheel encoder data, applying PID controllers for precise position and orientation tracking, navigating predefined waypoints, correcting errors in real time, and outputting detailed logs to visualize motion progress.  

### `odometry_activity.py`

This script implements the core odometry algorithm by processing wheel encoder displacements, applying differential-drive kinematics to estimate robot pose, providing position (x, y) and orientation updates, and exposing mathematical utilities for angle normalization and pose transformations.  

### `odometry_node.py`

This node wraps odometry into a ROS interface by subscribing to wheel encoder topics, continuously computing and publishing robot pose, loading calibration parameters (wheel radius, baseline) from the ROS parameter server, and maintaining pose consistency in the global coordinate frame.  