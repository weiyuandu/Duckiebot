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

  - Yellow Lane Detection: Use HSV thresholding with brightness-aware parameters for robust yellow line identification.   

  - White Lane Detection: Use HSV color space, restricted to right-side search area

- Robustness Features: Includes hysteresis tracking for lane disappearance and adaptive thresholding for varying lighting conditions



### autonomous_driving/controller.py

Primary Function: Decision making and control logic

- The lane marking masks are multiplied by a pre-set weighting matrix, giving a value for each pixel in the masks

- The weighted results are converted into a steering vector and passed to the wheels.

- Essentially, this is a variation of the ”P” portion of PID Control



### autonomous_driving/utils.py

Primary Function: Utility functions and helper operations  

- Visualization Tools: Creates debug overlay displays showing lane detection results and mask visualizations

- Geometry Utilities:  

  - Estimates lane center from single detected line using exponential moving average of lane width  

  - Updates lane width estimation with smoothing

- Conversion Functions: Handles coordinate system transformations and normalization



### autonomous_driving/pid_control_standalone.py

Primary function: An alternate approach using full PID-based control and turn-awareness modifications. It is a standalone executable, and can be run via `python pid\_control\_standalone.py --env-name <map name>` 



