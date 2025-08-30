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
Run：docker pull duckietown/gym-duckietown  
Create C:\duckiesim on Windows host  
Run these comands:  
docker run -ti --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/c/duckiesim:/gym_duckietown--env="QT_X11_NO_MITSHM=1" --name duckie duckietown/gym-duckietown bash  
apt update && apt install -y fontconfig libglib2.0-0  
pip install stable_baselines3  
If you want to exit the container, run: exit  

If you want to save changes, run: docker commit duckie updated_duckietown_image  
You can start the simulator with：docker start -ai duckie

Test your setup with python manual_control.py


## Overview of the functionalities of different modules

### main.py
Primary Function: Main entry point and orchestration layer

- Initializes the Duckietown simulation environment with configurable parameters
- Creates and manages the lane keeping controller instance
- Handles the main simulation loop and step execution
- Processes command-line arguments for runtime configuration
- Manages environment reset and graceful shutdown
- Provides debug visualization when enabled
- Coordinates between perception, control, and simulation components

### perception.py
Primary Function: Computer vision and environmental sensing
- Image Preprocessing: Applies CLAHE (Contrast Limited Adaptive Histogram Equalization) on LAB color space to enhance contrast and handle varying lighting conditions
- Lane Detection:  
Yellow Lane Detection: Uses adaptive HSV thresholding with brightness-aware parameters for robust yellow line identification.   
White Lane Detection: Combines LAB and HSV color spaces with adaptive thresholding, restricted to right-side search area
- Robustness Features: Includes hysteresis tracking for lane disappearance and adaptive thresholding for varying lighting conditions

### controller.py
Primary Function: Decision making and control logic
- PID Control System: Implements Proportional-Integral-Derivative controller with configurable gains
- State Management: Maintains comprehensive state machine for different driving scenarios
- Turn Awareness:  
Detects when one lane marking disappears consistently
Activates turn mode with appropriate steering bias  
Implements ramp-up/down for smooth transition into and out of turns  
Uses lane width estimation for single-line navigation
- Traffic Sign Handling:  
Stop Sign State Machine: Implements 3-second stop and 2-second crossing sequence
Slow Sign Response: Reduces speed temporarily when SLOW signs are detected

### utils.py
Primary Function: Utility functions and helper operations  
- Visualization Tools: Creates debug overlay displays showing lane detection results and mask visualizations
- Geometry Utilities:  
Estimates lane center from single detected line using exponential moving average of lane width  
Updates lane width estimation with smoothing
- Conversion Functions: Handles coordinate system transformations and normalization


## 待更改区域
### 1. 基础控制
- `my_publisher_node.py` / `my_subscriber_node.py`：ROS 通信示例
- `twist_control_node.py`：Twist 指令控制
- `wheel_control_node.py`：左右轮速独立控制
- `wheel_encoder_reader_node.py`：读取轮编码器数据

#### 2. 视觉感知
- `camera_reader_node.py`：检测颜色并切换 LED
- 通过摄像头读取环境信息，获取颜色方块，并在视频窗口中标记颜色和区域
- 根据识别到的颜色改变LED颜色
- 支持（绿色，黄色，蓝色），可扩展到其他颜色
![LED状态机](assets/pics/Color_LED.drawio.svg)


#### 3. 运动控制
- `square_controller_node.py`：开环，按时间驱动正方形轨迹，LED 变色
- 使用时间控制的状态机，实现小车从起点出发，完成一个边长为1m的正方形轨迹，而后回到起点
- `square_controller_state_machine_node.py`：闭环，状态机+编码器反馈，精准正方形轨迹
- 使用状态机控制，实现上述目标
- 使用 IMU/里程计记录小车空间位置变化并记录
- 使用Python Matplotlib绘制小车从起始点到结束的空间位置

#### 4. 里程计
- `odometry_activity.py`：轮编码器运动学计算
- `odometry_node.py`：发布里程计与位姿

1. 轮子旋转角度（弧度）
左轮：
$$ \Delta\phi_L = \frac{2\pi \cdot \Delta N_L}{N_{tot}} $$
右轮：
$$ \Delta\phi_R = \frac{2\pi \cdot \Delta N_R}{N_{tot}} $$
其中 $\Delta N_L, \Delta N_R$ 为编码器tick变化，$N_{tot}$ 为每圈tick数。
2. 轮子行驶距离
左轮：
$$ \Delta s_L = R \cdot \Delta\phi_L $$
右轮：
$$ \Delta s_R = R \cdot \Delta\phi_R $$
$R$ 为轮半径。
3. 机器人位姿增量
距离增量：
$$ \Delta s = \frac{\Delta s_L + \Delta s_R}{2} $$
朝向增量：
$$ \Delta\theta = \frac{\Delta s_R - \Delta s_L}{B} $$
$B$ 为轮距（baseline）。
4. 新位姿计算
新位置：
$$ \begin{align*} x_{new} &= x_{prev} + \Delta s \cdot \cos(\theta_{prev} + \frac{\Delta\theta}{2}) \ y_{new} &= y_{prev} + \Delta s \cdot \sin(\theta_{prev} + \frac{\Delta\theta}{2}) \ \theta_{new} &= \theta_{prev} + \Delta\theta \end{align*} $$

---

## 总结

- **Lab1**：掌握运动控制与 PID 调参。  
- **Lab2**：实现车道检测与障碍规避，形成基础自动驾驶能力。  
- **Lab3**：引入交通标志和信号灯的识别与决策，让 Duckiebot 行为更接近真实交通场景。


report link:  https://www.notion.so/CPS-Duckietown-Report-250192af408780bf9519c62ca5486e08?source=copy_link


