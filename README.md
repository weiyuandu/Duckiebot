# Duckiebot Lab

---
## Lab1 Open Loop and Feedback Control

**目标**  
- 掌握 Duckiebot 基本运动控制、状态机和编码器反馈。

### 功能模块概览

#### 1. 基础控制
- `my_publisher_node.py` / `my_subscriber_node.py`：ROS 通信示例
- `twist_control_node.py`：Twist 指令控制
- `wheel_control_node.py`：左右轮速独立控制
- `wheel_encoder_reader_node.py`：读取轮编码器数据

#### 2. 视觉感知
- `camera_reader_node.py`：检测颜色并切换 LED
- 通过摄像头读取环境信息，获取颜色方块，并在视频窗口中标记颜色和区域
- 根据识别到的颜色改变LED颜色
- 支持（绿色，黄色，蓝色），可扩展到其他颜色
(LED状态机)["assets/pics/Color_LED.drawio.svg"]


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

---

## Lab2 Lane Following and Obstacle Avoidance

**目标**  
- 让 Duckiebot 在道路中自动沿车道行驶。  
- 检测和绕行简单障碍物（可选）。  
  (等待修改）
**子任务**  
1. **车道检测（Lane Detection）**  
   - 使用摄像头图像，通过颜色过滤（白/黄车道线）+ Canny 边缘检测 + 霍夫直线变换，提取车道边界。  
   - 计算车身相对车道中心的偏移量和航向角误差。  

2. **车道跟随（Lane Following）**  
   - 将车道误差作为 PID 控制器的输入，实时调整左右轮速，实现自动居中。  

3. **障碍物检测与避让（Obstacle Detection & Avoidance）**  
   - 通过颜色过滤（红色小鸭/方块）检测障碍物区域。  
   - 使用简单的几何规则（例如区域大小/位置）判断距离。  
   - 遇到障碍时减速/停车，规划绕行路径（例如短暂偏离车道线）。  

---

## Lab3 Road Signs and Traffic Lights Recognition

**目标**  
- 识别 Duckietown 中的路牌（Stop、Turn Left、Turn Right、T-Intersection 等）。  
- 识别交通信号灯（红灯停车、绿灯通行）。  
- 将识别结果融入自动驾驶决策。  
（等待修改）
**子任务**  
1. **路牌识别（Road Sign Detection）**  
   - **传统图像处理方法**：  
     - 颜色阈值（例如蓝底、红边的标志）。  
     - 形状检测（Hough 圆检测、轮廓匹配）。  
   - **分类/识别**：  
     - 提取 ROI（感兴趣区域），用模板匹配或小型 SVM 分类器判断是哪种标志。  

2. **交通灯识别（Traffic Light Detection）**  
   - 颜色分割：提取红、绿区域。  
   - 圆形检测：用 Hough 圆检测灯泡区域。  
   - 状态判别：红 → 停车，绿 → 前进。  

3. **行为决策（Behavior Planning）**  
   - **Stop Sign**：检测到后 → 停车 3 秒 → 再启动。  
   - **Turn Signs**：在路口结合标志提示选择方向。  
   - **Traffic Light**：红灯停车等待，绿灯通行。  

**实验扩展**  
- 与车道保持（Lab2）结合：在识别路牌/红绿灯后，需要暂停或改变车道控制策略。  
- 添加更多交通规则（如优先通行、限速标志）。  

---

## 总结

- **Lab1**：掌握运动控制与 PID 调参。  
- **Lab2**：实现车道检测与障碍规避，形成基础自动驾驶能力。  
- **Lab3**：引入交通标志和信号灯的识别与决策，让 Duckiebot 行为更接近真实交通场景。


report link:  https://www.notion.so/CPS-Duckietown-Report-250192af408780bf9519c62ca5486e08?source=copy_link


