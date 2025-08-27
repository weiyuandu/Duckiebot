# Duckiebot Lab

---

## Lab1 Open Loop and PID Control

**目标**  
- 熟悉 Duckiebot 的基本运动控制。  
- 实现基于 PID 的速度和转向控制。  

**任务**  
1. **Open Loop Control**  
   - 通过给定固定的转速指令，让 Duckiebot 前进、后退、旋转。  
   - 观察实际运行轨迹与预期的差异。  

2. **PID Control**  
   - 使用 IMU/里程计反馈，调试 PID 控制器，实现稳定的速度与角度控制。  
   - 分别测试 P、PI、PID 的效果差异。  

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
slides link:  https://tumde-my.sharepoint.com/:p:/g/personal/go69zug_tum_de/EYYgkq2WLWFNoQ0QcJK4PFEBbK8xNNI5GurfaGTvWmInrg

