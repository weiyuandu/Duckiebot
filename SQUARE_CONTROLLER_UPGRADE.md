# Square Controller 状态机改进说明

## 🔄 改进概要

您的 `square_controller_node.py` 已成功从**开环时间控制**改写为**闭环状态机控制**！

## 📊 主要变化对比

### 原版本 (开环控制)
```python
# 基于时间的控制
def move_forward(self, duration):
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        self.publish_car_cmd(self._linear_velocity, 0.0)

def turn_left(self, duration):
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time:
        self.publish_car_cmd(0.0, self._angular_velocity)
```

### 新版本 (状态机 + 编码器反馈)
```python
# 基于距离/角度的精确控制
def _do_straight_side(self, side_num):
    distance_traveled = self.dist - self.segment_start_dist
    if distance_traveled >= self.side_length - self.dist_threshold:
        self._set_state(next_turn_state)  # 精确切换状态

def _do_turn(self, turn_num):
    angle_diff = self._calculate_angle_diff(self.theta_curr, target_angle)
    if abs(angle_diff) <= self.angle_threshold:
        self._set_state(next_state)  # 精确角度控制
```

## 🎯 核心改进

### 1. **状态机架构**
- **9个状态**: `STOP`, `SIDE1-4`, `TURN1-4`, `COMPLETE`
- **清晰的状态转换**: 基于实际测量值而非时间
- **LED指示**: 不同状态显示不同颜色

### 2. **精确定位系统**
- **编码器里程计**: 实时计算 (x, y, θ) 位置
- **运动学模型**: 使用 `odometry_activity.py` 的差分驱动算法
- **累计距离跟踪**: 精确测量每条边的行驶距离

### 3. **闭环控制**
- **距离控制**: 每边精确1米 (±5cm阈值)
- **角度控制**: 90°转弯 (±5°阈值)
- **实时修正**: 直线行驶时的角度修正

### 4. **数据记录**
- **rosbag记录**: 保存完整轨迹数据
- **详细日志**: 实时状态和位置信息
- **性能统计**: 总时间和最终位置

## 🔧 关键参数

| 参数 | 值 | 说明 |
|------|----|----|
| `side_length` | 1.0m | 方形边长 |
| `speed_gain` | 0.4 m/s | 线速度 |
| `steer_gain` | 2.0 rad/s | 转弯角速度 |
| `correction_gain` | 3.0 | 直线修正增益 |
| `dist_threshold` | 0.05m | 距离阈值 (5cm) |
| `angle_threshold` | 5° | 角度阈值 |

## 🚀 使用方法

### 运行状态机版本
```bash
roslaunch my_package square_controller_state_machine.launch
```

### 运行encoder集成版本
```bash
roslaunch my_package square_encoder.launch
```

## 📈 优势对比

| 特性 | 原版本 | 新版本 |
|------|--------|--------|
| **控制方式** | 开环时间 | 闭环反馈 |
| **精度** | 低 (累积误差) | 高 (实时校正) |
| **鲁棒性** | 差 (环境敏感) | 好 (自适应) |
| **可调性** | 固定参数 | 多级参数 |
| **调试性** | 有限 | 详细日志 |
| **数据记录** | 无 | 完整轨迹 |

## 🎮 状态机流程

```
STOP (红色LED)
  ↓
SIDE1 (绿色LED) → 移动1米 
  ↓
TURN1 (蓝色LED) → 左转90°
  ↓
SIDE2 (绿色LED) → 移动1米
  ↓
TURN2 (蓝色LED) → 左转90°
  ↓
SIDE3 (绿色LED) → 移动1米
  ↓
TURN3 (蓝色LED) → 左转90°
  ↓
SIDE4 (绿色LED) → 移动1米
  ↓
TURN4 (蓝色LED) → 左转90°
  ↓
COMPLETE (白色LED) → 完成并关机
```

## 🛠️ 技术实现

### 编码器集成
- 订阅左右轮编码器 `/tick` 话题
- 使用 `odometry_activity.delta_phi()` 计算轮子转角
- 使用 `odometry_activity.estimate_pose()` 计算位姿

### 状态管理
- 每个状态有独立的控制逻辑
- 使用属性标志跟踪状态进度
- 智能状态转换基于测量阈值

### 实时修正
- 直线行驶时的角度修正
- 转弯时的精确角度控制
- 可调节的控制增益

## 🎯 调试建议

1. **观察LED颜色** - 了解当前状态
2. **查看日志输出** - 监控距离和角度进度
3. **分析rosbag数据** - 验证轨迹精度
4. **调整参数** - 根据实际表现优化

这个状态机版本将为您提供更精确、更可靠的方形路径控制！ 🎉
