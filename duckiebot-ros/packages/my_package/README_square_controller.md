# Square Controller State Machine with Encoder Feedback

这个改进的square controller使用状态机和轮式编码器反馈来精确控制Duckiebot行驶1m×1m的正方形轨迹。

## 主要改进

### 1. **状态机设计**
- `INIT`: 初始化状态
- `WAIT_FOR_ENCODERS`: 等待编码器数据
- `SET_CORNER_LED`: 设置每个角落的LED颜色
- `MOVING_FORWARD`: 基于位置反馈的前进控制
- `TURNING_LEFT`: 基于角度反馈的转向控制
- `COMPLETED`: 完成状态
- `SHUTDOWN`: 关闭状态

### 2. **基于微分方程的控制**
使用差动驱动运动学模型：
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = ω
```

### 3. **编码器反馈控制**
- **位置控制**: 使用PID控制到达目标位置
- **角度控制**: 使用角度反馈精确转向90°
- **实时位姿估计**: 基于轮式编码器的odometry

## 文件结构

```
src/
├── square_controller_state_machine_node.py  # 主控制节点
├── odometry_activity.py                     # 运动学计算
└── odometry_node.py                        # 独立的odometry节点

launchers/
├── square-controller-state-machine.sh      # 启动脚本
└── odometry.sh                             # Odometry节点启动脚本
```

## 使用方法

### 1. 启动Square Controller
```bash
./launchers/square-controller-state-machine.sh
```

### 2. 启动独立的Odometry Node
```bash
./launchers/odometry.sh
```

### 3. 直接运行节点
```bash
rosrun my_package square_controller_state_machine_node.py
```

## 控制算法

### 位置控制器
```python
def calculate_position_control(self, target_x, target_y):
    dx = target_x - self.x_curr
    dy = target_y - self.y_curr
    distance_error = sqrt(dx² + dy²)
    
    desired_theta = atan2(dy, dx)
    angle_error = target_angle - current_angle
    
    v = speed_gain if distance_error > threshold else 0
    omega = steer_gain * angle_error
```

### 角度控制器
```python
def calculate_orientation_control(self, target_theta):
    angle_error = target_theta - current_theta
    v = 0  # 纯旋转
    omega = steer_gain * angle_error
```

## 📊 正方形轨迹

| 阶段 | 目标位置 | 目标角度 | LED颜色 |
|------|----------|----------|---------|
| 开始 | (0, 0)   | 0°       | 红色    |
| 角1  | (1, 0)   | 90°      | 绿色    |
| 角2  | (1, 1)   | 180°     | 蓝色    |
| 角3  | (0, 1)   | 270°     | 白色    |
| 结束 | (0, 0)   | 0°       | 关闭    |

## 参数配置

### 运动学参数 (自动从ROS参数加载)
- `wheel_radius`: 0.0318m (默认)
- `baseline`: 0.1m (默认)

### 控制参数
- `speed_gain`: 0.4 (线速度增益)
- `steer_gain`: 5.5 (角速度增益)
- `position_threshold`: 0.05m (位置精度)
- `angle_threshold`: 5° (角度精度)

## 调试信息

节点运行时会输出详细的调试信息：
```
[INFO] Current pose: x=0.123, y=0.456, θ=45.0°
[INFO] Progress: pos=(0.500, 0.200), θ=15.0°, dist_err=0.050m, ang_err=2.5°
[INFO] Reached position (1.00, 0.00)
[INFO] Turn progress: current_θ=85.0°, target_θ=90.0°, ang_err=5.0°
```

## 技术特点

1. **精确控制**: 基于编码器反馈，比时间控制更精确
2. **鲁棒性**: 自动错误恢复和异常处理
3. **模块化**: 状态机设计便于扩展和修改
4. **实时反馈**: 10Hz控制循环提供流畅控制
5. **兼容性**: 完全兼容Duckietown DTROS框架

## 故障排除

### 常见问题
1. **编码器未初始化**: 节点会等待编码器数据
2. **位置偏差过大**: 检查轮子校准参数
3. **转向不准确**: 调整steer_gain参数
4. **LED不工作**: 检查LED服务是否运行

### 调试建议
1. 监控odometry话题: `rostopic echo /{vehicle_name}/odometry_node/pose`
2. 检查编码器数据: `rostopic echo /{vehicle_name}/left_wheel_encoder_node/tick`
3. 调整控制增益以获得最佳性能

这个改进版本提供了更精确、更可靠的正方形轨迹控制，同时保持了与现有Duckiebot系统的完全兼容性。
