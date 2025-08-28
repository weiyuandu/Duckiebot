# Duckiebot Square Controller State Machines

这个包包含了两个基于状态机的方形路径控制器，都使用编码器反馈进行精确定位。

## 文件结构

### 核心文件
- `odometry_activity.py` - 里程计算法和工具函数
- `encoder_pos_node.py` - 原始编码器节点，添加了简化方形路径模式
- `square_controller_state_machine.py` - 专用的方形路径状态机控制器

### 启动文件
- `square_encoder.launch` - 启动encoder_pos_node的方形模式
- `square_state_machine.launch` - 启动专用的状态机控制器

## 两种控制器的区别

### 1. encoder_pos_node.py (方形模式)
**特点:**
- 基于原有的复杂系统，添加了简化的方形路径模式
- 包含原始的STATE2/3/4复杂路径和新的SQUARE状态
- 通过`square_mode`参数选择模式

**状态:**
- `SQUARE_SIDE1-4`: 四条边的直线行驶
- `SQUARE_TURN1-4`: 四个90度左转
- `SQUARE_COMPLETE`: 完成状态

### 2. square_controller_state_machine.py (专用)
**特点:**
- 完全独立的方形路径控制器
- 更简洁的代码结构
- 专为1m×1m方形路径设计
- 支持LED颜色指示和详细日志

**状态:**
- `SIDE1-4`: 边的直线行驶 (绿色LED)
- `TURN1-4`: 转弯 (蓝色LED)
- `COMPLETE`: 完成 (白色LED)

## 使用方法

### 方法1: 使用修改后的encoder_pos_node
```bash
# 启动方形模式
roslaunch my_package square_encoder.launch

# 或者手动设置参数
roslaunch my_package encoder_pos_node.launch square_mode:=true
```

### 方法2: 使用专用状态机
```bash
# 启动专用状态机
roslaunch my_package square_state_machine.launch
```

## 控制参数

### 运动参数
- **边长**: 1.0米 (可在代码中调整`side_length`或`square_side_length`)
- **线速度**: 0.4 m/s (`speed_gain`)
- **角速度**: 2.0 rad/s (转弯时的`steer_gain`)
- **修正增益**: 3.0 (直线行驶时的角度修正)

### 阈值参数
- **距离阈值**: 5cm (`dist_threshold`)
- **角度阈值**: 5度 (`angle_threshold`)

## 状态机工作原理

### 1. 直线行驶 (`_do_straight_side`)
1. 记录起始距离和角度
2. 计算已行驶距离
3. 使用角度修正保持直线
4. 达到目标距离后转换到转弯状态

### 2. 转弯 (`_do_turn`)
1. 记录起始角度
2. 计算目标角度 (当前角度 + 90°)
3. 执行原地左转
4. 达到目标角度后转换到下一条边或完成状态

### 3. 完成 (`_do_complete`)
1. 停止机器人
2. 记录总时间和最终位置
3. 关闭数据记录
4. 发送shutdown信号

## 调试和监控

### 日志输出
- 实时位置信息 (x, y, θ)
- 状态转换信息
- 距离和角度进度
- 完成时间统计

### 数据记录
- 世界坐标系轨迹保存到rosbag文件
- 文件位置: `/data/bags/world_frame_[timestamp].bag`

### LED指示
- **红色**: 停止状态
- **绿色**: 直线行驶
- **蓝色**: 转弯
- **白色**: 完成

## 精确定位原理

### 编码器里程计
使用`odometry_activity.py`中的函数:
- `delta_phi()`: 计算轮子转角变化
- `estimate_pose()`: 差分驱动运动学模型

### 角度处理
- 角度规范化到[0, 2π)范围
- 最小角度差计算 (考虑2π周期性)
- 实时角度修正保持直线

## 参数调优建议

### 提高精度
- 减小距离和角度阈值
- 增加修正增益 (但可能导致震荡)
- 调低运动速度

### 提高速度
- 增加线速度和角速度
- 适当放宽阈值
- 减少修正增益

### 调试步骤
1. 先在仿真环境中测试
2. 逐步调整参数
3. 观察LED指示和日志输出
4. 分析rosbag轨迹数据

## 故障排除

### 常见问题
1. **机器人偏离直线**: 增加`correction_gain`
2. **转弯不准确**: 检查`steer_gain`和`angle_threshold`
3. **距离不准确**: 验证轮子半径和基线参数
4. **LED不工作**: 检查LED服务是否可用

### 检查清单
- [ ] 编码器数据正常接收
- [ ] 运动学参数正确校准
- [ ] LED服务/发布器可用
- [ ] 足够的存储空间用于rosbag
