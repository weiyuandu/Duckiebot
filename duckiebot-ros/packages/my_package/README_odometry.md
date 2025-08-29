# Odometry Node

这个odometry node为Duckiebot提供位姿估计功能，基于轮式编码器数据计算机器人的位置和朝向。

## 文件说明

- `odometry_activity.py`: 包含odometry计算的核心函数
- `odometry_node.py`: 主要的ROS节点，订阅编码器数据并发布位姿信息
- `launchers/odometry.sh`: 启动脚本

## 功能特性

### 订阅的话题
- `/{vehicle_name}/left_wheel_encoder_node/tick`: 左轮编码器数据
- `/{vehicle_name}/right_wheel_encoder_node/tick`: 右轮编码器数据

### 发布的话题
- `/{vehicle_name}/odometry_node/odometry`: 标准的nav_msgs/Odometry消息
- `/{vehicle_name}/odometry_node/pose`: 简化的Pose2DStamped消息

### TF变换
- 发布从`odom`到`{vehicle_name}/base_link`的变换

## 使用方法

### 1. 直接运行节点
```bash
rosrun my_package odometry_node.py
```

### 2. 使用launcher脚本
```bash
./launchers/odometry.sh
```

### 3. 在其他launch文件中包含
```xml
<include file="$(find my_package)/launch/odometry.launch">
    <arg name="veh" value="$(arg veh)"/>
</include>
```

## 参数配置

节点会自动从ROS参数服务器加载运动学参数：
- `/{vehicle_name}/kinematics_node/radius`: 轮子半径 (默认: 0.0318m)
- `/{vehicle_name}/kinematics_node/baseline`: 轮距 (默认: 0.1m)

## 输出示例

节点运行时会输出当前位姿信息：
```
[INFO] Pose: x=0.123, y=0.456, θ=45.0°
```

## 技术细节

### Odometry计算
使用差动驱动运动学模型：
1. 根据编码器脉冲计算轮子旋转角度
2. 计算每个轮子行驶的距离
3. 估算机器人中心的位移和旋转
4. 更新位姿估计

### 坐标系
- `odom`: 里程计坐标系（起始位置为原点）
- `base_link`: 机器人本体坐标系

### 误差处理
- 提供协方差估计（误差随行驶距离增加）
- 角度标准化到[-π, π]范围
- 编码器数据同步处理

## 兼容性

这个节点与Duckietown DTROS框架完全兼容，可以与其他Duckiebot节点无缝集成。
