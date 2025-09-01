#!/usr/bin/env python3

import re
import matplotlib.pyplot as plt
import numpy as np

def parse_log_positions(log_text):
    """
    从日志中提取位置信息
    """
    positions = []
    
    # 匹配位置信息的正则表达式
    patterns = [
        # 匹配 "Current pose: x=..., y=..., θ=...°"
        r'Current pose: x=([-\d.]+), y=([-\d.]+), θ=([-\d.]+)°',
        # 匹配 "Progress: pos=(...), θ=...°"
        r'Progress: pos=\(([-\d.]+), ([-\d.]+)\), θ=([-\d.]+)°',
        # 匹配 "Initial position: x=..., y=..., θ=...°"
        r'Initial position: x=([-\d.]+), y=([-\d.]+), θ=([-\d.]+)°',
        # 匹配 "Final position: x=..., y=..., θ=...°"
        r'Final position: x=([-\d.]+), y=([-\d.]+), θ=([-\d.]+)°',
        # 匹配 "Final odometry: x=..., y=..., θ=...°"
        r'Final odometry: x=([-\d.]+), y=([-\d.]+), θ=([-\d.]+)°'
    ]
    
    # 按时间戳排序提取位置
    lines = log_text.strip().split('\n')
    
    for line in lines:
        for pattern in patterns:
            match = re.search(pattern, line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                theta = float(match.group(3))
                
                # 提取时间戳用于排序
                timestamp_match = re.search(r'\[(\d+\.\d+)\]', line)
                timestamp = float(timestamp_match.group(1)) if timestamp_match else 0
                
                positions.append({
                    'timestamp': timestamp,
                    'x': x,
                    'y': y,
                    'theta': theta,
                    'line': line.strip()
                })
                break
    
    # 按时间戳排序
    positions.sort(key=lambda p: p['timestamp'])
    
    return positions

def plot_trajectory(positions):
    """
    绘制轨迹图
    """
    if not positions:
        print("No positions found in the log!")
        return
    
    # 提取坐标数据
    x_coords = [p['x'] for p in positions]
    y_coords = [p['y'] for p in positions]
    timestamps = [p['timestamp'] for p in positions]
    
    # 创建图形
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # 轨迹图
    ax1.plot(x_coords, y_coords, 'b-', linewidth=2, label='Robot Trajectory')
    ax1.scatter(x_coords, y_coords, c=range(len(x_coords)), cmap='viridis', s=50, alpha=0.7)
    
    # 标记起点和终点
    ax1.scatter(x_coords[0], y_coords[0], c='green', s=100, marker='o', label=f'Start ({x_coords[0]:.3f}, {y_coords[0]:.3f})')
    ax1.scatter(x_coords[-1], y_coords[-1], c='red', s=100, marker='s', label=f'End ({x_coords[-1]:.3f}, {y_coords[-1]:.3f})')
    
    # 标记目标正方形的角点
    target_corners = [(0, 0), (1, 0), (1, 1), (0, 1)]
    for i, (tx, ty) in enumerate(target_corners):
        ax1.scatter(tx, ty, c='orange', s=80, marker='^', alpha=0.8)
        ax1.annotate(f'Target {i+1}', (tx, ty), xytext=(5, 5), textcoords='offset points', fontsize=8)
    
    # 绘制理想正方形路径
    ideal_x = [0, 1, 1, 0, 0]
    ideal_y = [0, 0, 1, 1, 0]
    ax1.plot(ideal_x, ideal_y, 'r--', alpha=0.5, linewidth=1, label='Ideal Square Path')
    
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Duckiebot Square Trajectory')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axis('equal')
    
    # 添加一些统计信息
    total_distance = sum(np.sqrt((x_coords[i+1] - x_coords[i])**2 + (y_coords[i+1] - y_coords[i])**2) 
                        for i in range(len(x_coords)-1))
    
    ax1.text(0.02, 0.98, f'Total Distance: {total_distance:.3f} m\nPoints: {len(positions)}', 
             transform=ax1.transAxes, verticalalignment='top', 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # 位置随时间变化图
    relative_time = [(t - timestamps[0]) for t in timestamps]
    
    ax2.plot(relative_time, x_coords, 'r-', label='X Position', linewidth=2)
    ax2.plot(relative_time, y_coords, 'b-', label='Y Position', linewidth=2)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()
    
    # 打印统计信息
    print(f"\n=== Trajectory Analysis ===")
    print(f"Total waypoints: {len(positions)}")
    print(f"Start position: ({x_coords[0]:.3f}, {y_coords[0]:.3f})")
    print(f"End position: ({x_coords[-1]:.3f}, {y_coords[-1]:.3f})")
    print(f"Estimated total distance: {total_distance:.3f} m")
    print(f"Duration: {relative_time[-1]:.1f} seconds")
    
    # 计算与目标点的误差
    print(f"\n=== Target Point Errors ===")
    targets = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    
    # 找到最接近每个目标点的实际位置
    for i, (tx, ty) in enumerate(targets):
        min_dist = float('inf')
        closest_point = None
        for pos in positions:
            dist = np.sqrt((pos['x'] - tx)**2 + (pos['y'] - ty)**2)
            if dist < min_dist:
                min_dist = dist
                closest_point = pos
        print(f"Target {i+1} ({tx}, {ty}): closest actual ({closest_point['x']:.3f}, {closest_point['y']:.3f}), error: {min_dist:.3f} m")

def main():
    # 这里是您提供的日志内容
    log_content = """
[INFO] [1756726491.490653]: [/square_controller_state_machine_node] Initializing...
[INFO] [1756726492.241529]: [/square_controller_state_machine_node] Health status changed [STARTING] -> [STARTED]
[INFO] [1756726492.287155]: [/square_controller_state_machine_node] Loading kinematics calibration...
[INFO] [1756726493.991077]: [/square_controller_state_machine_node] Square controller state machine initialized.
[INFO] [1756726494.019444]: [/square_controller_state_machine_node] Wheel radius: 0.0318 m, Baseline: 0.1000 m
[INFO] [1756726495.128798]: [/square_controller_state_machine_node] Vehicle name: duck
[INFO] [1756726495.172822]: [/square_controller_state_machine_node] Publishing movement commands to: /duck/car_cmd_switch_node/cmd
[INFO] [1756726495.225568]: [/square_controller_state_machine_node] Publishing LED patterns to: /duck/led_emitter_node/led_pattern
[INFO] [1756726495.243674]: [/square_controller_state_machine_node] Subscribing to encoders: /duck/left_wheel_encoder_node/tick, /duck/right_wheel_encoder_node/tick
[INFO] [1756726495.296622]: [/square_controller_state_machine_node] Waiting for system initialization...
[INFO] [1756726497.351598]: [/square_controller_state_machine_node] Starting square movement with encoder feedback...
[INFO] [1756726497.413866]: [/square_controller_state_machine_node] Initial position: x=0.000, y=0.000, θ=0.0°
[INFO] [1756726497.465714]: [/square_controller_state_machine_node] State transition: INIT -> WAIT_FOR_ENCODERS
[INFO] [1756726497.514310]: [/square_controller_state_machine_node] Encoder data received, starting square movement
[INFO] [1756726497.550624]: [/square_controller_state_machine_node] State transition: WAIT_FOR_ENCODERS -> SET_CORNER_LED
[INFO] [1756726497.615061]: [/square_controller_state_machine_node] LED pattern published: RED RGB[1.0, 0.0, 0.0]
[INFO] [1756726497.653350]: [/square_controller_state_machine_node] Corner 1: LED set to RED
[INFO] [1756726497.704783]: [/square_controller_state_machine_node] Current pose: x=0.000, y=0.000, θ=0.0°
[INFO] [1756726497.777146]: [/square_controller_state_machine_node] State transition: SET_CORNER_LED -> MOVING_FORWARD
[INFO] [1756726497.814509]: [/square_controller_state_machine_node] Moving to position (1.00, 0.00)
[INFO] [1756726499.918542]: [/square_controller_state_machine_node] Progress: pos=(0.791, 0.011), θ=357.5°, dist_err=0.209m, ang_err=-0.5°
[INFO] [1756726500.316747]: [/square_controller_state_machine_node] Reached position (1.00, 0.00)
[INFO] [1756726500.905727]: [/square_controller_state_machine_node] State transition: MOVING_FORWARD -> TURNING_LEFT
[INFO] [1756726500.948712]: [/square_controller_state_machine_node] Turning to orientation 90.0°
[INFO] [1756726502.052026]: [/square_controller_state_machine_node] Turn progress: current_θ=140.8°, target_θ=90.0°, ang_err=-50.8°
[INFO] [1756726502.551854]: [/square_controller_state_machine_node] Reached orientation 90.0°
[INFO] [1756726503.100709]: [/square_controller_state_machine_node] State transition: TURNING_LEFT -> SET_CORNER_LED
[INFO] [1756726503.180292]: [/square_controller_state_machine_node] LED pattern published: GREEN RGB[0.0, 1.0, 0.0]
[INFO] [1756726503.211102]: [/square_controller_state_machine_node] Corner 2: LED set to GREEN
[INFO] [1756726503.245924]: [/square_controller_state_machine_node] Current pose: x=1.048, y=0.012, θ=89.0°
[INFO] [1756726503.315704]: [/square_controller_state_machine_node] State transition: SET_CORNER_LED -> MOVING_FORWARD
[INFO] [1756726503.331478]: [/square_controller_state_machine_node] Moving to position (1.00, 1.00)
[INFO] [1756726505.483815]: [/square_controller_state_machine_node] Progress: pos=(0.995, 0.826), θ=89.0°, dist_err=0.174m, ang_err=-0.6°
[INFO] [1756726505.783663]: [/square_controller_state_machine_node] Reached position (1.00, 1.00)
[INFO] [1756726506.314879]: [/square_controller_state_machine_node] State transition: MOVING_FORWARD -> TURNING_LEFT
[INFO] [1756726506.338899]: [/square_controller_state_machine_node] Turning to orientation 180.0°
[INFO] [1756726507.410058]: [/square_controller_state_machine_node] Turn progress: current_θ=191.6°, target_θ=180.0°, ang_err=-11.6°
[INFO] [1756726507.494670]: [/square_controller_state_machine_node] Reached orientation 180.0°
[INFO] [1756726508.020753]: [/square_controller_state_machine_node] State transition: TURNING_LEFT -> SET_CORNER_LED
[INFO] [1756726508.061466]: [/square_controller_state_machine_node] LED pattern published: BLUE RGB[0.0, 0.0, 1.0]
[INFO] [1756726508.102688]: [/square_controller_state_machine_node] Corner 3: LED set to BLUE
[INFO] [1756726508.134620]: [/square_controller_state_machine_node] Current pose: x=1.002, y=1.018, θ=173.8°
[INFO] [1756726508.171113]: [/square_controller_state_machine_node] State transition: SET_CORNER_LED -> MOVING_FORWARD
[INFO] [1756726508.212890]: [/square_controller_state_machine_node] Moving to position (0.00, 1.00)
[INFO] [1756726510.262583]: [/square_controller_state_machine_node] Progress: pos=(0.222, 0.975), θ=167.1°, dist_err=0.224m, ang_err=6.6°
[INFO] [1756726510.763267]: [/square_controller_state_machine_node] Reached position (0.00, 1.00)
[INFO] [1756726511.309176]: [/square_controller_state_machine_node] State transition: MOVING_FORWARD -> TURNING_LEFT
[INFO] [1756726511.339343]: [/square_controller_state_machine_node] Turning to orientation 270.0°
[INFO] [1756726512.040637]: [/square_controller_state_machine_node] Reached orientation 270.0°
[INFO] [1756726512.573379]: [/square_controller_state_machine_node] State transition: TURNING_LEFT -> SET_CORNER_LED
[INFO] [1756726512.613704]: [/square_controller_state_machine_node] LED pattern published: WHITE RGB[1.0, 1.0, 1.0]
[INFO] [1756726512.653309]: [/square_controller_state_machine_node] Corner 4: LED set to WHITE
[INFO] [1756726512.675067]: [/square_controller_state_machine_node] Current pose: x=-0.038, y=1.024, θ=268.0°
[INFO] [1756726512.741143]: [/square_controller_state_machine_node] State transition: SET_CORNER_LED -> MOVING_FORWARD
[INFO] [1756726512.777539]: [/square_controller_state_machine_node] Moving to position (0.00, 0.00)
[INFO] [1756726514.815439]: [/square_controller_state_machine_node] Progress: pos=(0.043, 0.251), θ=257.8°, dist_err=0.255m, ang_err=2.4°
[INFO] [1756726515.316259]: [/square_controller_state_machine_node] Reached position (0.00, 0.00)
[INFO] [1756726515.346771]: [/square_controller_state_machine_node] State transition: MOVING_FORWARD -> COMPLETED
[INFO] [1756726515.412952]: [/square_controller_state_machine_node] LED pattern published: OFF RGB[0.0, 0.0, 0.0]
[INFO] [1756726515.444501]: [/square_controller_state_machine_node] LEDs turned off
[INFO] [1756726515.483468]: [/square_controller_state_machine_node] Square movement completed!
[INFO] [1756726515.524198]: [/square_controller_state_machine_node] Final position: x=-0.012, y=0.001, θ=262.9°
[INFO] [1756726515.550953]: [/square_controller_state_machine_node] Total distance traveled: 4.223 m
[INFO] [1756726515.594913]: [/square_controller_state_machine_node] State transition: COMPLETED -> SHUTDOWN
[INFO] [1756726515.644234]: [/square_controller_state_machine_node] State machine shutting down...
[INFO] [1756726515.717199]: [/square_controller_state_machine_node] Square controller state machine completed. Keeping node alive for 2 seconds...
[INFO] [1756726517.754068]: [/square_controller_state_machine_node] LED pattern published: OFF RGB[0.0, 0.0, 0.0]
[INFO] [1756726518.182956]: [/square_controller_state_machine_node] Received shutdown request.
[INFO] [1756726518.210158]: [/square_controller_state_machine_node] LED pattern published: OFF RGB[0.0, 0.0, 0.0]
[INFO] [1756726518.237908]: [/square_controller_state_machine_node] LEDs turned off
[INFO] [1756726518.305534]: [/square_controller_state_machine_node] Square controller state machine shutting down...
[INFO] [1756726518.342527]: [/square_controller_state_machine_node] Final odometry: x=-0.012, y=0.001, θ=262.9°
[INFO] [1756726518.372139]: [/square_controller_state_machine_node] Total distance traveled: 4.223 m
[INFO] [1756726518.404900]: [/square_controller_state_machine_node] LED pattern published: OFF RGB[0.0, 0.0, 0.0]
[INFO] [1756726518.422927]: [/square_controller_state_machine_node] LEDs turned off
[INFO] [1756726518.442775]: [/square_controller_state_machine_node] Square controller state machine shutting down...
[INFO] [1756726518.470204]: [/square_controller_state_machine_node] Final odometry: x=-0.012, y=0.001, θ=262.9°
[INFO] [1756726518.485723]: [/square_controller_state_machine_node] Total distance traveled: 4.223 m
    """
    
    print("Parsing log data...")
    positions = parse_log_positions(log_content)
    
    print(f"Found {len(positions)} position entries")
    
    if positions:
        print("\nFirst few positions:")
        for i, pos in enumerate(positions[:5]):
            print(f"  {i+1}: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['theta']:.1f}°)")
        
        print("Plotting trajectory...")
        plot_trajectory(positions)
    else:
        print("No position data found in log!")

if __name__ == "__main__":
    main()
