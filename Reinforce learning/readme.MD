# Duckie Avoid: Randomized-Obstacle RL Training on Duckietown

This project aims to construct diverse training/testing scenarios in the Duckietown simulation environment by randomizing obstacles in the map and training an obstacle avoidance policy using PPO. The project consists of three parts:

1. **`lane_keeping_1.py`**：Provides utility functions for *YAML map obstacle randomization* and a built-in **vision+PID lane keeping baseline controller (with turn awareness and lane loss recovery)**.
2. **`duckie_avoid_env.py`**：Customize the **Gym** environment wrapper `DuckieAvoidWrapper` to package Duckietown into an RL environment with image observations and continuous actions, randomize obstacles during `reset()`, and provide an **obstacle avoidance-guided reward function** in `step()`.
3. **`train_ppo.py`**：Start training on `DuckieAvoidWrapper` using **Stable-Baselines3**'s PPO (`CnnPolicy`).

> Goal: **Randomly generate obstacles → Use reward function to let the agent learn to "move forward, try not to collide, and turn more smoothly" → Start PPO training and save the model. **
---

## Document Structure

```
.
├── lane_keeping_1.py          # Obstacle Randomization Tool + Vision-PID Lane Keeping Baseline (can be run separately)
├── duckie_avoid_env.py        # RL Environment Wrapper: DuckieAvoidWrapper (Randomization + Rewards)
└── train_ppo.py               # Training script: PPO (CnnPolicy) trained on DuckieAvoidWrapper
```

---

## 快速开始

### 1) 运行车道保持基线（可视化）

- **俯视图 + 正常视角（同时）**
```bash
python lane_keeping_1.py --env-name loop_obstacles --randomize-obstacles --show-topdown
```

- **仅正常视角**
```bash
python lane_keeping_1.py --env-name loop_obstacles --randomize-obstacles
```

说明：
- `--env-name loop_obstacles` 选择带障碍的地图。
- `--randomize-obstacles` 在创建环境前与每次 episode 结束后，对 YAML 中的障碍进行**随机化**（在可行驶路面上采样、控制物体最小间距等）。
- `--show-topdown` 打开俯视图窗口（没有该参数则只显示车载相机视角）。
- 如果你希望只使用随机化功能（不跑控制器），也可以在启动前单独调用 `randomize_obstacles_in_yaml()` 生成若干张“标准测试地图”。

### 2) 启动强化学习训练

```bash
python train_ppo.py
```

- 脚本中会：
  - 使用 `DuckieAvoidWrapper` 包装 Duckietown 环境（图像观测为 84×84×3，连续动作为 `[speed, steer]`）。
  - 在 `reset()` 时调用 `randomize_obstacles_in_yaml()` 随机化障碍布局。
  - 用 `PPO("CnnPolicy", ...)` 进行训练，并在训练结束后保存模型（例如 `ppo_duckie_avoid.zip`）。

---

## 关键模块说明

### A. `lane_keeping_1.py` 功能模块

1. **障碍随机化工具**
   - `randomize_obstacles_in_yaml(map_yaml_path, kinds, min_dist, include_static, margin, seed)`：
     - 读取地图 YAML，枚举 `objects` 中指定 `kinds`（如 `duckie`、`cone`、`barrier`）的物体；
     - 仅在**可行驶的 road tiles** 上均匀随机采样落点；
     - 强制物体间**最小间距 min_dist**，保留一定 **margin** 远离 tile 边；
     - 成功后**原地写回 YAML**，方便后续创建环境直接加载新布局。
2. **视觉感知（黄/白线检测）**
   

3. **转弯感知与丢线恢复**
   

4. **PID 控制与平滑**
   

---

### B. `duckie_avoid_env.py` 的奖励函数机制

`DuckieAvoidWrapper.step()` 返回的奖励 `r` 由三部分组成：

1. **前进进度奖励**：`+1.0 * progress`  
   - `progress` 为相邻两帧小车位置（xy）欧氏距离；鼓励**持续前进**，避免原地抖动。

2. **平滑性惩罚**：`-0.05 * |steer - last_steer|`  
   - 惩罚舵角的大幅跳变，鼓励**动作平滑**。

3. **碰撞惩罚 + 终止**：若 `info["collision"] == True`，则 `-1.0` 并 `done=True`  
   - 强烈惩罚碰撞并提前结束该 episode，引导智能体**主动规避障碍**。

**其它要点**：
- 观测为相机图像下采样到 **(84, 84, 3)**，`dtype=uint8`；
- 动作为连续空间 `[speed, steer]`（训练时传给环境会对 steer 取反以匹配 Duckietown 的约定）；
- `frame_skip=2`：一次 `step()` 内重复动作 2 帧，稳定训练并加速推进。

---

## 训练脚本说明（`train_ppo.py`）

- 使用 `DummyVecEnv([make_env])` 创建向量化环境（后续可扩展为多进程以加速采样）；
- 策略为 `CnnPolicy`（适合图像输入）；常用超参例如：`learning_rate=3e-4`、`n_steps=1024`、`batch_size=256`、`gamma=0.99`、`gae_lambda=0.95`；
- 初期可先用 `total_timesteps=20_000` 验证训练管道；上规模训练建议加入：
  - `VecMonitor` 记录 `episode_return`/`length`；
  - `EvalCallback` 固定评测环境并保存最佳模型；
  - 固定随机种子生成**标准测试集**，便于不同算法/轮次对比复现。

---

## 依赖与环境

- Python 3.8+
- `gym-duckietown`, `stable-baselines3`, `opencv-python`, `numpy`, `pyyaml`
- 安装环境指令
```bash
pip install stable-baselines3==1.8.0
```
- 确保可访问到 `loop_obstacles（1）.yaml` （这个地图就是Ray在群里提供地图的无障碍物版本）
---



---


