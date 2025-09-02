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

## How to start

### 1) Running Lane Keeping Baseline (Visualization)
- **Top view + normal view (simultaneously)**
```bash
python lane_keeping_1.py --env-name loop_obstacles --randomize-obstacles --show-topdown
```

- **Normal view only**
```bash
python lane_keeping_1.py --env-name loop_obstacles --randomize-obstacles
```

Explation：
- `--env-name loop_obstacles` Select a map with barriers.
- `--randomize-obstacles` Before creating the environment and after each episode, randomize the obstacles in the YAML (sampling on drivable roads, controlling minimum distance between objects, etc.).
- `--show-topdown` Open the top view window (without this parameter, only the vehicle camera perspective is displayed)。
- If you only want to use the randomization function (without running the controller), you can also call `randomize_obstacles_in_yaml()` before starting to generate several "standard test maps"。

### 2) start reinforce learning training

```bash
python train_ppo.py
```

- in the script：
  - Use `DuckieAvoidWrapper` to wrap the Duckietown environment (image observation is 84×84×3, continuous action is `[speed, steer]`).
  - Call `randomize_obstacles_in_yaml()` to randomize the obstacle layout when `reset()`.
  - Use `PPO("CnnPolicy", ...)` to train and save the model after training (e.g. `ppo_duckie_avoid.zip`).

---

## Key module description

### A. `lane_keeping_1.py` function moduel

1. **Hurdle Randomization Tool**
   - `randomize_obstacles_in_yaml(map_yaml_path, kinds, min_dist, include_static, margin, seed)`：
     - Read the map YAML and enumerate the objects of the specified `kinds` (such as `duckie`, `cone`, `barrier`) in `objects`;
     - Only randomly sample landing points on drivable road tiles;
     - Enforce a minimum distance min_dist between objects, keeping a certain margin away from tile edges;
     - After success, **write back to YAML** in place, so that the new layout can be loaded directly in the subsequent environment creation.
2. **Visual perception (yellow/white line detection)**
   

3. **Turning perception and line loss recovery**
   

4. **PID control and smoothing**
   
(same as the part of pid lane_keeping)
---

### B. `duckie_avoid_env.py` Reward function mechanism

`DuckieAvoidWrapper.step()` The returned reward `r` consists of three parts:

1. **Progress Rewards**：`+1.0 * progress`  
   - `progress` The Euclidean distance between the car's positions (xy) in two adjacent frames; it encourages continuous movement to avoid jittering.

2. **Smoothness penalty**：`-0.05 * |steer - last_steer|`  
   - Punish large jumps in rudder angle and encourage smooth movements.

3. **Collision Penalty + Termination**：If `info["collision"] == True`, then `-1.0` and `done=True`
   - Strongly penalize collisions and end the episode early, guiding the agent to actively avoid obstacles.

**Other points**：
- The observation is the camera image downsampled to **(84, 84, 3)**，`dtype=uint8`；
- Action is continuous space `[speed, steer]`（When passing it to the environment during training, the steer is negated to match the convention of Duckietown）；
- `frame_skip=2`：Repeat the action for 2 frames in one `step()` to stabilize training and accelerate progress.
---

## Training script description（`train_ppo.py`）

- Use `DummyVecEnv([make_env])` to create a vectorized environment (which can be expanded to multiple processes to speed up sampling later);
- The policy is `CnnPolicy` (suitable for image input); common hyperparameters include: `learning_rate=3e-4`, `n_steps=1024`, `batch_size=256`, `gamma=0.99`, `gae_lambda=0.95`;
- Initially, you can use `total_timesteps=20_000` to verify the training pipeline; for large-scale training, it is recommended to add:
  - `VecMonitor` record `episode_return`/`length`；
  - `EvalCallback` Fix the evaluation environment and save the best model;
  - A fixed random seed is used to generate a standard test set to facilitate comparison and reproduction of different algorithms/rounds.
(Later we trained 50,000 times per round, with a learning rate of 0.0003 for the first round and 0.0001 for the second round.)
---

## Dependency and Environment

- Python 3.8+
- `gym-duckietown`, `stable-baselines3`, `opencv-python`, `numpy`, `pyyaml`
- Installation environment instructions
```bash
pip install stable-baselines3==1.8.0
```
- Make sure you have access to `loop_obstacles(1).yaml` (this map is the obstacle-free version of the map Ray provided in the group)
---



---
