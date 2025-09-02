import os, random
import numpy as np
import cv2
import gym
from gym import spaces

from gym_duckietown.envs import DuckietownEnv
# Reuse the random obstacle function in lane_keeping_1.py
from lane_keeping_1 import randomize_obstacles_in_yaml, _find_maps_dir


class DuckieAvoidWrapper(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array", "top_down"]}

    def __init__(self, map_name="loop_obstacles",
                 obs_size=(84,84), frame_skip=2,
                 randomize=True, rand_kwargs=None):
        super().__init__()
        self.map_name = map_name
        self.randomize = randomize
        self.rand_kwargs = rand_kwargs or {}
        self.obs_w, self.obs_h = obs_size
        self.frame_skip = frame_skip

        self.env = DuckietownEnv(map_name=map_name,
                                 domain_rand=False,
                                 draw_curve=False, draw_bbox=False)
        self.action_space = spaces.Box(low=np.array([0.0,-1.0]),
                                       high=np.array([0.8, 1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=0, high=255,
                                            shape=(self.obs_h, self.obs_w, 3), dtype=np.uint8)

        self.last_steer = 0.0
        self.prev_pos = None

        maps_dir = _find_maps_dir()
        self.map_yaml = os.path.join(maps_dir, f"{self.map_name}.yaml")

    def _preproc(self, img):
        return cv2.resize(img, (self.obs_w, self.obs_h), interpolation=cv2.INTER_AREA)

    def _progress(self, pos):
        if self.prev_pos is None:
            return 0.0
        return float(np.linalg.norm(np.array(pos[:2]) - np.array(self.prev_pos[:2])))

    def reset(self):
        if self.randomize:
            randomize_obstacles_in_yaml(self.map_yaml, **self.rand_kwargs)
        obs = self.env.reset()
        self.prev_pos = self.env.cur_pos.copy()
        self.last_steer = 0.0
        return self._preproc(obs)

    def step(self, action):
        speed, steer = float(action[0]), float(action[1])
        total_r, done, info = 0.0, False, {}

        for _ in range(self.frame_skip):
            obs, _, done, info = self.env.step(np.array([speed, -steer], dtype=np.float32))
            if done:
                break

        obs_small = self._preproc(obs)
        progress = self._progress(self.env.cur_pos)
        self.prev_pos = self.env.cur_pos.copy()

        # Custom Rewards
        r = 0.0
        r += +1.0 * progress
        r += -0.05 * abs(steer - self.last_steer)  # smooth
        self.last_steer = steer
        if info.get("collision", False):
            r += -1.0
            done = True

        return obs_small, r, done, {"progress": progress, **info}

    def render(self, mode="human"):
        return self.env.render(mode=mode)

    def close(self):
        self.env.close()
