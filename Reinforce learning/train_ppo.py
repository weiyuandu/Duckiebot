from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from duckie_avoid_env import DuckieAvoidWrapper


def make_env():
    return DuckieAvoidWrapper(
        map_name="loop_obstacles",
        obs_size=(84,84),
        frame_skip=2,
        randomize=True,
        rand_kwargs=dict(kinds=["duckie","cone","barrier"],
                         min_dist=0.6, include_static=False, margin=0.2)
    )

env = DummyVecEnv([make_env])

model = PPO("CnnPolicy", env,
            learning_rate=3e-4, n_steps=1024, batch_size=256,
            gamma=0.99, gae_lambda=0.95,
            verbose=1)

model.learn(total_timesteps=20000)  # 先跑 2 万步试试看
model.save("ppo_duckie_avoid.zip")
