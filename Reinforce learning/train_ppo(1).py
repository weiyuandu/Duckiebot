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

model = PPO(
    "CnnPolicy", env,
    learning_rate=1e-4,      # ↓ 学习率（之前 3e-4 偏激）
    n_steps=2048,            # ↑ 轨迹更长，优势估计更稳
    batch_size=512,          # ↑ 批更大，方差更小
    clip_range=0.1,          # ↓ 裁剪幅度，降低过大更新
    gamma=0.99,
    gae_lambda=0.95,
    vf_coef=1.0,             # ↑ 强化价值损失权重，帮助价值网
    ent_coef=0.0,            # 先不鼓励更多探索（你环境已足够随机）
    verbose=1
)
# 载入旧参数后“热启动”（可选）
model.set_parameters("ppo_duckie_avoid.zip", exact_match=False)
model.learn(total_timesteps=500_000, reset_num_timesteps=False, progress_bar=True)
model.save("ppo_duckie_avoid_v2.zip")
