import gymnasium as gym

env = gym.make('CartPole-v1', render_mode='human')
obs, _ = env.reset()

for _ in range(500):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        obs, _ = env.reset()
