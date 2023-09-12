import gymnasium as gym

if __name__ == "__main__":
    env = gym.make("Reacher-v4", render_mode="human")
    obs, info = env.reset()
    for _ in range(1000):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
    env.close()
