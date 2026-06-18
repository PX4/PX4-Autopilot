import gymnasium as gym
import numpy as np
import h5py
import mujoco

# create venv
# pip install gymnasium[mujoco] h5py

env = gym.make('Ant-v4')


def get_state(env):
    return np.concatenate([env.data.qpos, env.data.qvel])

obs, _ = env.reset(seed=0)
action_space = env.action_space
action_space.seed(0)

observations = []
next_observations = []
states = []
next_states = []
actions = []
rewards = []
terminated_flags = []
truncated_flags = []
for step_i in range(10000):
    observations.append(obs)
    states.append(get_state(env))
    action = action_space.sample()
    actions.append(action)

    mujoco.mj_forward(env.model, env.data)
    obs, reward, terminated, truncated, _ = env.step(action)
    next_observations.append(obs)
    next_states.append(get_state(env))
    rewards.append(reward)
    terminated_flags.append(terminated)
    truncated_flags.append(truncated)
    if terminated or truncated:
        obs, _ = env.reset()

observations = np.array(observations)
next_observations = np.array(next_observations)
states = np.array(states)
next_states = np.array(next_states)
actions = np.array(actions)
rewards = np.array(rewards)
terminated_flags = np.array(terminated_flags).astype(np.double)
truncated_flags = np.array(truncated_flags).astype(np.double)

with h5py.File('tests_rl_environments_mujoco_ant_data.h5', 'w') as f:
    f.create_dataset('observations', data=observations)
    f.create_dataset('next_observations', data=next_observations)
    f.create_dataset('states', data=states)
    f.create_dataset('next_states', data=next_states)
    f.create_dataset('actions', data=actions)
    f.create_dataset('rewards', data=rewards)
    f.create_dataset('terminated_flags', data=terminated_flags)
    f.create_dataset('truncated_flags', data=truncated_flags)

