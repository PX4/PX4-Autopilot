import h5py
import gymnasium as gym
import os
import numpy as np


run_path = os.path.join("hof", "ppo_ant")
runs = os.listdir(run_path)
runs = sorted(runs)
run = runs[-1]

checkpoints = os.listdir(os.path.join(run_path, run))
checkpoints = sorted(checkpoints)
checkpoint = checkpoints[-1]
checkpoint_path = os.path.join(run_path, run, checkpoint)

actor_checkpoint = h5py.File(checkpoint_path, "r")

W_input = actor_checkpoint["actor"]["input_layer"]["weights"]["parameters"][:]
b_input = actor_checkpoint["actor"]["input_layer"]["biases"]["parameters"][:].ravel()
W_hidden_1 = actor_checkpoint["actor"]["hidden_layer_0"]["weights"]["parameters"][:]
b_hidden_1 = actor_checkpoint["actor"]["hidden_layer_0"]["biases"]["parameters"][:].ravel()
W_output = actor_checkpoint["actor"]["output_layer"]["weights"]["parameters"][:]
b_output = actor_checkpoint["actor"]["output_layer"]["biases"]["parameters"][:].ravel()

def forward(x, hidden_activation_fn=np.tanh, output_activation_fn=lambda x: x):
    x = W_input @ x + b_input
    x = hidden_activation_fn(x)
    x = W_hidden_1 @ x + b_hidden_1
    x = hidden_activation_fn(x)
    x = W_output @ x + b_output
    x = output_activation_fn(x)
    return x

env = gym.make('Ant-v4', render_mode="human")

obs, _ = env.reset()

reward_acc = 0
step = 0

while True:
    action = forward(obs)
    obs, reward, terminated, truncated, info = env.step(action)
    reward_acc += reward
    print(f"x pos: {env.data.qpos[0]}")
    if terminated or truncated:
        obs, _ = env.reset()
        print(f"Episode reward: {reward_acc} after {step} steps.")
        reward_acc = 0
        step = 0
    step += 1