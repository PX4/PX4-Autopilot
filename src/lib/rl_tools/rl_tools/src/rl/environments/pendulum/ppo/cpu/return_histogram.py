import matplotlib.pyplot as plt
import json
import numpy as np
import os
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--file", type=str, default="pendulum_ppo_returns.json")

args = parser.parse_args()
with open(args.file, "r") as f:
    returns = json.load(f)

plt.hist(np.array(returns).ravel(), bins=100)
plt.show()
