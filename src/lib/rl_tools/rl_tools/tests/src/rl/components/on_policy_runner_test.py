import h5py
import matplotlib.pyplot as plt
import numpy as np


f = h5py.File("test_rl_components_on_policy_runner_buffer.h5", "r")

N_ENVIRONMENTS = 3

for k in f["buffer"]:
    if k != "data":
        data = f["buffer"][k][:]
        data_env_1 = data[np.arange(0, data.shape[0], N_ENVIRONMENTS), :]
        if k == "terminated" or k == "truncated":
            plt.scatter(np.arange(0, 100), data_env_1[:100], label=k)
        else:
            plt.plot(data_env_1[:100], label=k)
plt.legend()
plt.show()


