import os, gzip, json, numpy as np
import matplotlib.pyplot as plt
run_path = "experiments/2025-01-22_17-37-04/786af4a_zoo_environment_algorithm/flag_sac/0010"
steps_path = os.path.join(run_path, "steps")
print(f"Steps ({steps_path}): ")
list(map(print, os.listdir(steps_path)))
step_path = os.path.join(steps_path, "000000000040000")
nn_analytics_file = os.path.join(step_path, "nn_analytics.json.gz")
with gzip.open(nn_analytics_file, 'rb') as f:
    data_string = f.read()
    data = json.loads(data_string)


layer_output = np.array(data["actor"]["layers"][1]["output"])
plt.imshow(layer_output[:, 0], cmap='hot', interpolation='nearest')
plt.colorbar()
plt.ylabel("Sequence")
plt.xlabel("Feature")
plt.show()