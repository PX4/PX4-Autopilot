import os
import json
import numpy as np
import matplotlib.pyplot as plt

def load_experiment(experiment, time, lower_percentile=0, upper_percentile=100):
    steps = []
    steps_set = False
    returns = []

    for run_path in experiment if isinstance(experiment, list) else map(lambda x: os.path.join(experiment, x), os.listdir(experiment)):
        # run_path = os.path.join(experiment, seed)
        if os.path.exists(os.path.join(run_path, "return.json.set")):
            with open(os.path.join(run_path, "return.json")) as f:
                results = json.load(f)
                for i, evaluation_step in enumerate(results):
                    if not steps_set:
                        steps.append(evaluation_step["step"])
                        returns.append([])
                    returns[i].append(np.mean(evaluation_step["returns"]))
            steps_set = True

    returns = np.array(returns)
    returns = returns.reshape((len(steps), -1))
    steps = np.array(steps)

    # mean_returns = np.mean(returns, axis=1)
    # std_returns = np.std(returns, axis=1)
    wall_time = steps/(steps.max() / time)
    upper_percentile_values = np.percentile(returns, upper_percentile, axis=1)
    lower_percentile_values = np.percentile(returns, lower_percentile, axis=1)
    iqm_mask = np.logical_and(returns >= lower_percentile_values[:, None], returns <= upper_percentile_values[:, None])
    iqm_data = np.where(iqm_mask, returns, np.nan)
    iqm_mean_returns = np.nanmean(iqm_data, axis=1)
    iqm_std_returns = np.nanstd(iqm_data, axis=1)
    return steps, wall_time, iqm_mean_returns, iqm_std_returns, returns


experiments = {
    # "baseline": ("experiments/2024-11-26_14-33-33/c91e9c4_zoo_environment_algorithm/l2f_sac", 1.144),
    # "50k": ("experiments/2024-11-26_16-12-12/c91e9c4_zoo_environment_algorithm/l2f_sac", 0.844),
    # "50k_+1-layer": ("experiments/2024-11-26_16-15-58/c91e9c4_zoo_environment_algorithm/l2f_sac", 0.844),
    # "aftera-all": ("experiments/2024-11-26_20-35-04/d56db9c_zoo_environment_algorithm/l2f_sac/", 0.99),
    # "dt2x": ("experiments/2024-11-26_21-17-12/dfa429a_zoo_environment_algorithm/l2f_sac", 0.99),
    # "new": ("experiments/2024-12-09_12-05-42/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "new2": ("experiments/2024-12-09_12-14-24/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "new3": ("experiments/2024-12-09_12-35-07/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "fullrb": ("experiments/2024-12-09_12-43-09/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "rb5k": ("experiments/2024-12-09_12-44-11/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "rb20k": ("experiments/2024-12-09_12-45-16/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "relu": ("experiments/2024-12-09_12-46-28/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "90deg": ("experiments/2024-12-09_13-11-13/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "tanh": ("experiments/2024-12-09_13-13-50/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "4layer": ("experiments/2024-12-09_13-15-53/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "positive": ("experiments/2024-12-09_13-18-36/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "no-guidance": ("experiments/2024-12-09_13-22-30/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "6-layer": ("experiments/2024-12-09_13-27-00/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "great_convergence": ("experiments/2024-12-09_13-30-20/06548f4_zoo_environment_algorithm/l2f_sac", 0.99),
    # "longer": ("experiments/2024-12-09_13-33-03/06548f4_zoo_environment_algorithm/l2f_sac", 12),
    # "bigger": ("experiments/2024-12-09_13-36-17/06548f4_zoo_environment_algorithm/l2f_sac", 4),
    # "rebase": ("experiments/2024-12-09_13-51-24/d9384be_zoo_environment_algorithm/l2f_sac", 0.92),
    # "2env": ("experiments/2024-12-09_13-57-49/d9384be_zoo_environment_algorithm/l2f_sac", 1.10),
    # "16env": ("experiments/2024-12-09_14-01-42/d9384be_zoo_environment_algorithm/l2f_sac", 1.40),
    # "lesswarmup": ("experiments/2024-12-09_14-04-01/d9384be_zoo_environment_algorithm/l2f_sac", 1.40),
    # "8env": ("experiments/2024-12-09_14-06-48/d9384be_zoo_environment_algorithm/l2f_sac", 1.23),
    # "low-lr": ("experiments/2024-12-09_14-12-10/d9384be_zoo_environment_algorithm/l2f_sac", 1.23),
    # "ms": ("experiments/2024-12-09_14-20-29/d9384be_zoo_environment_algorithm/l2f_sac", 0.92),
    # "0.90": ([
    #     "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0110",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0111",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0112",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0113",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0114",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0115",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0116",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0117",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0118",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0119",
    # ], 20*60),
    # "0.95": ([
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0010",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0011",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0012",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0013",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0014",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0015",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0016",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0017",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0018",
    #      "experiments/2025-01-10_13-11-25/60ee5b2_zoo_environment_algorithm/flag_sac/0019",
    # ], 20*60),
    "old": ("experiments/2025-01-28_19-06-49/b09c43f_zoo_environment_algorithm/l2f_sac", 20*60),
    "new": ("experiments/2025-01-28_19-36-18/1bc11bc_zoo_environment_algorithm/l2f_sac", 20*60)
    # "0.98": ("experiments/2025-01-10_14-45-12/78dd508_zoo_environment_algorithm/flag_sac", 20*60),
}


combinations = [
    (True, (0, 100)),
    (True, (25, 75)),
    (True, (90, 100)),
    (False, (0, 100)),
    (False, (25, 75)),
    (False, (90, 100)),
]



cols = 3
fig, axes = plt.subplots(2, cols, figsize=(12, 10))

for idx, (use_wall_time, (iqm_lower_percentile, iqm_upper_percentile)) in enumerate(combinations):
    ax = axes[idx // cols, idx % cols]
    for experiment_name, experiment in experiments.items():
        steps, wall_time, mean_returns, std_returns, returns = load_experiment(*experiment, lower_percentile=iqm_lower_percentile, upper_percentile=iqm_upper_percentile)
        x = wall_time if use_wall_time else steps
        ax.plot(x, mean_returns, label=experiment_name)
        ax.fill_between(x, mean_returns - std_returns, mean_returns + std_returns, alpha=0.2)
    
    xlabel = "Time [s]" if use_wall_time else "Step"
    ylabel = "Returns"
    title = f"Learning Curve ({'Wall Time' if use_wall_time else 'Steps'}, {f'IQM({iqm_lower_percentile}%:{iqm_upper_percentile}%)'})"
    
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()

plt.tight_layout()
plt.show()


plt.figure()
for experiment_name, experiment in experiments.items():
    steps, wall_time, mean_returns, std_returns, returns = load_experiment(*experiment)
    plt.hist(returns[-1], bins=5, density=True, label=experiment_name)
plt.legend()
plt.show()


# plt.figure(figsize=(10, 6))

# for idx, (experiment_name, experiment) in enumerate(experiments.items()):
#     steps, wall_time, mean_returns, std_returns, returns = load_experiment(*experiment)
#     plt.hist(returns[-1], bins=10, density=True, label=experiment_name, alpha=0.5, histtype='bar')

# plt.legend()
# plt.xlabel('Returns')
# plt.ylabel('Density')
# plt.title('Distribution of Returns by Experiment')
# plt.show()