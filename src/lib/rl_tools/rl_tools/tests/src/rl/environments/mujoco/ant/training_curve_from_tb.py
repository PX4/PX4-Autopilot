# %%
from tbparse import SummaryReader
import os
from tqdm import tqdm
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
log_dir = "../../../../../../logs_ppo_ant_normobs_adaptive"
reader = SummaryReader(log_dir, extra_columns={"dir_name"})
df = reader.scalars
# %%

episode_return = df.loc[df["tag"] == "episode/return"] #.plot(x="step", y="value")
print(episode_return.keys())
bins = list(range(0, 10200000, 100000))
episode_return_bins = pd.cut(episode_return["step"], bins=bins, labels=bins[:-1])
grouped = df.groupby(episode_return_bins)["value"].agg(["mean", "std"])

# %%
grouped


# %%
fig, ax = plt.subplots()
ax.plot(grouped.index, grouped["mean"], color='blue', label='mean')
ax.fill_between(grouped.index, grouped["mean"] - grouped["std"], grouped["mean"] + grouped["std"], alpha=0.3, color='blue', label='std')
ax.legend(loc="upper left")


# %%
# Generate some data
x = np.linspace(0, 10, 100)
y = np.sin(x)
y_err = 0.2 * np.random.randn(len(x))

# Create the plot

# Add labels and title
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_title('Plot Title')

# Customize the plot
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)
ax.tick_params(axis='both', which='both', length=0)

plt.legend()
plt.show()

# %%
