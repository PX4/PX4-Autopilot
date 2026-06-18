import json
import matplotlib.pyplot as plt
import os
import time

def truncate_floats(obj):
    if isinstance(obj, float):
        # Truncate the float to 2 decimal places, using scientific notation when necessary
        return f"{obj:.2e}" if abs(obj) < 0.01 or abs(obj) > 1000 else round(obj, 2)
    elif isinstance(obj, dict):
        return {k: truncate_floats(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [truncate_floats(i) for i in obj]
    return obj

def update_chart(axs, fig, n_plots):
    # Clear the existing plot
    for ax in axs:
        ax.clear()

    with open('gru_training_sequences.json') as f:
        data = json.load(f)

    fig.suptitle(f"{json.dumps(truncate_floats(data['meta']), indent=4)}", fontsize=8)

    for i in range(n_plots):
        batch = data["batch"]
        ax = axs[i]
        label_set_input = False
        label_set_reset = False
        for j, s in enumerate(batch[i]):
            if s["input"] == 1:
                ax.axvline(x=j, color='black', alpha=0.2, linestyle='--', label="input" if not label_set_input else None)
                label_set_input = True
            if s["reset"] == 1:
                ax.axvline(x=j, color='r', alpha=0.5, linestyle='-', label="reset" if not label_set_reset else None)
                label_set_reset = True
        ax.plot(range(len(batch[i])), [s["target"] for s in batch[i]], label="target")
        ax.plot(range(len(batch[i])), [s["output"] for s in batch[i]], label="output")
        ax.legend()

    plt.savefig('gru_training_sequences.pdf')
    # plt.draw()
    # plt.pause(0.1)

if __name__ == "__main__":
    # plt.ion()  # Enable interactive mode

    file_path = 'gru_training_sequences.json'
    last_mod_time = os.path.getmtime(file_path)

    n_plots = 2
    fig, axs = plt.subplots(n_plots, 1, figsize=(10, 10))

    # Run the initial chart display
    update_chart(axs, fig, n_plots)

    try:
        while True:
            # Check if the file modification time has changed
            current_mod_time = os.path.getmtime(file_path)
            if current_mod_time != last_mod_time:
                print("File changed, updating chart...")
                last_mod_time = current_mod_time
                update_chart(axs, fig, n_plots)

            time.sleep(1)  # Poll every 1 second to check for changes
    except KeyboardInterrupt:
        print("Terminating the monitoring...")


