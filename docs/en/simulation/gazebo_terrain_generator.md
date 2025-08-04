# Gazebo Terrain Generator

This section contains information about _community-supported_ simulations

:::warning
These simulators are not maintained, tested, or supported, by the core development team.
They may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

A super easy-to-use tool for generate 3D Gazebo terrain using real-world elevation and satellite data.


<lite-youtube videoid="pxL2UF9xl_w" title="3D Gazebo terrain Generator"/>

::: info
**Tested Environment:**
- Ubuntu 24.04 LTS
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) (Latest LTS)
:::


## Installtion


It's recommended to use a virtual environment to avoid dependency conflicts:


### 1. Create Virtual Environment

```bash
# Create and activate virtual environment
python3 -m venv terrain_generator
source terrain_generator/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Configuration (Optional)

Set custom output directory for generated worlds:

```bash
# Default: gazebo_terrian_generator/output/gazebo_terrian/
export GAZEBO_WORLD_PATH="~/PX4-Autopilot/Tools/simulation/gz/models"
export GAZEBO_MODEL_PATH="~/PX4-Autopilot/Tools/simulation/gz/worlds"
```

## Usage

### 1. Run Gazebo world Generator
Navigate to the `gazebo_terrian_generator` repository:

```bash
source terrain_generator/bin/activate
python scripts/server.py
```
### 2. Generate Terrain

1. Open browser: `http://localhost:8080`
2. Search for any location on Earth.
3. Select the region of interest.
4. Give a model name.
5. Place spawn marker at desired location.
6. Click **Generate Terrain**

## Spawn the Model in Gazebo

1. Export the model name and enable Launch pad.

```bash
export PX4_GZ_WORLD=<Model name>
export ENABLE_LAUNCH_PAD=True
make px4_sitl gz_x500 # For spawing Quadrotor (x500)
```


## Further Information
- [Gazebo_terrain_generator readme](https://github.com/saiaravind19/gazebo_terrain_generator)
