# Gazebo Terrain Generator

::: warning
These simulators are not maintained, tested, or supported by the core development team.
They may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

A super easy-to-use tool to generate 3D Gazebo terrain using real-world elevation and satellite data.

<lite-youtube videoid="-RYXWoHUZNU" title="3D Gazebo terrain Generator"/>

::: info
**Tested Environment**
- Ubuntu 24.04 LTS
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) (latest LTS)
:::



## Installation

Clone the repo.
```bash
git clone https://github.com/saiaravind19/gazebo_terrain_generator.git
```

### Create Virtual Environment
::: info
It is recommended to use a Python virtual environment to avoid dependency conflicts.
:::
1. Navigate to your local repository.
   ```bash
   cd ~/wherever/gazebo_terrain_generator
   ```

2. Setup a virtual environment and install dependency packages.
   ```bash
   # Create and activate virtual environment
   python3 -m venv terrain_generator
   source terrain_generator/bin/activate

   # Install dependencies
   pip install -r requirements.txt
   ```

### Configuration

Set environment variables to change output directory for generated files:

   ```bash
   export GAZEBO_MODEL_PATH="/path/to/PX4-Autopilot/Tools/simulation/gz/models"
   export GAZEBO_WORLD_PATH="/path/to/PX4-Autopilot/Tools/simulation/gz/worlds"
   ```

:::info
**Default location:** If no environment variables are set, model and world files are saved to:

- Models saved in **~/wherever/gazebo_terrain_generator/output/gazebo_terrain/**
- World files in **~/wherever/gazebo_terrain_generator/output/gazebo_terrain/worlds**

:::

## Usage

### 1. Run Gazebo world generator
1. Navigate to your local repository.
   ```bash
   cd ~/wherever/gazebo_terrain_generator
   ```

2. Start the gazebo_terrain_generator server.
   ```bash
   source terrain_generator/bin/activate
   python scripts/server.py
   ```

### 2. Generate Terrain

1. Open browser: `http://localhost:8080`
2. Search for any location on Earth.
3. Select the region of interest.
4.  Toggle the *Include Buildings* button to include or exclude buildings.
5. Give a model name.
6. Place the spawn marker at the desired location.
7. Click **Generate Terrain**

## Spawn the Model in Gazebo

1. Export the model name.

```bash
export PX4_GZ_WORLD=<Model name>
make px4_sitl gz_x500 # For spawning quadrotor (x500)
```
::: info
Replace `<Model name>` with same name used when generating the gazebo world.
For example if model name is **Zurich** then the command is ```export PX4_GZ_WORLD=Zurich```

:::


## Further Information
- [Gazebo_terrain_generator readme](https://github.com/saiaravind19/gazebo_terrain_generator)
