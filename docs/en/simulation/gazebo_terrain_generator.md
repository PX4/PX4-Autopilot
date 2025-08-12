# Gazebo Terrain Generator

:::warning
These simulators are not maintained, tested, or supported, by the core development team.
They may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

A super easy-to-use tool for generate 3D Gazebo terrain using real-world elevation and satellite data.


<lite-youtube videoid="g09vorj0wf0" title="3D Gazebo terrain Generator"/>


::: info
**Tested Environment:**
- Ubuntu 24.04 LTS
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_ubuntu/) (Latest LTS)
:::



## Installtion

Clone the repo.
```bash
git clone https://github.com/saiaravind19/gazebo_terrain_generator.git
```

It's recommended to use a virtual environment to avoid dependency conflicts:


### 1. Create Virtual Environment

```bash
# Create and activate virtual environment
python3 -m venv terrain_generator
source terrain_generator/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Configuration

Set environment variables to change output directory for generated files:

```bash
export GAZEBO_MODEL_PATH="/path/to/PX4-Autopilot/Tools/simulation/gz/models"
export GAZEBO_WORLD_PATH="/path/to/PX4-Autopilot/Tools/simulation/gz/worlds"
```
:::info
**Default location:**  If no environment variable is set, model and world files are saved to:
```
Models saved in **/path/to/gazebo_terrain_generator/output/gazebo_terrain/**
World files in **/path/to/gazebo_terrain_generator/output/gazebo_terrain/worlds**

```
:::

## Usage

### 1. Run Gazebo world Generator
Navigate to the `gazebo_terrain_generator` repository:

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

1. Export the model name.

```bash
export PX4_GZ_WORLD=<Model name>
make px4_sitl gz_x500 # For spawing Quadrotor (x500)
```


## Further Information
- [Gazebo_terrain_generator readme](https://github.com/saiaravind19/gazebo_terrain_generator)
