# Gazebo Terrain Generator

This section contains information about _community-supported_ simulations

:::warning
These simulators are not maintained, tested, or supported, by the core development team.
They may or may not work with current versions of PX4.

See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

A super easy-to-use tool for generate 3D Gazebo terrain using real-world elevation and satellite data.


<lite-youtube videoid="pxL2UF9xl_w" title="3D Gazebo terrain Generator"/>


## Installtion

### Create and Activate Virtual Environment (Recommended)

It's recommended to use a virtual environment to avoid dependency conflicts:

<details>
<summary><strong>For Linux/macOS</strong></summary>

```bash
python3 -m venv venv
source venv/bin/activate
```

</details>

<details>
<summary><strong>For Windows</strong></summary>

```bash
python -m venv venv
venv\Scripts\activate
```

</details>

### Install Requirements

Make sure your virtual environment is active, then install all required Python packages using:
  ```bash
  pip install -r requirements.txt
  ```

## Run Gazebo world Generator
1. Navigate to the `gazebo_terrian_generator` repository:
    ```bash
    cd gazebo_terrian_generator/scripts
    python server.py
    ```

2. To access application open up your web browser and navigate to `http://localhost:8080`.
3. Gazebo world generated are stored inside `output/gazebo_terrian/` by default. Feel free to change the path defined in `scripts/utils/param.py` as per you choice.

## Spawning the gazebo world
1. Export gazebo resource path.
    ```bash
    export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/<your model path here>
    ```

2. Run Gazebo with the required world file.
    ```bash
    gz sim <path of your  world file>
    ```


**Note:** Replace path with your actual path of the world file.


## Spawing sample worlds Example:

1. Export the gazebo model path.
    ```bash
    GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/gazebo_terrian_generator/sample_worlds
    ```
2. Spawing gazebo world.
    ```bash
    gz sim prayag/prayag.sdf
    ```

:::warning
For more information please refer [github](https://github.com/saiaravind19/gazebo_terrain_generator)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/saiaravind19/gazebo_terrain_generator)
:::
