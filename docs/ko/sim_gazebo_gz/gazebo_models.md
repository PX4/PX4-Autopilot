# Gazebo Models Repository (PX4-gazebo-models)

The [PX4-gazebo-models](https://github.com/PX4/PX4-gazebo-models) repository is used to store all [Gazebo](../sim_gazebo_gz/index.md) models and worlds that are supported by PX4.

- Models are stored in the `/models` directory and worlds are stored in the `/worlds` directory.
- The [simulation-gazebo](https://github.com/PX4/PX4-gazebo-models/blob/main/simulation-gazebo) Python script is used for [starting Gazebo in standalone mode](../sim_gazebo_gz/index.md#standalone-mode).

The `PX4-gazebo-models` repository is included in PX4 as a submodule, and all models are available by default when using the "normal" `make` targets, such as `make px4_sitl gz_x500`.

For Gazebo [standalone simulations](../sim_gazebo_gz/index.md#standalone-mode) you first have obtain the [simulation-gazebo](https://github.com/PX4/PX4-gazebo-models/blob/main/simulation-gazebo) Python script, and then it will fetch the models and worlds to `~/.simulation-gazebo` if that directory is not present.

## simulation-gazebo (Standalone Simulation Start-up Script)

The [simulation-gazebo](https://github.com/PX4/PX4-gazebo-models/blob/main/simulation-gazebo) Python script is used for starting Gazebo in [standalone mode](../sim_gazebo_gz/index.md#standalone-mode).
The script can communicate with a PX4 SITL instance on the same host by default.
If the script arguments are set correctly, it can also communicate with any PX4 instance on any machine within the same network.

The `simulation-gazebo` script does not require any additional libraries and should work out of the box.

### 기본 사용법

The default `simulation-script` can be run with:

```sh
python simulation-gazebo
```

This will fetch the models and worlds from the [PX4 gazebo models repository](https://github.com/PX4/PX4-gazebo-models) into subfolders of the `~/.simulation-gazebo` directory the first time it is called (or more precisely, if the directory is not detected).
A _gz-server_ instance will then be launched using the default grey plane world.

The build system won't automatically update the local copy again if the `.simulation-gazebo` folder is detected, but you can force it to update to the latest models and vehicles by passing the `overwrite` flag to the script.
The resulting command will look something like:

```sh
python simulation-gazebo --overwrite
```

You can connect a PX4-enabled vehicle to an instance of _gz-server_ using several approaches:

- In a new terminal, run PX4 using `PX4_GZ_STANDALONE=1 make px4_sitl gz_<vehicle>` and you will observe a vehicle appearing in Gazebo.

- Gazebo also has its own, built-in "resource spawner".
  It can be called up by clicking on the three dots in the top right corner of the Gazebo GUI.
  There enter "resource spawner" and, under "Fuel resources", add the owner "px4".
  You can then drag and drop any PX4 model into your simulation.

  ::: info
  These models are taken from an web-server called [Gazebo Fuel](https://app.gazebosim.org/dashboard), which essentially acts as an online database for all types of models and worlds that can be launched in Gazebo.

:::

  After dropping the vehicle of your choice into Gazebo, launch PX4 SITL with:

  ```sh
  PX4_SYS_AUTOSTART=<airframe-number-of-choice> PX4_GZ_MODEL_NAME=<vehicle-of-choice> ./build/px4_sitl_default/bin/px4`
  ```

  This will connect PX4 SITL to the running instance of Gazebo.

All the functionality and flexibility that exists for PX4 is applicable and directly works in the Gazebo instance.

### Command line arguments

The following arguments can be passed to the `simulation-gazebo` script:

- `--world`
  A string variable that names the sdf file which runs the simulation world.
  Default argument is "default", which links to the default world.

- `--gz_partition`
  A string variable that sets the gazebo partition to run in (more information [here](https://gazebosim.org/api/transport/13/envvars.html))

- `--gz_ip`
  A string variable that sets the IP of the outgoing network interface (more information [here](https://gazebosim.org/api/transport/13/envvars.html))

- `--interactive` A boolean variable that requires the ability to run the code in interactive mode, allowing for custom paths for `--model_download_source`.
  If this is not set, `--model_download_source` will only download from the default Github repo.

- `--model_download_source`
  A string variable setting the path to a directory from where models are to be imported.
  At the moment this can only be a local file directory or a http address.
  The source should end with the zipped model file (for example: https://path/to/simulation/models/models.zip).

- `--model_store`
  A string variable setting the path to the model storage directory.
  This is where the zip file provided in `model_download_source` will be placed.

- `--overwrite`
  A boolean variable indicating that the `.simulation-gazebo` should be updated with world and vehicle data from the gazebo model repository.

- `--dryrun`
  A boolean variable that can be set when running test cases.
  It will not provide any interactivity and will not start a gz-server instance.

None of these arguments are required for `simulation-gazebo` to work.
They are needed when you want to provide custom model downloads, other worlds, or you want to run Gazebo and PX4 on separate hosts.

## Example: Running One Host with Multiple Terminals

This example explains how you can run standalone mode PX4 via two terminals on one host.

1. In one terminal run

  ```sh
  PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=windy ./build/px4_sitl_default/bin/px4
  ```

2. In a second terminal window run:

  ```sh
  python3 /path/to/simulation-gazebo --world windy
  ```

No additional parameters have to be passed to the simulation-gazebo script in order for this example to work, as all Gazebo nodes run on the same host.
See the example below for a more involved scenario with different hosts.

## Example: Running Multiple Hosts

The following example will illustrate how you can set up a distributed system, running PX4 on one host on a network (called "PX4-host") and Gazebo on another (called "Gazebo-host").
This will result in two Gazebo nodes running on two different hosts in the same network, and communicating using the `gz-transport` library.

We first have to figure out what IP address we can use to send out messages on both hosts.

On the PX4-host, run the following commands:

```sh
sudo apt update
sudo apt install iproute2
```

Then type:

```sh
ip a
```

If you are connected to a network via WiFi, then the desired address will usually have a name along the lines of `wlp12345`.
Record the IPv4 address for this interface by noting the number listed next to `inet`.
It should be four numbers, separated by points and followed by a slash and another number.
A valid IPv4 address would be for example: `192.168.24.89/24`.

You only require the first four numbers.
So in the example's case, the IP address of the PX4-host would be: `192.168.24.89`.
Note that if you are connected via Ethernet the network interface might start with `eth` or `en`, however this is not standardized.

Repeat the same procedure on the Gazebo-host and note down the second IPv4 address.
For this example we will take `192.168.24.107`.

We can now start setting up both hosts.
We first set up the PX4-host:

```sh
GZ_PARTITION=relay GZ_RELAY=192.168.24.107 GZ_IP=192.168.24.89 PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_WORLD=baylands ./build/px4_sitl_default/bin/px4
```

Explanation of the environment variables:

- `GZ_PARTITION` declares the partition name that the two gazebo nodes will run in.
  This **has** to be same across all connected notes.
- `GZ_RELAY` points to the target IP address on the other host (in this case the Gazebo-host).
  This environment variable is necessary to connect the two nodes to each other.
  Note that this connection is bidirectional, so `GZ_RELAY` only has to be set on one host.
- `GZ_IP` tells the current host what network interface to use to send out messages.
  This is required when advertising topics or services.

We can then set up the Gazebo-host.
Note that the actual setup order (PX4-host first or Gazebo-host first) is not actually important.
Both will continuously look for other Gazebo nodes until they find one.
On your Gazebo-host, in a terminal, run:

```sh
python3 /path/to/simulation-gazebo --gz_partition relay --gz_ip 192.168.24.107 --world baylands
```

Here we pass same environmental variables as arguments.
Note that the `--world` values must be the same in order to be able to connect.
If you accidentally set `baylands` in one host and say `default` in the other, then the two nodes will not be able to connect.

If everything worked correctly, then the two hosts should now be connected and you should be able to fly your vehicle in the command line on the PX4-host.

Furthermore, you could also set up QGC and fly your vehicle that way.
In addition it is also possible to connect multiple PX4-hosts to the same Gazebo-host by setting the `-i` flag as shown on the [multi-vehicle simulation](./multi_vehicle_simulation.md) page.
For more information, concerning the environment variables, you can also refer to the [gz-transport documentation](https://gazebosim.org/api/transport/12/envvars.html).

A connection over VPN (and hence multiple networks) is also possible, but is currently not documented.

## Workflow

When a branch gets merged onto the main branch (this could be a model addition, deletion or a something like a parameter change), all models that have received any sort of change will automatically be updated and uploaded to the PX4 account on [Gazebo Fuel](https://app.gazebosim.org/PX4).
Furthermore, there is also a workflow that can be used to check the validity of any provided sdf file.
