# PX4 Docker Containers

Docker containers are provided for the complete [PX4 development toolchain](../dev_setup/dev_env.md#supported-targets) including NuttX and Linux based hardware, [Gazebo Classic](../sim_gazebo_classic/index.md) simulation, and [ROS](../simulation/ros_interface.md).

This topic shows how to use the [available docker containers](#px4_containers) to access the build environment in a local Linux computer.

::: info
Dockerfiles and README can be found on [Github here](https://github.com/PX4/PX4-containers/tree/master?tab=readme-ov-file#container-hierarchy).
They are built automatically on [Docker Hub](https://hub.docker.com/u/px4io/).
:::

## Prerequisites

::: info
PX4 containers are currently only supported on Linux (if you don't have Linux you can run the container [inside a virtual machine](#virtual_machine)).
Do not use `boot2docker` with the default Linux image because it contains no X-Server.
:::

[Install Docker](https://docs.docker.com/installation/) for your Linux computer, preferably using one of the Docker-maintained package repositories to get the latest stable version. You can use either the _Enterprise Edition_ or (free) _Community Edition_.

For local installation of non-production setups on _Ubuntu_, the quickest and easiest way to install Docker is to use the [convenience script](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-convenience-script) as shown below (alternative installation methods are found on the same page):

```sh
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

The default installation requires that you invoke _Docker_ as the root user (i.e. using `sudo`). However, for building the PX4 firmware we suggest to [use docker as a non-root user](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user). That way, your build folder won't be owned by root after using docker.

```sh
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
# Log in/out again before using docker!
```

<a id="px4_containers"></a>

## Container Hierarchy

The available containers are on [Github here](https://github.com/PX4/PX4-containers/tree/master?tab=readme-ov-file#container-hierarchy).

These allow testing of various build targets and configurations (the included tools can be inferred from their names).
The containers are hierarchical, such that containers have the functionality of their parents.
For example, the partial hierarchy below shows that the docker container with nuttx build tools (`px4-dev-nuttx-focal`) does not include ROS 2, while the simulation containers do:

```plain
- px4io/px4-dev-base-focal
  - px4io/px4-dev-nuttx-focal
  - px4io/px4-dev-simulation-focal
    - px4io/px4-dev-ros-noetic
      - px4io/px4-dev-ros2-foxy
  - px4io/px4-dev-ros2-rolling
- px4io/px4-dev-base-jammy
  - px4io/px4-dev-nuttx-jammy
```

The most recent version can be accessed using the `latest` tag: `px4io/px4-dev-nuttx-focal:latest`
(available tags are listed for each container on _hub.docker.com_.
For example, the `px4io/px4-dev-nuttx-focal` tags can be found [here](https://hub.docker.com/r/px4io/px4-dev-nuttx-focal/tags?page=1&ordering=last_updated)).

:::tip
Typically you should use a recent container, but not necessarily the `latest` (as this changes too often).
:::

## Use the Docker Container

The following instructions show how to build PX4 source code on the host computer using a toolchain running in a docker container.
The information assumes that you have already downloaded the PX4 source code to **src/PX4-Autopilot**, as shown:

```sh
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

### Helper Script (docker_run.sh)

The easiest way to use the containers is via the [docker_run.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/docker_run.sh) helper script.
This script takes a PX4 build command as an argument (e.g. `make tests`). It starts up docker with a recent version (hard coded) of the appropriate container and sensible environment settings.

For example, to build SITL you would call (from within the **/PX4-Autopilot** directory):

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

Or to start a bash session using the NuttX toolchain:

```sh
./Tools/docker_run.sh 'bash'
```

:::tip
The script is easy because you don't need to know anything much about _Docker_ or think about what container to use. However it is not particularly robust! The manual approach discussed in the [section below](#manual_start) is more flexible and should be used if you have any problems with the script.
:::

<a id="manual_start"></a>

### Calling Docker Manually

The syntax of a typical command is shown below.
This runs a Docker container that has support for X forwarding (makes the simulation GUI available from inside the container).
It maps the directory `<host_src>` from your computer to `<container_src>` inside the container and forwards the UDP port needed to connect _QGroundControl_.
With the `-–privileged` option it will automatically have access to the devices on your host (e.g. a joystick and GPU). If you connect/disconnect a device you have to restart the container.

```sh
# enable access to xhost from the container
xhost +

# Run docker
docker run -it --privileged \
    --env=LOCAL_USER_ID="$(id -u)" \
    -v <host_src>:<container_src>:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    -p 14570:14570/udp \
    --name=<local_container_name> <container>:<tag> <build_command>
```

Where,

- `<host_src>`: The host computer directory to be mapped to `<container_src>` in the container. This should normally be the **PX4-Autopilot** directory.
- `<container_src>`: The location of the shared (source) directory when inside the container.
- `<local_container_name>`: A name for the docker container being created. This can later be used if we need to reference the container again.
- `<container>:<tag>`: The container with version tag to start - e.g.: `px4io/px4-dev-ros:2017-10-23`.
- `<build_command>`: The command to invoke on the new container. E.g. `bash` is used to open a bash shell in the container.

The concrete example below shows how to open a bash shell and share the directory **~/src/PX4-Autopilot** on the host computer.

```sh
# enable access to xhost from the container
xhost +

# Run docker and open bash shell
docker run -it --privileged \
--env=LOCAL_USER_ID="$(id -u)" \
-v ~/src/PX4-Autopilot:/src/PX4-Autopilot/:rw \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=:0 \
--network host \
--name=px4-ros px4io/px4-dev-ros2-foxy:2022-07-31 bash
```

::: info
We use the host network mode to avoid conflicts between the UDP port access control when using QGroundControl on the same system as the docker container.
:::

::: info
If you encounter the error "Can't open display: :0", `DISPLAY` may need to be set to a different value.
On Linux (XWindow) hosts you can change `-e DISPLAY=:0` to `-e DISPLAY=$DISPLAY`.
On other hosts you might iterate the value of `0` in `-e DISPLAY=:0` until the "Can't open display: :0" error goes away.
:::

If everything went well you should be in a new bash shell now.
Verify if everything works by running, for example, SITL:

```sh
cd src/PX4-Autopilot    #This is <container_src>
make px4_sitl_default gazebo-classic
```

### Re-enter the Container

The `docker run` command can only be used to create a new container. To get back into this container (which will retain your changes) simply do:

```sh
# start the container
docker start container_name
# open a new bash shell in this container
docker exec -it container_name bash
```

If you need multiple shells connected to the container, just open a new shell and execute that last command again.

### Clearing the Container

Sometimes you may need to clear a container altogether. You can do so using its name:

```sh
docker rm mycontainer
```

If you can't remember the name, then you can list inactive container ids and then delete them, as shown below:

```sh
docker ps -a -q
45eeb98f1dd9
docker rm 45eeb98f1dd9
```

### QGroundControl

When running a simulation instance e.g. SITL inside the docker container and controlling it via _QGroundControl_ from the host, the communication link has to be set up manually. The autoconnect feature of _QGroundControl_ does not work here.

In _QGroundControl_, navigate to [Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_view.html) and select Comm Links. Create a new link that uses the UDP protocol. The port depends on the used [configuration](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/rcS) e.g. port 14570 for the SITL config. The IP address is the one of your docker container, usually 172.17.0.1/16 when using the default network. The IP address of the docker container can be found with the following command (assuming the container name is `mycontainer`):

```sh
$ docker inspect -f '{ {range .NetworkSettings.Networks}}{ {.IPAddress}}{ {end}}' mycontainer
```

::: info
Spaces between double curly braces above should be not be present (they are needed to avoid a UI rendering problem in gitbook).
:::

### Troubleshooting

#### Permission Errors

The container creates files as needed with a default user - typically "root". This can lead to permission errors where the user on the host computer is not able to access files created by the container.

The example above uses the line `--env=LOCAL_USER_ID="$(id -u)"` to create a user in the container with the same UID as the user on the host. This ensures that all files created within the container will be accessible on the host.

#### Graphics Driver Issues

It's possible that running Gazebo Classic will result in a similar error message like the following:

```sh
libGL error: failed to load driver: swrast
```

In that case the native graphics driver for your host system must be installed. Download the right driver and install it inside the container. For Nvidia drivers the following command should be used (otherwise the installer will see the loaded modules from the host and refuse to proceed):

```sh
./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
```

More information on this can be found [here](http://gernotklingler.com/blog/howto-get-hardware-accelerated-opengl-support-docker/).

<a id="virtual_machine"></a>

## Virtual Machine Support

Any recent Linux distribution should work.

The following configuration is tested:

- OS X with VMWare Fusion and Ubuntu 14.04 (Docker container with GUI support on Parallels make the X-Server crash).

**Memory**

Use at least 4GB memory for the virtual machine.

**Compilation problems**

If compilation fails with errors like this:

```sh
The bug is not reproducible, so it is likely a hardware or OS problem.
c++: internal compiler error: Killed (program cc1plus)
```

Try disabling parallel builds.

**Allow Docker Control from the VM Host**

Edit `/etc/defaults/docker` and add this line:

```sh
DOCKER_OPTS="${DOCKER_OPTS} -H unix:///var/run/docker.sock -H 0.0.0.0:2375"
```

You can then control docker from your host OS:

```sh
export DOCKER_HOST=tcp://<ip of your VM>:2375
# run some docker command to see if it works, e.g. ps
docker ps
```
