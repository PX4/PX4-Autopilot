# PX4 도커 컨테이너

Docker containers are provided for the complete [PX4 development toolchain](../dev_setup/dev_env.md#supported-targets) including NuttX and Linux based hardware, [Gazebo Classic](../sim_gazebo_classic/index.md) simulation, and [ROS](../simulation/ros_interface.md).

This topic shows how to use the [available docker containers](#px4_containers) to access the build environment in a local Linux computer.

:::info
Dockerfiles and README can be found on [Github here](https://github.com/PX4/PX4-containers/tree/master?tab=readme-ov-file#container-hierarchy).
They are built automatically on [Docker Hub](https://hub.docker.com/u/px4io/).
:::

## 준비 사항

:::info
PX4 containers are currently only supported on Linux (if you don't have Linux you can run the container [inside a virtual machine](#virtual_machine)).
Do not use `boot2docker` with the default Linux image because it contains no X-Server.
:::

[Install Docker](https://docs.docker.com/installation/) for your Linux computer, preferably using one of the Docker-maintained package repositories to get the latest stable version. You can use either the _Enterprise Edition_ or (free) _Community Edition_.

For local installation of non-production setups on _Ubuntu_, the quickest and easiest way to install Docker is to use the [convenience script](https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-using-the-convenience-script) as shown below (alternative installation methods are found on the same page):

```sh
curl -fsSL get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

The default installation requires that you invoke _Docker_ as the root user (i.e. using `sudo`). However, for building the PX4 firmware we suggest to [use docker as a non-root user](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user). 그렇게하면, docker를 사용한 후 빌드 폴더를 관리자가 소유하지 않습니다.

```sh
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
# Log in/out again before using docker!
```

<a id="px4_containers"></a>

## 컨테이너 계층

The available containers are on [Github here](https://github.com/PX4/PX4-containers/tree/master?tab=readme-ov-file#container-hierarchy).

이를 통하여 다양한 빌드 대상 및 구성을 테스트할 수 있습니다(포함된 도구는 이름에서 유추할 수 있음).
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
For example, the `px4io/px4-dev-nuttx-focal` tags can be found here).

:::tip
Typically you should use a recent container, but not necessarily the `latest` (as this changes too often).
:::

## 도커 컨테이너 활용

도커 컨테이너에서 실행되는 툴체인을 사용하여 호스트 컴퓨터에서 PX4 빌드 방법을 설명합니다.
The information assumes that you have already downloaded the PX4 source code to **src/PX4-Autopilot**, as shown:

```sh
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

### 보조 스크립트(docker_run.sh)

The easiest way to use the containers is via the [docker_run.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/docker_run.sh) helper script.
This script takes a PX4 build command as an argument (e.g. `make tests`). 적절한 컨테이너 및 합리적인 환경 설정의 최신 버전(하드 코딩됨)으로 도커를 시작합니다.

For example, to build SITL you would call (from within the **/PX4-Autopilot** directory):

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

또는 NuttX 도구 체인을 사용하여 bash 세션을 시작합니다.

```sh
./Tools/docker_run.sh 'bash'
```

:::tip
The script is easy because you don't need to know anything much about _Docker_ or think about what container to use. 그러나, 특별히 견고하지는 않습니다! The manual approach discussed in the [section below](#manual_start) is more flexible and should be used if you have any problems with the script.
:::

<a id="manual_start"></a>

### 도커 수동 호출

일반적인 명령어 구문은 다음과 같습니다.
이것은 X 포워딩을 지원하는 Docker 컨테이너를 실행합니다(컨테이너 내부에서 시뮬레이션 GUI를 사용할 수 있게 함).
It maps the directory `<host_src>` from your computer to `<container_src>` inside the container and forwards the UDP port needed to connect _QGroundControl_.
With the `-–privileged` option it will automatically have access to the devices on your host (e.g. a joystick and GPU). 장치를 연결/연결 해제하는 경우에는 컨테이너를 다시 시작하여야 합니다.

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

여기서,

- `<host_src>`: The host computer directory to be mapped to `<container_src>` in the container. This should normally be the **PX4-Autopilot** directory.
- `<container_src>`: The location of the shared (source) directory when inside the container.
- `<local_container_name>`: A name for the docker container being created. 나중에 컨테이너를 다시 참조해야 하는 경우에 사용할 수 있습니다.
- `<container>:<tag>`: The container with version tag to start - e.g.: `px4io/px4-dev-ros:2017-10-23`.
- `<build_command>`: The command to invoke on the new container. 예: `bash` is used to open a bash shell in the container.

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

:::info
We use the host network mode to avoid conflicts between the UDP port access control when using QGroundControl on the same system as the docker container.
:::

:::info
If you encounter the error "Can't open display: :0", `DISPLAY` may need to be set to a different value.
On Linux (XWindow) hosts you can change `-e DISPLAY=:0` to `-e DISPLAY=$DISPLAY`.
On other hosts you might iterate the value of `0` in `-e DISPLAY=:0` until the "Can't open display: :0" error goes away.
:::

모든 것이 잘 실행되면, 새로운 bash 쉘이 실행됩니다.
예를 들어 SITL을 실행하여 모든 것이 작동하는 지 확인하십시오.

```sh
cd src/PX4-Autopilot    #This is <container_src>
make px4_sitl_default gazebo-classic
```

### 컨테이너 재진입

The `docker run` command can only be used to create a new container. 변경 사항을 유지하는 이 컨테이너로 돌아가려면 다음을 실행하십시오.

```sh
# start the container
docker start container_name
# open a new bash shell in this container
docker exec -it container_name bash
```

컨테이너에 연결된 여러 셸이 필요한 경우에는 새 셸을 열고 마지막 명령을 다시 실행합니다.

### 컨테이너 정리

때로는 컨테이너를 완전히 비워야 합니다. 이름을 사용하여 정리할 수 있습니다.

```sh
docker rm mycontainer
```

이름이 기억나지 않으면, 아래와 같이 비활성 컨테이너 ID를 나열한 다음 삭제합니다.

```sh
docker ps -a -q
45eeb98f1dd9
docker rm 45eeb98f1dd9
```

### QGroundControl

When running a simulation instance e.g. SITL inside the docker container and controlling it via _QGroundControl_ from the host, the communication link has to be set up manually. The autoconnect feature of _QGroundControl_ does not work here.

In _QGroundControl_, navigate to [Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_view.html) and select Comm Links. ::: The port depends on the used [configuration](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/rcS) e.g. port 14570 for the SITL config. IP 주소는 도커 컨테이너 중 하나이며, 기본 네트워크는 172.17.0.1/16입니다. The IP address of the docker container can be found with the following command (assuming the container name is `mycontainer`):

```sh
$ docker inspect -f '{ {range .NetworkSettings.Networks}}{ {.IPAddress}}{ {end}}' mycontainer
```

:::info
Spaces between double curly braces above should be not be present (they are needed to avoid a UI rendering problem in gitbook).
:::

### 문제 해결

#### 권한 에러

컨테이너는 기본 사용자(일반적으로 "루트") 계정으로 파일을 생성합니다. 이것 때문에, 호스트 컴퓨터의 사용자가 컨테이너에서 생성한 파일에 액세스할 수 없는 상황이 발생합니다.

The example above uses the line `--env=LOCAL_USER_ID="$(id -u)"` to create a user in the container with the same UID as the user on the host. 이렇게 하면 컨테이너 내에서 생성된 모든 파일을 호스트에서 액세스할 수 있습니다.

#### 그래픽 드라이버 문제

It's possible that running Gazebo Classic will result in a similar error message like the following:

```sh
libGL error: failed to load driver: swrast
```

이 경우 호스트 시스템의 기본 그래픽 드라이버를 설치합니다. 올바른 드라이버를 다운로드하여 컨테이너 내부에 설치합니다. Nvidia 드라이버의 경우 다음 명령어를 사용합니다(그렇지 않으면 설치 프로그램이 호스트에서 로드된 모듈을 보고 진행을 거부합니다).

```sh
./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
```

More information on this can be found [here](http://gernotklingler.com/blog/howto-get-hardware-accelerated-opengl-support-docker/).

<a id="virtual_machine"></a>

## 가상 머신 지원

최신 Linux 배포판에서는 정상적으로 작동하여야 합니다.

다음 설정은 테스트 되었습니다.

- VMWare Fusion 및 Ubuntu 14.04가 포함된 OS X(Parallels에서 GUI를 지원하는 Docker 컨테이너로 인해 X-Server가 충돌함).

**Memory**

가상 머신에 최소 4GB 메모리를 사용하십시오.

**Compilation problems**

다음과 같은 오류로 컴파일이 실패하는 경우:

```sh
The bug is not reproducible, so it is likely a hardware or OS problem.
c++: internal compiler error: Killed (program cc1plus)
```

병렬 빌드를 비활성화하십시오.

**Allow Docker Control from the VM Host**

Edit `/etc/defaults/docker` and add this line:

```sh
DOCKER_OPTS="${DOCKER_OPTS} -H unix:///var/run/docker.sock -H 0.0.0.0:2375"
```

이제 호스트 운영체제에서 도커를 제어할 수 있습니다:

```sh
export DOCKER_HOST=tcp://<ip of your VM>:2375
# run some docker command to see if it works, e.g. ps
docker ps
```
