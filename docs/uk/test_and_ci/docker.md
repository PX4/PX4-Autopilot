# Docker контейнери для PX4

Docker containers are provided for the complete [PX4 development toolchain](../dev_setup/dev_env.md#supported-targets) including NuttX and Linux based hardware, [Gazebo Classic](../sim_gazebo_classic/index.md) simulation, and [ROS](../simulation/ros_interface.md).

This topic shows how to use the [available docker containers](#px4_containers) to access the build environment in a local Linux computer.

:::info
Dockerfiles and README can be found on [Github here](https://github.com/PX4/PX4-containers/tree/master?tab=readme-ov-file#container-hierarchy).
They are built automatically on [Docker Hub](https://hub.docker.com/u/px4io/).
:::

## Вимоги

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

The default installation requires that you invoke _Docker_ as the root user (i.e. using `sudo`). However, for building the PX4 firmware we suggest to [use docker as a non-root user](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user). Таким чином, директорія для збірки не буде належати користувачу root після використання docker.

```sh
# Create docker group (may not be required)
sudo groupadd docker
# Add your user to the docker group.
sudo usermod -aG docker $USER
# Log in/out again before using docker!
```

<a id="px4_containers"></a>

## Ієрархія контейнерів

The available containers are on [Github here](https://github.com/PX4/PX4-containers/tree/master?tab=readme-ov-file#container-hierarchy).

Вони дозволяють тестувати різні цілі збірки та конфігурації (включені інструменти можна зрозуміти з їх назв).
Контейнери є ієрархічними, тобто такими, що мають функціональність вихідних контейнерів.
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

## Використання Docker контейнера

Наступні інструкції показують, як зібрати вихідний код PX4 на основному комп'ютері за допомогою інструментарію, що працює у docker контейнері.
The information assumes that you have already downloaded the PX4 source code to **src/PX4-Autopilot**, as shown:

```sh
mkdir src
cd src
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

### Допоміжний скрипт (docker_run.sh)

The easiest way to use the containers is via the [docker_run.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/docker_run.sh) helper script.
This script takes a PX4 build command as an argument (e.g. `make tests`). Він запускає docker із найновішою версією відповідного контейнера (вказано в коді) і слушними налаштуваннями середовища.

For example, to build SITL you would call (from within the **/PX4-Autopilot** directory):

```sh
./Tools/docker_run.sh 'make px4_sitl_default'
```

Або почати сеанс bash використовуючи інструментарій NuttX:

```sh
./Tools/docker_run.sh 'bash'
```

:::tip
The script is easy because you don't need to know anything much about _Docker_ or think about what container to use. Однак він не дуже надійний! The manual approach discussed in the [section below](#manual_start) is more flexible and should be used if you have any problems with the script.
:::

<a id="manual_start"></a>

### Запуск Docker вручну

Синтаксис типової команди показано нижче.
Це запускає Docker контейнер з підтримкою переадресації X (що робить графічний інтерфейс симуляції доступним з середини контейнера).
It maps the directory `<host_src>` from your computer to `<container_src>` inside the container and forwards the UDP port needed to connect _QGroundControl_.
With the `-–privileged` option it will automatically have access to the devices on your host (e.g. a joystick and GPU). Якщо ви під'єднуєте/від'єднуєте пристрій, вам слід перезапустити контейнер.

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

Де:

- `<host_src>`: The host computer directory to be mapped to `<container_src>` in the container. This should normally be the **PX4-Autopilot** directory.
- `<container_src>`: The location of the shared (source) directory when inside the container.
- `<local_container_name>`: A name for the docker container being created. Це потім можна використовувати, якщо потрібно посилатись на контейнер знову.
- `<container>:<tag>`: The container with version tag to start - e.g.: `px4io/px4-dev-ros:2017-10-23`.
- `<build_command>`: The command to invoke on the new container. Наприклад, `bash` is used to open a bash shell in the container.

The concrete example below shows how to open a bash shell and share the directory **~/src/PX4-Autopilot** on the host computer.

```sh
# дозвольте доступ до xhost з контейнера
xhost +

# запуск docker та оболонки bash
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

Якщо все пройшло добре, ви повинні бути в новій оболонці bash.
Перевірте, чи все працює запустивши, наприклад, SITL:

```sh
cd src/PX4-Autopilot    #This is <container_src>
make px4_sitl_default gazebo-classic
```

### Повторний вхід в контейнер

The `docker run` command can only be used to create a new container. Щоб повернутися у цей контейнер (що збереже ваші зміни) просто зробіть:

```sh
# запуск контейнера
docker start container_name
# запуск нової оболонки bash shell в цьому контейнері
docker exec -it container_name bash
```

Якщо вам потрібні кілька консолей, підключених до контейнера, просто відкрийте нову оболонку і виконайте останню команду знову.

### Видалення контейнера

Іноді може знадобитися взагалі видалити контейнер. Це можна зробити, використовуючи його ім'я:

```sh
docker rm mycontainer
```

Якщо ви не можете згадати назву, ви можете знайти неактивні ідентифікатори контейнерів і видалити їх, як показано нижче:

```sh
docker ps -a -q
45eeb98f1dd9
docker rm 45eeb98f1dd9
```

### QGroundControl

When running a simulation instance e.g. SITL inside the docker container and controlling it via _QGroundControl_ from the host, the communication link has to be set up manually. The autoconnect feature of _QGroundControl_ does not work here.

In _QGroundControl_, navigate to [Settings](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/settings_view.html) and select Comm Links. Створіть новий канал, що використовує UDP-протокол. The port depends on the used [configuration](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/rcS) e.g. port 14570 for the SITL config. IP-адреса є адресою одного з ваших контейнерів, зазвичай це адреса з мережі 172.17.0.1/16 при використанні мережі за замовчуванням. The IP address of the docker container can be found with the following command (assuming the container name is `mycontainer`):

```sh
$ docker inspect -f '{ {range .NetworkSettings.Networks}}{ {.IPAddress}}{ {end}}' mycontainer
```

:::info
Spaces between double curly braces above should be not be present (they are needed to avoid a UI rendering problem in gitbook).
:::

### Усунення проблем

#### Помилки з правами доступу

Контейнер створює файли, необхідні для роботи від імені стандартного користувача, як правило, "root". Це може призвести до помилок прав доступу, коли користувач на основному комп'ютері не має доступу до файлів, створених контейнером.

The example above uses the line `--env=LOCAL_USER_ID="$(id -u)"` to create a user in the container with the same UID as the user on the host. Це гарантує, що всі файли, створені у контейнері, будуть доступні з основного комп'ютера.

#### Проблеми з драйверами графіки

Можливо, що запуск Gazebo Classic призведе до подібного повідомлення про помилку:

```sh
libGL error: failed to load driver: swrast
```

У цьому випадку необхідно встановити нативний графічний драйвер для вашої системи. Завантажте відповідний драйвер і встановіть його всередині контейнера. Для драйверів Nvidia слід використовувати наступну команду (інакше встановлювач побачить завантажені модулі на головній машині та відмовиться продовжувати):

```sh
./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module
```

More information on this can be found [here](http://gernotklingler.com/blog/howto-get-hardware-accelerated-opengl-support-docker/).

<a id="virtual_machine"></a>

## Підтримка віртуальних машин

Будь-який останній дистрибутив Linux повинен працювати.

Наступна конфігурація протестована:

- OS X з підтримкою VMWare Fusion і Ubuntu 14.04 (Docker контейнер з підтримкою GUI в Parallels призводить до падіння X-Server).

**Memory**

Потрібно не менше 4 ГБ пам'яті для віртуальної машини.

**Compilation problems**

Якщо компіляція завершується з помилками на кшталт:

```sh
The bug is not reproducible, so it is likely a hardware or OS problem.
c++: internal compiler error: Killed (program cc1plus)
```

Спробуйте вимкнути паралельну збірку.

**Allow Docker Control from the VM Host**

Edit `/etc/defaults/docker` and add this line:

```sh
DOCKER_OPTS="${DOCKER_OPTS} -H unix:///var/run/docker.sock -H 0.0.0.0:2375"
```

Тепер можна керувати docker на вашій основній ОС:

```sh
export DOCKER_HOST=tcp://<ip of your VM>:2375
# run some docker command to see if it works, e.g. ps
docker ps
```
