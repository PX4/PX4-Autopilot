# Gazebo Container GUI

The [Gazebo runtime and ROS development images](px4_sitl_prebuilt_packages.md) can display the Gazebo Harmonic GUI through a Linux X server or Windows WSLg.
Both use the same X11 client libraries and Qt configuration.
The X server runs outside the application container; installing Xvfb inside the image would not forward a window to your desktop.

The examples use software rendering to avoid depending on GPU passthrough.
Gazebo still renders the 3D scene, but performance depends on the CPU.
Leave `HEADLESS` unset or empty: `HEADLESS=0` also disables the GUI.

## Linux

Use Docker Engine from a local graphical-session terminal, with `DISPLAY` set and the host's `xauth` command installed.
Xorg and Wayland desktops with XWayland are supported by this X11 route.
The following command copies only the current display's authentication records into a private temporary file:

```bash
(
  set -euo pipefail
  umask 077
  : "${DISPLAY:?Open a terminal in your local graphical session}"
  IMAGE="${IMAGE:-px4io/px4-sitl-gazebo:latest}"
  auth=$(mktemp)
  trap 'rm -f "$auth"' EXIT

  xauth nlist "$DISPLAY" |
    sed 's/^..../ffff/' |
    xauth -f "$auth" nmerge -
  if [ ! -s "$auth" ]; then
    echo "No Xauthority cookie for $DISPLAY; check the host XAUTHORITY setting." >&2
    exit 1
  fi

  docker run --rm -it --name px4-gazebo --network host \
    --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
    --mount "type=bind,src=$auth,dst=/tmp/px4.xauth,readonly" \
    -e "DISPLAY=$DISPLAY" -e XAUTHORITY=/tmp/px4.xauth \
    -e QT_QPA_PLATFORM=xcb -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e ROS_DOMAIN_ID=83 \
    "$IMAGE"
)
```

The copied cookie uses a wildcard address family so the container's hostname does not need to match the host's.
Authentication remains enabled.
Only give this cookie to trusted images: it grants access to your X session.
Do not use `xhost +`, `--privileged` or shared host IPC.
The example assumes ordinary rootful Docker; rootless or user-namespace-remapped installations may require different file ownership, not world-readable cookie permissions.

### GPU Acceleration

Once software rendering works, remove `-e LIBGL_ALWAYS_SOFTWARE=1` and add the appropriate options before the image name:

| Linux GPU                                                | Docker Options                                                              |
| -------------------------------------------------------- | --------------------------------------------------------------------------- |
| Intel/AMD with DRM render devices                        | `--device=/dev/dri`                                                         |
| NVIDIA with the host driver and NVIDIA Container Toolkit | `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display` |

NVIDIA's default compute/utility capabilities alone do not provide the graphics/display libraries.
See the [NVIDIA Container Toolkit documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html#driver-capabilities).
GPU access remains opt-in; the images do not force software rendering globally.

## Windows With WSLg

Use Windows 11, an up-to-date WSL2 Ubuntu distribution, and Docker Desktop's Linux-container backend with [WSL integration](https://docs.docker.com/desktop/features/wsl/) enabled for that distribution.
Run the commands below from its Ubuntu terminal, not PowerShell.
WSLg supplies the X server; a separate Windows X server is not required.

```bash
(
  set -euo pipefail
  IMAGE="${IMAGE:-px4io/px4-sitl-gazebo:latest}"
  if [ "${DISPLAY:-}" != :0 ] || [ ! -S /mnt/wslg/.X11-unix/X0 ]; then
    echo "WSLg display unavailable; check WSLg and Docker Desktop WSL integration." >&2
    exit 1
  fi

  docker run --rm -it --name px4-gazebo \
    --mount type=bind,src=/mnt/wslg/.X11-unix,dst=/tmp/.X11-unix,readonly \
    -e DISPLAY=:0 -e QT_QPA_PLATFORM=xcb \
    -e LIBGL_ALWAYS_SOFTWARE=1 -e ROS_DOMAIN_ID=83 \
    "$IMAGE"
)
```

This follows [Microsoft's X11 container interface](https://github.com/microsoft/wslg/blob/main/samples/container/Containers.md), using the resolved WSLg socket directory.
The recipe has not yet been exercised on a Windows host with these images.
Standard WSLg does not require creating an Xauthority file or changing `xhost` access rules.
Do not replace its `DISPLAY` with a Windows host IP from older X-server instructions.
Do not forward `WAYLAND_DISPLAY`: this recipe deliberately selects Qt's XCB backend.

If Docker cannot bind the socket, confirm integration with the Ubuntu distribution and follow [WSLg display diagnostics](https://github.com/microsoft/wslg/wiki/Diagnosing-%22cannot-open-display%22-type-issues-with-WSLg).
The software-rendering example does not require `/dev/dxg` or Windows GPU passthrough.
Windows GPU acceleration has additional driver, Mesa and Docker-backend requirements; it is not implied by Docker's support for GPU compute.

## ROS Development Image

Before running either platform's example, select the ROS variant:

```bash
export IMAGE=px4io/px4-sitl-gazebo-ros2:latest
```

It opens a ROS-enabled shell instead of starting PX4 automatically.
In that shell, start the Agent:

```sh
MicroXRCEAgent udp4 -p 8888
```

From a second host terminal, start PX4 and the Gazebo GUI:

```sh
docker exec -it px4-gazebo /usr/local/bin/ros2-entrypoint.sh px4-gazebo
```

The [ROS topic and Go-to example commands](px4_sitl_prebuilt_packages.md#go-to-example) also apply; use `px4-gazebo` as the container name.
Use distinct `ROS_DOMAIN_ID` values for independent simulations.

## Diagnostics

While the container is running, inspect the OpenGL implementation:

```sh
docker exec px4-gazebo glxinfo -B
```

Software rendering should report a renderer such as `llvmpipe`.
Ogre 2 needs a working OpenGL 3.3-class-or-newer implementation and its required extensions; upstream recommends OpenGL 4.3 or newer.
See [Gazebo's rendering troubleshooting](https://gazebosim.org/docs/harmonic/troubleshooting/).
Successfully displaying a simple X11 window is not sufficient evidence that Gazebo can render.

If PX4 starts but the window does not appear, launch the GUI with visible diagnostics:

```sh
docker exec px4-gazebo gz sim -g -v 4
```

For XCB plugin loading failures, add `-e QT_DEBUG_PLUGINS=1` to that `docker exec` command.
The images include Qt SVG support for GUI icons and set `QT_X11_NO_MITSHM=1` because the container and host X server do not share an IPC namespace.

## macOS And Linux Virtual Machines

Gazebo containers on macOS are supported headlessly, using `-e HEADLESS=1`.
XQuartz 2.8.5 accepted an authenticated X11 connection in our evaluation, but its indirect OpenGL path failed to render Gazebo with both Ogre 2 and Ogre 1.
Changing the Qt backend to software did not provide a working fallback.
The limitation was rendering, not an inability to speak X11.
For SIH, use [native Hawkeye](../sim_hawkeye/index.md) instead.

A Linux desktop VM is another way to use and evaluate the Linux GUI path from macOS.
On Apple Silicon, use an Ubuntu 24.04 ARM64 guest and ARM64 container images.
Run Docker and the Linux commands above inside the guest, initially with software rendering.
The GUI appears on the VM's desktop without passing through XQuartz.
An isolated Linux Xvfb server can also exercise socket forwarding, authorization and Mesa rendering automatically, but does not validate desktop integration or physical GPU drivers.
