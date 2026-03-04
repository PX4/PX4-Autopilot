# 동영상 스트리밍 (보조 컴퓨터/QGroundControl)

PX4-based vehicles support video streaming using a camera connected to a [companion computer](../companion_computer/index.md).

:::info
You can't video stream directly from a camera connected to PX4.
:::

GStreamer is used to send the video to _QGroundControl_ over an IP link.
To support streaming use cases you will need to install _GStreamer_ development packages on both your companion computer and on the system running _QGroundControl_.
_QGroundControl_ uses GStreamer 1.18.1 and a stripped down version of _QtGstreamer_ to support UDP RTP and RSTP video streaming.

## 보조 컴퓨터 설정

You will need to install the _GStreamer_ packages that match those used by QGC.

For a Ubuntu companion, a minimal set might be:

```sh
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```

For the full set you can mirror the QGC dependencies installed by [/tools/setup/install-dependencies-debian.sh](https://github.com/mavlink/qgroundcontrol/blob/master/tools/setup/install-dependencies-debian.sh).
At time of writing this is:

```sh
DEBIAN_FRONTEND=noninteractive apt-get -y --quiet install \
    libgstreamer1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstreamer-gl1.0-0 \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-plugins-rtp \
    gstreamer1.0-gl \
    gstreamer1.0-libav \
    gstreamer1.0-rtsp \
    gstreamer1.0-vaapi \
    gstreamer1.0-x
```

:::tip
For other companion platforms you can mirror the instructions in the [corresponding installation scripts](https://github.com/mavlink/qgroundcontrol/tree/master/tools/setup).
:::

General instructions for starting the stream on a companion computer are provided in the [QGroundControl VideoReceiver GStreamer README](https://github.com/mavlink/qgroundcontrol/blob/master/src/VideoManager/VideoReceiver/GStreamer/README.md).

The setup of cameras and data links depends on many factors.
Examples in this library are listed below (note, these are options, not "recommended"):

- [Video Streaming using WFB-ng Wifi](../companion_computer/video_streaming_wfb_ng_wifi.md) (Tutorial): Using RPi and WiFi module in unconnected (broadcast) mode to stream video and as a bidirectional telemetry link.

## QGC 설정

QGC로 비디오 스트리밍을 설정하고 사용하려면 :

1. Install GStreamer if using Ubuntu QGC 4 release builds (or earlier)

   ::: tip
   All other QGroundControl builds include GStreamer binaries, including daily builds, release builds after version 4, and builds for other platforms include GStreamer binaries.

:::

   Install the runtime dependencies using the command:

   ```sh
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
   ```

2. Start QGC

3. Enable video in _Fly View_: [QGroundControl > General Settings (Settings View) > Video](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/general.html#video)

4. 모든 것이 정상적으로 작동하게 되면, QGC 비디오 스위처 (QGC Fly View 왼쪽 하단 모서리)에 비디오 스트림이 표시됩니다.
   아래 스크린 샷과 같이 비디오 스위처를 클릭하여 비디오를 전체 화면으로 전환  수 있습니다.

   ![QGC displaying video stream](../../assets/videostreaming/qgc-screenshot.png)

## Gazebo Classic Simulation

[Gazebo Classic](../sim_gazebo_classic/index.md) supports video streaming from within the simulated environment.
For more information see [Gazebo Classic Simulation > Video Streaming](../sim_gazebo_classic/index.md#video-streaming).
