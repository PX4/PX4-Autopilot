# GStreamer Camera System Plugin for Gazebo Harmonic

This plugin provides GStreamer-based video streaming capabilities for cameras in Gazebo Harmonic simulation.

## Features

- Automatically discovers camera sensors in the simulation and selects the first.
- Streams camera feed via UDP (RTP/H.264) or RTMP
- Support for hardware acceleration with NVIDIA GPUs (CUDA)
- Low-latency streaming

## Prerequisites

- GStreamer 1.0 with development files
- NVIDIA drivers and CUDA (optional, for hardware acceleration)

## Configuration

The plugin can be configured with the following parameters in the SDF model file:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| udpHost | string | 127.0.0.1 | Destination IP address for UDP streaming |
| udpPort | int | 5600 | Destination port for UDP streaming |
| rtmpLocation | string | | RTMP URL for streaming. If provided, RTMP will be used instead of UDP |
| useCuda | bool | true | Enable NVIDIA hardware acceleration |

## Viewing Streams

### UDP Stream

For UDP streams, you can use GStreamer to view the video feed:

```bash
gst-launch-1.0 udpsrc port=5600 \
    ! application/x-rtp,encoding-name=H264,payload=96 \
    ! rtph264depay \
    ! avdec_h264 \
    ! videoconvert \
    ! autovideosink
```

### RTMP Stream

For RTMP streams, you can use any RTMP-compatible player, such as:

- VLC: Media > Open Network Stream > rtmp://your-rtmp-url
- ffplay: `ffplay rtmp://your-rtmp-url`

## Multi-vehicle simulation

The port specified in the SDF file is used as a base port. Each instance gets base_port + instance_index:
| Instance | Port | 
|----------|------|
| 0        | 5600 | 
| 1        | 5601 |
| 2        | 5602 |

Note: ports are assigned by the GStreamer plugin in launch order, not by the -i flag value. Always start instances sequentially: 0, then 1, then 2, etc.
Restarting an individual instance preserves its port — e.g. a restarted instance 1 will use port 5601 again.

*Warning:* RTMP streaming for multi-vehicle simulation is currently not supported.

## Environment Variables

- `PX4_VIDEO_HOST_IP`: Can be set to override the default UDP destination IP

## License

BSD 3-Clause
