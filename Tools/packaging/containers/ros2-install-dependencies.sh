#!/usr/bin/env bash
set -euo pipefail

rm -f /etc/apt/apt.conf.d/docker-clean
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl
curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb" \
    -o /ros2-apt-source.deb
apt-get install -y --no-install-recommends /ros2-apt-source.deb
rm /ros2-apt-source.deb
apt-get update
apt-get install -y --no-install-recommends \
    build-essential ccache cmake git ninja-build \
    python3-empy python3-pip ros-dev-tools ros-jazzy-ros-base

# Keep Ubuntu/Jazzy's Python packages while satisfying PX4's shared requirements.
# Distro-only versions (e.g. ubuntu suffixes) cannot be pip constraints.
python3 - <<'PY' > /px4-python-constraints.txt
from importlib.metadata import distributions
from packaging.version import InvalidVersion, Version

for dist in distributions():
    if not dist.metadata["Name"] or not dist.version:
        continue
    try:
        Version(dist.version)
    except InvalidVersion:
        continue
    print(f"{dist.metadata['Name']}=={dist.version}")
PY
RUNS_IN_DOCKER=true PIP_CONSTRAINT=/px4-python-constraints.txt PIP_NO_CACHE_DIR=1 \
    bash /px4-setup/ubuntu.sh --no-nuttx --no-sim-tools
rm /px4-python-constraints.txt

rosdep init
rosdep update --rosdistro jazzy
if [ "$#" -gt 0 ]; then
    # ROS setup scripts are not nounset-safe.
    set +u
    source /opt/ros/jazzy/setup.bash
    rosdep install --from-paths "$1" --ignore-src --rosdistro jazzy -y
fi
