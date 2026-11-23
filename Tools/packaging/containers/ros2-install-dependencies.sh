#!/usr/bin/env bash
set -euo pipefail

INCLUDE_SIM_TOOLS=${1:-0}

source /etc/os-release
case "${VERSION_CODENAME}:${ROS_DISTRO}" in
    jammy:humble|noble:jazzy) ;;
    *) echo "Unsupported Ubuntu/ROS pair: ${VERSION_CODENAME}/${ROS_DISTRO}" >&2; exit 1 ;;
esac

rm -f /etc/apt/apt.conf.d/docker-clean
apt-get update
apt-get install -y --no-install-recommends ca-certificates curl
curl -fsSL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb" \
    -o /ros2-apt-source.deb
apt-get install -y --no-install-recommends /ros2-apt-source.deb
rm /ros2-apt-source.deb
apt-get update
apt-get install -y --no-install-recommends \
    astyle build-essential ccache cmake git ninja-build \
    python3-empy python3-pip ros-dev-tools "ros-${ROS_DISTRO}-ros-base" \
    "ros-${ROS_DISTRO}-rmw-zenoh-cpp"

# astyle-py requires PyYAML 6, which conflicts with the distro PyYAML pinned
# below (5.4.1 on jammy). Rely on the distro astyle 3.1 installed above, which
# Tools/astyle/fix_code_style.sh accepts. /px4-setup is a read-only bind mount,
# so filter a copy of it.
px4_setup_dir=$(mktemp -d)
cp -r /px4-setup/. "${px4_setup_dir}"
sed -i '/^astyle-py/d' "${px4_setup_dir}/requirements.txt"

# Keep Ubuntu/ROS Python packages while satisfying PX4's shared requirements.
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
if [ "${INCLUDE_SIM_TOOLS}" -eq 0 ]; then
    extra_args="--no-sim-tools"
else
    extra_args=""
fi
RUNS_IN_DOCKER=true PIP_CONSTRAINT=/px4-python-constraints.txt PIP_NO_CACHE_DIR=1 \
    bash "${px4_setup_dir}/ubuntu.sh" --no-nuttx ${extra_args}
rm -rf /px4-python-constraints.txt "${px4_setup_dir}"

rosdep init
rosdep update --rosdistro "${ROS_DISTRO}"
if [ "$#" -gt 1 ]; then
    # ROS setup scripts are not nounset-safe.
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    rosdep install --from-paths "$2" --ignore-src --rosdistro "${ROS_DISTRO}" -y
fi
