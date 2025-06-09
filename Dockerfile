FROM ubuntu:24.04

# environment
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# install dependencies and PX4 tools
COPY Tools/setup/requirements.txt /tmp/requirements.txt
COPY Tools/setup/ubuntu.sh /tmp/ubuntu.sh
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends lsb-release && \
    bash /tmp/ubuntu.sh --no-sim-tools && \
    rm -rf /var/lib/apt/lists/*

# copy source
COPY . /px4
WORKDIR /px4

# default command
CMD ["/bin/bash"]
