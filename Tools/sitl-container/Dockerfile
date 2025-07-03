FROM ubuntu:24.04

RUN apt-get update \
  && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    libbsd0 \
    libcom-err2 \
    libc6 \
    libgcc-s1 \
    libgssapi-krb5-2 \
    libk5crypto3 \
    libkeyutils1 \
    libkrb5-3 \
    libkrb5support0 \
    libmd0 \
    libnorm1 \
    libpgm-5.3-0 \
    libprotobuf32 \
    libprotobuf-lite32t64 \
    libsodium23 \
    libstdc++6 \
    libuuid1 \
    libzmq5 \
    wget \
    zlib1g \
  && rm -fr /var/lib/apt/lists/*

RUN curl -sSL https://packages.osrfoundation.org/gazebo.key \
  | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg \
  && echo "deb  [signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
  && apt-get update \
  && apt-get install -y --no-install-recommends \
    libgz-msgs10 \
    libgz-transport13 \
    libgz-utils2 \
  && rm -fr /var/lib/apt/lists/*

COPY build/px4_sitl_default/bin /opt/px4/bin
COPY build/px4_sitl_default/etc /opt/px4/etc

WORKDIR /opt/px4

COPY Tools/sitl-container/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

EXPOSE 14540/udp 14550/udp 8888/udp

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD []
