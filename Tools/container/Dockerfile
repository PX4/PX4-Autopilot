# PX4 base development environment
#

FROM ubuntu:24.04
LABEL maintainer="Daniel Agar <daniel@agar.ca>, Ramon Roche <mrpollo@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Installing required utilities
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
  ca-certificates \
  gnupg \
  lsb-release \
  sudo \
  software-properties-common \
  wget \
  gosu \
  ;

# Install PX4 Requirements
COPY Tools/setup/requirements.txt /tmp/requirements.txt
COPY Tools/setup/ubuntu.sh /tmp/ubuntu.sh
# We need to hardcode the path to the gcc binary
ENV PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"

RUN bash /tmp/ubuntu.sh --from-docker

ENV DISPLAY=:99
ENV TERM=xterm
ENV TZ=UTC

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

COPY Tools/container/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]
