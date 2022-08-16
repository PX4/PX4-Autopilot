#
# PX4 base development environment
#

FROM ubuntu:20.04
LABEL maintainer="Daniel Agar <daniel@agar.ca>, Ramon Roche <mrpollo@gmail.com>"

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Installing required utilities
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
  ca-certificates \
  gnupg \
  lsb-core \
  lsb-release \
  sudo \
  software-properties-common \
  wget \
  gosu \
  ;

# Install PX4 Requirements
COPY Tools/setup/requirements.txt /tmp/requirements.txt
COPY Tools/setup/ubuntu.sh /tmp/ubuntu.sh
# The PATH env is set within ubuntu.sh, but given how we
# are running the image using `gosu` to avoid read-only problems
# with the filesystem the env variable does not persist
ENV PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"

RUN bash /tmp/ubuntu.sh --from-docker

ENV DISPLAY :99

ENV FASTRTPSGEN_DIR="/usr/local/bin/"
ENV TERM=xterm
ENV TZ=UTC

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# create and start as LOCAL_USER_ID
COPY Tools/setup/docker-entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]
