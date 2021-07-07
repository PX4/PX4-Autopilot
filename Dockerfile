#
# PX4 development environment
#

FROM ubuntu:20.04
LABEL maintainer="Daniel Agar <daniel@agar.ca>"

COPY Tools/setup/ubuntu.sh /tmp/ubuntu.sh
COPY Tools/setup/requirements.txt /tmp/requirements.txt
RUN DEBIAN_FRONTEND=noninteractive /tmp/ubuntu.sh --no-sim-tools \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

ENV CCACHE_UMASK=000
ENV PATH="/usr/lib/ccache:$PATH"

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# create and start as LOCAL_USER_ID
COPY Tools/setup/entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]
