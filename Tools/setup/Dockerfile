# PX4 base development environment
FROM ubuntu:24.04
LABEL maintainer="Daniel Agar <daniel@agar.ca>, Ramon Roche <mrpollo@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DISPLAY=:99
ENV TERM=xterm
ENV TZ=UTC

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

# Install PX4 Requirements
COPY requirements.txt /tmp/requirements.txt
COPY ubuntu.sh /tmp/ubuntu.sh
RUN touch /.dockerenv
RUN bash /tmp/ubuntu.sh

RUN git config --global --add safe.directory '*'

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]

CMD ["/bin/bash"]
