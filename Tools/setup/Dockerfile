# PX4 base development environment
FROM ubuntu:24.04
LABEL maintainer="Daniel Agar <daniel@agar.ca>, Ramon Roche <mrpollo@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DISPLAY=:99
ENV TERM=xterm
ENV TZ=UTC
ENV RUNS_IN_DOCKER=true

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# Install Shell Script Entrypoint
COPY docker-entrypoint.sh /usr/local/bin/docker-entrypoint.sh

# Install PX4 Requirements
COPY requirements.txt /tmp/requirements.txt
COPY ubuntu.sh /tmp/ubuntu.sh

RUN bash /tmp/ubuntu.sh --no-sim-tools

# Install the optional tooling requirements as well, so CI jobs that need them
# (itcm_check) don't have to reach out to PyPI on every run.
COPY optional-requirements.txt /tmp/optional-requirements.txt

RUN python3 -m pip install --break-system-packages -r /tmp/optional-requirements.txt

# Make sure git is ok with your local copy
RUN git config --global --add safe.directory '*'

# Create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

ENV PATH="${PATH}:/opt/xtensa-esp-elf/bin"

ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]

CMD ["/bin/bash"]
