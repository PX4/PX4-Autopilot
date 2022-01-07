# px4-firmware builder for PixHawk4
FROM ros:foxy as px4-cross-builder

ENV LANG C.UTF-8
ENV LANGUAGE C.UTF-8
ENV LC_ALL C.UTF-8

RUN echo "deb [trusted=yes] https://ssrc.jfrog.io/artifactory/ssrc-debian-public-remote focal fog-sw" >> /etc/apt/sources.list.d/fogsw-latest.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential wget cmake lsb-core ninja-build \
    python3-empy python3-toml python3-numpy python3-genmsg \
    python3-packaging python3-jinja2 python3-yaml openjdk-11-jre \
    fast-dds-gen \
    python3-nacl python3-pip \
    && pip3 install kconfiglib jsonschema \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /tools

WORKDIR tmp

RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2
RUN tar xvf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2 -C /tools
ENV PATH=/tools/gcc-arm-none-eabi-9-2019-q4-major/bin/:$PATH

WORKDIR /build

COPY . .

RUN . /opt/ros/foxy/setup.sh \
    && SIGNING_TOOL=Tools/cryptotools.py make px4_fmu-v5_ssrc \
    && SIGNING_TOOL=Tools/cryptotools.py make px4_fmu-v5x_ssrc

FROM scratch
COPY --from=px4-cross-builder /build/build/px4_fmu-v5_ssrc/px4_fmu-v5_ssrc.px4 /artifacts/
COPY --from=px4-cross-builder /build/build/px4_fmu-v5x_ssrc/px4_fmu-v5x_ssrc.px4 /artifacts/
COPY --from=px4-cross-builder /build/Tools/px_uploader.py /artifacts/
