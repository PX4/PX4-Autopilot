ARG saluki_pi_fpga_version
ARG saluki_v2_fpga_version
ARG saluki_v3_fpga_version

FROM ghcr.io/tiiuae/saluki-pi-fpga:$saluki_pi_fpga_version AS SALUKI_PI
FROM ghcr.io/tiiuae/saluki-pi-fpga:$saluki_v2_fpga_version AS SALUKI_V2
FROM ghcr.io/tiiuae/saluki-pi-fpga:$saluki_v3_fpga_version AS SALUKI_V3
FROM ghcr.io/tiiuae/saluki_bootloader_v2:master AS SALUKI_BOOTLOADER_v2

FROM python:alpine3.14

# run this with something like:
#
#   $ docker run --rm -it --network=host --device=/dev/ttyS7:/dev/px4serial px4-fw-updater \
#     --udp-addr=192.168.200.101 \
#     --udp-port=14541 \
#     --port=/dev/px4serial \
#     --baud-bootloader=2000000 \
#     px4_fmu-v5_ssrc.px4

# This gets built in environment with somewhat unorthodox paths:
# - The build context is at /
# - The repository itself is mounted in /px4-firmware/
# - Built firmware files are in /bin/
#
# ("/" above is relative to GH action runner home dir)
# (see .github/workflows/tiiuae-pixhawk.yaml)

# copy fpga files from separate saluki containers
COPY --from=SALUKI_PI /firmware/saluki_pi-fpga /firmware/fpga/saluki_pi
COPY --from=SALUKI_V2 /firmware/saluki_v2-fpga /firmware/fpga/saluki_v2
COPY --from=SALUKI_V3 /firmware/saluki_v3-fpga /firmware/fpga/saluki_v3
# copy px_uploader.py from saluki_bootloader_v2 container
COPY --from=SALUKI_BOOTLOADER_v2 /firmware/bootloader_v2/px_uploader.py /firmware/px_uploader.py

WORKDIR /firmware

ENTRYPOINT ["/entrypoint.sh"]

# dependency of px_uploader.py
RUN pip3 install --user pyserial

ADD px4-firmware/Tools/px_uploader.py /bin/
ADD px4-firmware/Tools/px_uploader.entrypoint /entrypoint.sh

# copy /bin/* -> /firmware/*
ADD bin/ /firmware/

ADD px4-firmware/ssrc_config /flight_modes
