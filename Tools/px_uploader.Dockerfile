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

WORKDIR /firmware

ENTRYPOINT ["/bin/px_uploader.py"]

# dependency of px_uploader.py
RUN pip3 install --user pyserial

ADD px4-firmware/Tools/px_uploader.py /bin/

# copy /bin/* -> /firmware/*
ADD bin/ /firmware/

ADD px4-firmware/ssrc_config/indoor_lighthouse/config.txt /flight_modes/indoor_config.txt
ADD px4-firmware/ssrc_config/HIL/config.txt /flight_modes/hitl_config.txt
ADD px4-firmware/ssrc_config/HIL/ethernet/config.txt /flight_modes/hitl_eth_config.txt
