FROM python:alpine3.14

# run this with something like:
#
#   $ docker run --rm -it --device=/dev/ttyS7:/dev/px4serial px4-fw-updater \
# 		--port=/dev/px4serial \
#		--baud-bootloader=115200 \
#		--baud-flightstack=1000000 \
#		px4_fmu-v5_ssrc.px4

# This gets built in environment with somewhat unorthodox paths:
# - This file is copied to /Dockerfile
# - The repository itself is mounted in /px4-firmware/
# - Built firmware files are in /bin/
#
# (/ is actually relative to GH action runner home dir)
# (see .github/workflows/tiiuae-pixhawk.yaml)

WORKDIR /firmware

ENTRYPOINT ["/bin/px_uploader.py"]

# dependency of px_uploader.py
RUN pip3 install --user pyserial

ADD px4-firmware/Tools/px_uploader.py /bin/

# copy /bin/* -> /firmware/*
ADD bin/ /firmware/
