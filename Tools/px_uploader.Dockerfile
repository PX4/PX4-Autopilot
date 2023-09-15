FROM alpine:3.20

ARG saluki_fpga_directory
ARG SALUKI_FILE_INFO_JSON=saluki_file_info.json
ARG PX4_EXPORT_DIR
ENV PX4_EXPORT_DIR=$PX4_EXPORT_DIR

COPY $saluki_fpga_directory /firmware/combine
COPY $SALUKI_FILE_INFO_JSON /$SALUKI_FILE_INFO_JSON
WORKDIR /firmware

ENTRYPOINT ["/entrypoint.sh"]

# tools needed to extract binaries from px4 files
RUN apk add pigz jq

ADD px4-firmware/Tools/px_uploader.entrypoint /entrypoint.sh

# copy /bin/* -> /firmware/*
ADD bin/ /firmware/

ADD px4-firmware/ssrc_config /flight_modes
