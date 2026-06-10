#!/usr/bin/env bash

DIR="$(dirname $(readlink -f $0))"
DEFAULT_AUTOPILOT_HOST=10.41.1.1
DEFAULT_AUTOPILOT_PORT=33333
DEFAULT_AUTOPILOT_USER=auterion
EXTERNAL_FIRMWARE_FILES=()

for i in "$@"
do
    case $i in
        --file=*)
        PX4_BINARY_FILE="${i#*=}"
        ;;
        --default-ip=*)
        DEFAULT_AUTOPILOT_HOST="${i#*=}"
        ;;
        --default-port=*)
        DEFAULT_AUTOPILOT_PORT="${i#*=}"
        ;;
        --default-user=*)
        DEFAULT_AUTOPILOT_USER="${i#*=}"
        ;;
        --revert)
        REVERT_AUTOPILOT_ARGUMENT=-r
        ;;
        --wifi)
        DEFAULT_AUTOPILOT_HOST=10.41.0.1
        ;;
        --ext-fw=*)
        EXTERNAL_FIRMWARE_FILES+=("${i#*=}")
        ;;
        *)
            # unknown option
        ;;
    esac
done

# allow these to be overridden
[ -z "$AUTOPILOT_HOST" ] && AUTOPILOT_HOST=$DEFAULT_AUTOPILOT_HOST
[ -z "$AUTOPILOT_PORT" ] && AUTOPILOT_PORT=$DEFAULT_AUTOPILOT_PORT
[ -z "$AUTOPILOT_USER" ] && AUTOPILOT_USER=$DEFAULT_AUTOPILOT_USER

ARGUMENTS=()
ARGUMENTS+=(-d "$AUTOPILOT_HOST")
ARGUMENTS+=(-p "$AUTOPILOT_PORT")
ARGUMENTS+=(-u "$AUTOPILOT_USER")
ARGUMENTS+=(${PX4_BINARY_FILE:+-f "$PX4_BINARY_FILE"})
for _ext_fw in "${EXTERNAL_FIRMWARE_FILES[@]}"; do
    ARGUMENTS+=(-x "$_ext_fw")
done
ARGUMENTS+=($REVERT_AUTOPILOT_ARGUMENT)

echo "Flashing $AUTOPILOT_HOST ..."

"$DIR"/remote_update_fmu.sh "${ARGUMENTS[@]}"

exit 0
