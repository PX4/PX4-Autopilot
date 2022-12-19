#!/usr/bin/env bash

DIR="$(dirname $(readlink -f $0))"
PX4_BINARY_FILE="$1"
DEFAULT_AUTOPILOT_HOST=10.41.0.1
DEFAULT_AUTOPILOT_PORT=33333
DEFAULT_AUTOPILOT_USER=auterion

for i in "$@"
do
    case $i in
        --default-ip=*)
        DEFAULT_AUTOPILOT_HOST="${i#*=}"
        ;;
        --default-port=*)
        DEFAULT_AUTOPILOT_PORT="${i#*=}"
        ;;
        --default-user=*)
        DEFAULT_AUTOPILOT_USER="${i#*=}"
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

echo "Uploading to $AUTOPILOT_HOST..."

"$DIR"/remote_update_fmu.sh -f "$PX4_BINARY_FILE" -d "$AUTOPILOT_HOST" -p $AUTOPILOT_PORT -u $AUTOPILOT_USER

exit 0
