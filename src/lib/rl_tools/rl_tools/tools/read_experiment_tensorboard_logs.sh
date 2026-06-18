
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <experiment_dir> <tag_name>"
    exit 1
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
if [ "$#" = 1 ]; then
  find $1 -type f | grep logs.tf | sort | head -n 1 | xargs -I{} $SCRIPT_DIR/read_tensorboard_logs.py {}
else
  OUTPUT=$(echo "$2" | tr '/' '_')
  echo "Working on:"
  find $1 -type f | grep logs.tf | sort
  find $1 -type f | grep logs.tf | sort | xargs -I{} dirname {} | xargs -P$(nproc) -I{} bash -c "$SCRIPT_DIR/read_tensorboard_logs.py {}/logs.tfevents $2 > {}/$OUTPUT.csv"
fi


