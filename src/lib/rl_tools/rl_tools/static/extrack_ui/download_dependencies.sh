set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"

if [ -d "$SCRIPT_DIR/lib/.git" ]; then
    echo "Dependencies already checked out, pulling latest changes..."
    cd $SCRIPT_DIR/lib && git pull
else
    echo "Checking out dependencies..."
    git clone https://github.com/rl-tools/extrack-ui-lib.git $SCRIPT_DIR/lib
fi
