#!/bin/bash
# Launch the "Rainbow is all you need" marimo notebooks.
#
# Usage:
#   start-rainbow.sh                 # start the marimo server on port 2718
#   start-rainbow.sh 01_dqn.py       # start it directly on one chapter
#
# The notebooks shipped read-only in /opt/rainbow-is-all-you-need are copied
# once into the (bind-mounted, hence persistent) student workspace, so that
# students keep their edits between container restarts.

set -euo pipefail

SRC=/opt/rainbow-is-all-you-need
DEST="${HOME}/workspace/rainbow-is-all-you-need"
PORT="${MARIMO_PORT:-2718}"

# Fall back to the home directory when the workspace is not writable (this
# happens when the container is started without the workspace bind mount and
# without HOST_UID/HOST_GID, so the volume stays owned by root).
if ! mkdir -p "$DEST" 2>/dev/null; then
    DEST="${HOME}/rainbow-is-all-you-need"
    echo "WARNING: ${HOME}/workspace is not writable, using $DEST instead."
    echo "         Your edits will NOT survive 'docker compose down'."
    mkdir -p "$DEST"
fi

if [ ! -e "$DEST/01_dqn.py" ]; then
    echo "First run: copying the notebooks into $DEST ..."
    cp -r "$SRC"/. "$DEST"/
fi

cd "$DEST"

if pgrep -f "marimo edit" >/dev/null 2>&1; then
    echo "A marimo server is already running on port ${PORT}."
else
    echo "Starting marimo on http://localhost:${PORT} (log: ${HOME}/marimo.log)"
    nohup marimo edit \
        --host 0.0.0.0 --port "${PORT}" \
        --no-token --headless \
        > "${HOME}/marimo.log" 2>&1 &
    echo $! > "${HOME}/marimo.pid"
    # Wait for the server to answer before reporting it as ready
    for _ in $(seq 1 30); do
        curl -sf "http://localhost:${PORT}" >/dev/null 2>&1 && break
        sleep 1
    done
fi

URL="http://localhost:${PORT}"
if [ $# -ge 1 ]; then
    URL="${URL}/?file=$1"
fi

echo "Notebooks available at: ${URL}"
if [ -n "${DISPLAY:-}" ] && command -v chromium-browser >/dev/null 2>&1; then
    chromium-browser --no-sandbox --disable-gpu "${URL}" >/dev/null 2>&1 &
fi
