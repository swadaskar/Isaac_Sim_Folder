#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export RESOURCE_NAME="IsaacSim"
exec "$SCRIPT_DIR/../kit/kit" "$SCRIPT_DIR/../apps/omni.isaac.sim.headless.websocket.kit" --ext-folder "$SCRIPT_DIR/../apps" --no-window --/app/livestream/websocket/encoder_selection=OPENH264 --/app/quitAfter=500 "$@"
