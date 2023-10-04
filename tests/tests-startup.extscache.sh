#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export RESOURCE_NAME="IsaacSim"
exec "$SCRIPT_DIR/../kit/kit" "$SCRIPT_DIR/../apps/omni.isaac.sim.kit" --ext-folder "$SCRIPT_DIR/../apps" --no-window --/app/quitAfter=500 --/app/extensions/registryEnabled=0 "$@"
