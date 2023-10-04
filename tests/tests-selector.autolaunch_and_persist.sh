#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export RESOURCE_NAME="IsaacSim"
exec "$SCRIPT_DIR/../kit/kit" "$SCRIPT_DIR/../apps/omni.isaac.sim.selector.kit" --ext-folder "$SCRIPT_DIR/../apps" --/app/quitAfter=500 --/persistent/ext/omni.isaac.selector/auto_start=true --/persistent/ext/omni.isaac.selector/show_console=true --/persistent/ext/omni.isaac.selector/persistent_selector=true --/persistent/ext/omni.isaac.selector/extra_args='--/app/quitAfter=10' "$@"
