#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export RESOURCE_NAME="IsaacSim"
exec "$SCRIPT_DIR/../kit/kit"  --empty --enable omni.kit.test --/exts/omni.kit.test/testExtEnableProfiler=0 --/exts/omni.kit.test/testExtDefaultTimeout=600 --/exts/omni.kit.test/testExtArgs/0="--no-window" --/exts/omni.kit.test/testExtArgs/1="--allow-root" --/exts/omni.kit.test/testExtArgs/2="--/exts/omni.kit.test/testExtDefaultTimeout=600" --/exts/omni.kit.test/testExtApp="$SCRIPT_DIR/../apps/omni.isaac.sim.test_ext.kit" --/exts/omni.kit.test/testExts/0='omni.isaac.examples_nodes' --ext-folder "$SCRIPT_DIR/../exts"  --ext-folder "$SCRIPT_DIR/../extscache"  --ext-folder "$SCRIPT_DIR/../apps"  --/app/enableStdoutOutput=0 --no-window --allow-root "$@"
