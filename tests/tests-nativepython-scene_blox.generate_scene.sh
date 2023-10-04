#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-scene_blox.generate_scene']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/tools/scene_blox/src/scene_blox/generate_scene.py --save_path _out_scene_blox $@
echo "##teamcity[testFinished name='tools/scene_blox/src/scene_blox/generate_scene.py']"
        