#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-omni.isaac.kit.load_stage']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/standalone_examples/api/omni.isaac.kit/load_stage.py --usd_path /Isaac/Environments/Simple_Room/simple_room.usd --test --headless $@
echo "##teamcity[testFinished name='standalone_examples/api/omni.isaac.kit/load_stage.py']"
        