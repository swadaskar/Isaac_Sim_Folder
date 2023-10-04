#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-omni.isaac.dynamic_control.franka_articulation']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/standalone_examples/api/omni.isaac.dynamic_control/franka_articulation.py  $@
echo "##teamcity[testFinished name='standalone_examples/api/omni.isaac.dynamic_control/franka_articulation.py']"
        