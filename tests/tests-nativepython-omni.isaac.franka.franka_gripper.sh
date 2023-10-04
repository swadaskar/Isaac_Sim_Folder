#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-omni.isaac.franka.franka_gripper']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/standalone_examples/api/omni.isaac.franka/franka_gripper.py --test $@
echo "##teamcity[testFinished name='standalone_examples/api/omni.isaac.franka/franka_gripper.py']"
        