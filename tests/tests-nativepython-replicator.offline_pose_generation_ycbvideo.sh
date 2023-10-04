#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-replicator.offline_pose_generation_ycbvideo']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/standalone_examples/replicator/offline_pose_generation/offline_pose_generation.py --num_mesh 3 --num_dome 3 --writer YCBVideo --output_folder _out_ycb $@
echo "##teamcity[testFinished name='standalone_examples/replicator/offline_pose_generation/offline_pose_generation.py']"
        