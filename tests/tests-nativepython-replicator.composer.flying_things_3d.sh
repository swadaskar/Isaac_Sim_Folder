#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-replicator.composer.flying_things_3d']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/tools/composer/src/main.py --input parameters/flying_things_3d.yaml --num-scenes 5 --output flying_things_3d_out --headless --overwrite --nucleus-server localhost/NVIDIA/Assets/Isaac/2022.2.1 $@
echo "##teamcity[testFinished name='tools/composer/src/main.py']"
        