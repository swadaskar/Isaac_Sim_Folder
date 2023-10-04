#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-replicator.composer.warehouse_2']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/tools/composer/src/main.py --input parameters/warehouse.yaml --visualize-models --headless --output warehouse_2_out --overwrite --nucleus-server localhost/NVIDIA/Assets/Isaac/2022.2.1 $@
echo "##teamcity[testFinished name='tools/composer/src/main.py']"
        