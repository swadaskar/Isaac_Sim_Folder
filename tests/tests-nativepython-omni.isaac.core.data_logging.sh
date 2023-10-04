#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-omni.isaac.core.data_logging']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/standalone_examples/api/omni.isaac.core/data_logging.py  $@
echo "##teamcity[testFinished name='standalone_examples/api/omni.isaac.core/data_logging.py']"
        