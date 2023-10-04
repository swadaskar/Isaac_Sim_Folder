#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-pip_list']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh"  -m pip list --
echo "##teamcity[testFinished name='tests-nativepython-pip_list']"
        