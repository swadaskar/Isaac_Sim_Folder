#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-import_sys']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh"  -c "import sys" --
echo "##teamcity[testFinished name='tests-nativepython-import_sys']"
        