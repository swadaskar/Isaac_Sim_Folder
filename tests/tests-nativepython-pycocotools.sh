#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-nativepython-pycocotools']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh"  -m pip install --force pycocotools --no-cache-dir --no-dependencies --
echo "##teamcity[testFinished name='tests-nativepython-pycocotools']"
        