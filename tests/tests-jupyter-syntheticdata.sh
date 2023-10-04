#!/bin/bash
set -e
echo "##teamcity[testStarted name='tests-jupyter-syntheticdata']"
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../jupyter_notebook.sh" test $SAMPLE_DIR/standalone_examples/testing/notebooks/test_syntheticdata_notebook.ipynb  $@
echo "##teamcity[testFinished name='standalone_examples/testing/notebooks/test_syntheticdata_notebook.ipynb']"
        