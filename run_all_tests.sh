#!/bin/bash

set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})

shopt -s globstar
args=""

# Check if we are running in a docker container
if [ -f /.dockerenv ]; then
    echo "running as root"
    args="$args --allow-root"

    # Check for vulkan in docker container
    if [[ -f "${SCRIPT_DIR}/vulkan_check.sh" ]]; then
      ${SCRIPT_DIR}/vulkan_check.sh
    fi
fi

if [[ -z "${DISPLAY}" ]]; then
    echo "running headless"
    args="$args --no-window"
fi

pushd $SCRIPT_DIR/tests/
for f in tests-*.sh; do
    echo "Executing Test: $f"
    bash "$f" $args $@
done
popd