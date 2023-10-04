#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})

# Add symlink to Isaac Examples
pushd ${SCRIPT_DIR}
if [ ! -L extension_examples ] && [ ! -e extension_examples ]; then
    ln -s exts/omni.isaac.examples/omni/isaac/examples extension_examples
    echo "Symlink extension_examples created"
else
    echo "Symlink or folder extension_examples exists"
fi
popd

# Install default Python packages
echo "Installing Python packages..."
${SCRIPT_DIR}/python.sh -m pip install -r ${SCRIPT_DIR}/requirements.txt
echo "Installing Icon"
${SCRIPT_DIR}/python.sh ${SCRIPT_DIR}/icon/install_icon.py
# # Warm up cache
# # Run command below to skip warm up
# #  echo "export ISAACSIM_SKIP_WARMUP=Y" >> ~/.profile
# if [[ -z "${ISAACSIM_SKIP_WARMUP}" ]]; then
#     set +e # Workaround post-install script failure
#     echo "Warming up cache for python app..."
#     ${SCRIPT_DIR}/python.sh ${SCRIPT_DIR}/standalone_examples/api/omni.isaac.kit/hello_world.py

#     echo "Warming up cache for main app..."
#     ${SCRIPT_DIR}/omni.isaac.sim.warmup.sh
#     set -e
# fi

echo "Isaac Sim post installation script completed!"
