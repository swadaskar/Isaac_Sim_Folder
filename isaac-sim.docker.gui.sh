#!/bin/bash

set -e

echo "Setting variables..."
command="$@"
if [[ -z "$@" ]]; then
    command="bash"
fi
omni_user="admin"
if ! [[ -z "${OMNI_USER}" ]]; then
	omni_user="${OMNI_USER}"
fi
omni_password="admin"
if ! [[ -z "${OMNI_PASS}" ]]; then
	omni_password="${OMNI_PASS}"
fi

omni_server="http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1"

if ! [[ -z "${OMNI_SERVER}" ]]; then
	omni_server="${OMNI_SERVER}"
fi

echo "Logging in to nvcr.io..."
docker login nvcr.io

echo "Pulling docker image..."
docker pull nvcr.io/nvidia/isaac-sim:2022.2.1

echo "Running Isaac Sim container with X11 forwarding..."
xhost +
docker run --name isaac-sim --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
	-v $HOME/.Xauthority:/root/.Xauthority \
	-e DISPLAY \
	-e "OMNI_USER=${omni_user}" -e "OMNI_PASS=${omni_password}" \
	-e "OMNI_SERVER=${omni_server}" \
    -v ~/docker/isaac-sim/kit/cache/Kit:/isaac-sim/kit/cache/Kit:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
	nvcr.io/nvidia/isaac-sim:2022.2.1 \
	-c "${command}"

echo "Isaac Sim container run completed!"
