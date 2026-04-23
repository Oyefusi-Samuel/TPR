#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source "${WORKSPACE}/install/setup.bash"
fi

exec "$@"
