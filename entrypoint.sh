#!/bin/bash
set -e
source "/opt/ros/jazzy/setup.bash"
if [ -f "/root/jackal_ar4_ws/install/setup.bash" ]; then
  source "/root/jackal_ar4_ws/install/setup.bash"
fi
exec "$@"
