#!/usr/bin/env bash
set -e
# ROS setup scripts do not guarantee compatibility with bash nounset.
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${VIRTUAL_ENV}/bin/activate"

# Support a numeric host UID; keep caches/logs out of both /root and the source tree.
export ROS_HOME="/tmp/piakf-${UID}/ros"
export ROS_LOG_DIR="${ROS_HOME}/log"
export XDG_CACHE_HOME="/tmp/piakf-${UID}/cache"
export MPLCONFIGDIR="/tmp/piakf-${UID}/matplotlib"
mkdir -p "$ROS_LOG_DIR" "$XDG_CACHE_HOME" "$MPLCONFIGDIR"
if [ "$#" -eq 0 ]; then
    set -- bash
fi
exec "$@"
