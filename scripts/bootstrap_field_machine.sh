#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed at /opt/ros/jazzy."
  exit 1
fi

source /opt/ros/jazzy/setup.bash
cd "${WORKSPACE_ROOT}"

python3 -m pip install \
  --user \
  --break-system-packages \
  -r requirements-field.txt

if command -v rosdep >/dev/null 2>&1; then
  rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro jazzy \
    -y
else
  echo "rosdep is unavailable; install ROS package dependencies manually."
fi

colcon build --symlink-install

if [[ ! -f config/field.env ]]; then
  cp config/field.env.example config/field.env
  chmod 600 config/field.env
  echo "Created config/field.env. Add Spot credentials before field use."
fi

echo "Bootstrap complete."
