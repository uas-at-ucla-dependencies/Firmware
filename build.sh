#!/bin/bash

# Exit if any errors are encountered.
set -e

git submodule update --init --recursive

# Activate python virtualenv if it exists
if test -f "px4_venv/bin/activate"; then
  source "px4_venv/bin/activate"
fi

# Note: A lot of the important build/run stuff happens in Firmware/platforms/posix/cmake/sitl_target.cmake if you want to understand how it works
make px4_sitl_default px4
make px4_sitl_default sitl_gazebo
