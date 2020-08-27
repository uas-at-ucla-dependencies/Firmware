#!/bin/bash

# Note: A lot of the important build/run stuff happens in Firmware/platforms/posix/cmake/sitl_target.cmake if you want to understand how it works
git submodule update --init --recursive
make px4_sitl_default px4
make px4_sitl_default sitl_gazebo
