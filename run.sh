#!/bin/bash

#   _    _               _____     ____     _    _    _____   _
#  | |  | |     /\      / ____|   / __ \   | |  | |  / ____| | |          /\
#  | |  | |    /  \    | (___    / / _` |  | |  | | | |      | |         /  \
#  | |  | |   / /\ \    \___ \  | | /_|_|  | |  | | | |      | |        / /\ \
#  | |__| |  / ____ \   ____) |  \ \__,_|  | |__| | | |____  | |____   / ____ \
#   \____/  /_/    \_\ |_____/    \____/    \_____/  \_____| |______| /_/    \_\

################################################################################

# random change to test pushing

cd "$(dirname "$0")";

# Exit if any errors are encountered.
set -e

# Load arguments into variables.
ACTION=$1
FRAME_TYPE=$2 # quad, plane, or any other frame supported supported by sitl
LOCATION=$3

# sitl_run.sh arguments
DEBUGGER="none"
PROGRAM="gazebo"

MODEL=$FRAME_TYPE
if [ "$MODEL" = "" ] || [ "$MODEL" = "quad" ]
then
  MODEL="none"
fi

WORLD="none"

# Note: A lot of the important build/run stuff happens in Firmware/platforms/posix/cmake/sitl_target.cmake if you want to understand how it works
DIR="$(pwd)"
SITL_RUN_CMD="cd \"$DIR/build/px4_sitl_default/tmp\" && \"$DIR/Tools/sitl_run.sh\" \"$DIR/build/px4_sitl_default/bin/px4\" $DEBUGGER $PROGRAM $MODEL $WORLD \"$DIR\" \"$DIR/build/px4_sitl_default\""

# Determine what action to perform.
if [ "$ACTION" = "simulate_headless" ]
then
  RUN_CMD="HEADLESS=1 $SITL_RUN_CMD"
elif [ "$ACTION" = "simulate" ]
then
  RUN_CMD="$SITL_RUN_CMD"
elif [ "$ACTION" = "mavlink_router" ]
then
  mavlink-routerd 0.0.0.0:14550 # example using a specific endpoint: mavlink-routerd -e localhost:9010 0.0.0.0:14550
  exit
else
  echo "Unknown action given: $ACTION"
  exit 1
fi

# Select lat/lng/alt based on selected location
unset LATITUDE
unset LONGITUDE
unset ALTITUDE

if [ "$LOCATION" = "" ]
then
  LOCATION="auvsi_competition"
fi

if [ "$LOCATION" = "apollo_practice" ]
then
  LATITUDE=34.173103
  LONGITUDE=-118.482008
  ALTITUDE=141.122
elif [ "$LOCATION" = "auvsi_competition" ]
then
  LATITUDE=38.147483
  LONGITUDE=-76.427778
  ALTITUDE=141.122
elif [ "$LOCATION" = "ucla_sunken_gardens" ]
then
  LATITUDE=34.071680
  LONGITUDE=-118.440213
  ALTITUDE=141.122
else
  echo "Unknown location given: $LOCATION"
  exit 1
fi

export PX4_HOME_LAT=$LATITUDE
export PX4_HOME_LON=$LONGITUDE
export PX4_HOME_ALT=$ALTITUDE
bash -c "$RUN_CMD"
