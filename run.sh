#!/bin/bash

#   _    _               _____     ____     _    _    _____   _
#  | |  | |     /\      / ____|   / __ \   | |  | |  / ____| | |          /\
#  | |  | |    /  \    | (___    / / _` |  | |  | | | |      | |         /  \
#  | |  | |   / /\ \    \___ \  | | /_|_|  | |  | | | |      | |        / /\ \
#  | |__| |  / ____ \   ____) |  \ \__,_|  | |__| | | |____  | |____   / ____ \
#   \____/  /_/    \_\ |_____/    \____/    \_____/  \_____| |______| /_/    \_\

################################################################################

cd "$(dirname "$0")";

# Exit if any errors are encountered.
set -e

# Load arguments into variables.
ACTION=$1
FRAME_TYPE=$2
LOCATION=$3

DOCKER_IP="192.168.3.20" # drone's docker network ip

unset GAZEBO_MODE
if [ "$FRAME_TYPE" = "" ] || [ "$FRAME_TYPE" = "quad" ]
then
  GAZEBO_MODE="none"
elif [ "$FRAME_TYPE" = "plane" ]
then
  GAZEBO_MODE="plane"
else
  echo "Unknown frame type: $FRAME_TYPE"
  exit 1
fi

DIR="$(pwd)"
SITL_RUN_CMD="cd \"$DIR/build/px4_sitl_default/tmp\" && \"$DIR/Tools/sitl_run.sh\" \"$DIR/build/px4_sitl_default/bin/px4\" none gazebo $GAZEBO_MODE \"$DIR\" \"$DIR/build/px4_sitl_default\""

# Determine what action to perform.
if [ "$ACTION" = "simulate_headless" ]
then
  RUN_CMD="HEADLESS=1 $SITL_RUN_CMD"
elif [ "$ACTION" = "simulate" ]
then
  RUN_CMD="$SITL_RUN_CMD"
elif [ "$ACTION" = "mavlink_router" ]
then
  mavlink-routerd -e localhost:9010 -e $DOCKER_IP:9011 0.0.0.0:14550
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
