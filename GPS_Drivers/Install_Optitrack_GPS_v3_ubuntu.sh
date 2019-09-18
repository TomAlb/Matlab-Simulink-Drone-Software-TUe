#!/bin/bash

# USAGE:
# chmod a+x Install_Optitrack_GPS.sh
# ./Install_Optitrack_GPS.sh {PATH_TO_PX$_FIRMARE}
# ./Install_Optitrack_GPS.sh /home/tom/Documents/MATLAB/PX4_Project/px4/Firmware


# SET VARS
Fetch_Repo="https://github.com/mathworks/PX4-Firmware.git (fetch)"
Push_Repo="https://github.com/mathworks/PX4-Firmware.git (push)"

Fetch_Repo="https://github.com/PX4/Firmware.git (fetch)"
Push_Repo="https://github.com/PX4/Firmware.git (push)"

echo "##############################################"
echo "##############################################"
echo "##                                          ##"
echo "##     INSTALLING OPTITRACK GPS DRIVERS     ##"
echo "##                                          ##"
echo "##############################################"
echo "##############################################"

set -o errexit # stop execution on error
PX4_Path=$1  # retrieve input parameter and populate a variable
Home_Path=$(pwd)  # retrieve input parameter and populate a variable

# Check if obtained directory exists
if [ ! -d "$PX4_Path" ]; then
	echo "Directory:"
	echo $PX4_Path
	echo "does not exists!"
	exit 0
fi

# Go to directoy
cd $PX4_Path

# Test if proper Git repository can be found
mapfile -t <<< "$(git \remote \-v)"

if [[ "${MAPFILE[0]}" != *"${Fetch_Repo}"* || "${MAPFILE[1]}" != *"${Push_Repo}"* ]]; then
	echo "Git Repository is not matching"
	exit 0
fi


#echo "$Home_Path/drv_gps.h"
echo "Copy files from: "
echo "$Home_Path/test.txt"
echo " to: "
echo "$PX4_Path/src/drivers" 

# Copy drv_gps.h
cp "$Home_Path/gps_optitrack_v3/drv_gps.h" "$PX4_Path/src/drivers"

# Copy /gps_optitrack/CMakeLists.txt
cp "$Home_Path/gps_optitrack_v3/CMakeLists.txt" "$PX4_Path/src/drivers/gps"

# Copy /gps_optitrack/gps.cpp
cp "$Home_Path/gps_optitrack_v3/gps.cpp" "$PX4_Path/src/drivers/gps"

# Copy /gps_optitrack/devices/src
cp -r "$Home_Path/gps_optitrack_v3/devices/src" "$PX4_Path/src/drivers/gps/devices"

# Copy /gps_optitrack/devices/src
cp "$Home_Path/gps_optitrack_v3/msg/vehicle_gps_optitrack.msg" "$PX4_Path/msg"

# Copy /gps_optitrack/devices/src
cp "$Home_Path/gps_optitrack_v3/msg/CMakeLists.txt" "$PX4_Path/msg"









