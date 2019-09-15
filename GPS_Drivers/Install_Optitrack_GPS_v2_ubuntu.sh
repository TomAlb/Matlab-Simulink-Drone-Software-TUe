#!/bin/bash

# USAGE:
# chmod a+x Install_Optitrack_GPS_v2_ubuntu.sh
# ./Install_Optitrack_GPS.sh {PATH_TO_PX$_FIRMARE}
# ./Install_Optitrack_GPS.sh /home/tom/Documents/MATLAB/PX4_Project/px4/MyPX4/Firmware



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
echo ${MAPFILE[0]}
echo ${MAPFILE[1]}

if [[ "${MAPFILE[0]}" != *"${Fetch_Repo}"* || "${MAPFILE[1]}" != *"${Push_Repo}"* ]]; then
        echo "Git Repository is not matching"
        exit 0
fi

#echo "$Home_Path/drv_gps.h"
echo "Copy files from: "
echo "$Home_Path"
echo " to: "
echo "$PX4_Path/src/drivers"

echo "Copying optitrack driver files"
echo "optitrack.cpp"
cp "$Home_Path/gps_optitrack_v2/devices/src/optitrack.cpp" "$PX4_Path/src/drivers/gps"

echo optitrack.h
cp "$Home_Path/gps_optitrack_v2/devices/src/optitrack.h" "$PX4_Path/src/drivers/gps/devices/src"

echo marv.cpp
cp "$Home_Path/gps_optitrack_v2/devices/src/marv.cpp" "$PX4_Path/src/drivers/gps/devices/src"

echo marv.h
cp "$Home_Path/gps_optitrack_v2/devices/src/marv.h" "$PX4_Path/src/drivers/gps/devices/src"

echo gps_helper.h
cp "$Home_Path/gps_optitrack_v2/devices/src/gps_helper.h" "$PX4_Path/src/drivers/gps/devices/src"

echo gps_helper.cpp
cp "$Home_Path/gps_optitrack_v2/devices/src/gps_helper.cpp" "$PX4_Path/src/drivers/gps/devices/src"

echo CMakeLists.txt
cp "$Home_Path/gps_optitrack_v2/CMakeLists.txt" "$PX4_Path/src/drivers/gps"

echo definitions.h
cp "$Home_Path/gps_optitrack_v2/definitions.h" "$PX4_Path/src/drivers/gps"

echo gps.cpp
cp "$Home_Path/gps_optitrack_v2/gps.cpp" "$PX4_Path/src/drivers/gps"

echo module.yaml
cp "$Home_Path/gps_optitrack_v2/module.yaml" "$PX4_Path/src/drivers/gps"

echo params.c
cp "$Home_Path/gps_optitrack_v2/params.c" "$PX4_Path/src/drivers/gps"


