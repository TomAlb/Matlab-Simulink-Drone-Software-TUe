ECHO OFF
ECHO ##############################################
ECHO ##############################################
ECHO ##                                          ##
ECHO ##     INSTALLING OPTITRACK GPS DRIVERS     ##
ECHO ##                                          ##
ECHO ##############################################
ECHO ##############################################

SET /p path="Enter Firmware Path: "
REM ECHO %path%\src\drivers\gps\devices\src

ECHO Copying optitrack driver files
ECHO optitrack.cpp
copy "%CD%\gps_optitrack_v2\devices\src\optitrack.cpp" "%path%\src\drivers\gps\devices\src"

ECHO optitrack.h
copy "%CD%\gps_optitrack_v2\devices\src\optitrack.h" "%path%\src\drivers\gps\devices\src"

ECHO marv.cpp
copy "%CD%\gps_optitrack_v2\devices\src\marv.cpp" "%path%\src\drivers\gps\devices\src"

ECHO marv.h
copy "%CD%\gps_optitrack_v2\devices\src\marv.h" "%path%\src\drivers\gps\devices\src"

ECHO gps_helper.h
copy "%CD%\gps_optitrack_v2\devices\src\gps_helper.h" "%path%\src\drivers\gps\devices\src"

ECHO gps_helper.cpp
copy "%CD%\gps_optitrack_v2\devices\src\gps_helper.cpp" "%path%\src\drivers\gps\devices\src"

ECHO CMakeLists.txt
copy "%CD%\gps_optitrack_v2\CMakeLists.txt" "%path%\src\drivers\gps"

ECHO definitions.h
copy "%CD%\gps_optitrack_v2\definitions.h" "%path%\src\drivers\gps"

ECHO gps.cpp
copy "%CD%\gps_optitrack_v2\gps.cpp" "%path%\src\drivers\gps"

ECHO module.yaml
copy "%CD%\gps_optitrack_v2\module.yaml" "%path%\src\drivers\gps"

ECHO params.c
copy "%CD%\gps_optitrack_v2\params.c" "%path%\src\drivers\gps"


PAUSE










