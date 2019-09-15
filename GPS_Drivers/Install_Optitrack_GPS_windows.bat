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

ECHO drv_gps.h
copy "%CD%\drv_gps.h" "%path%\src\drivers"

ECHO CMakeLists.txt
copy "%CD%\gps_optitrack\CMakeLists.txt" "%path%\src\drivers\gps"

ECHO gps.copyp
copy "%CD%\gps_optitrack\gps.cpp" "%path%\src\drivers\gps"

ECHO ashtech.cpp
ECHO ashtech.h
ECHO gps_helper.cpp
ECHO gps_helper.h
ECHO marv.cpp
ECHO marv.h
ECHO mtk.cpp
ECHO mtk.h
ECHO optitrack.cpp
ECHO optitrack.h
ECHO ubx.cpp
ECHO ubx.h
copy "%CD%\gps_optitrack\devices\src\*.*" "%path%\src\drivers\gps\devices"

ECHO vehicle_gps_optitrack.msg
copy "%CD%\gps_optitrack\msg\vehicle_gps_optitrack.msg" "%path%\msg"

ECHO CMakeLists.txt
copy "%CD%\gps_optitrack\msg\CMakeLists.txt" "%path%\msg"

PAUSE
