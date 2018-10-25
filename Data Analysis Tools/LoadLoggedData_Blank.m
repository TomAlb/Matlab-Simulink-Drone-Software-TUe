clear all; clc
%% Load data
Folder  = 'Measurement_Data\2018_10_03_Heigth_Tuning';
file    = 'F';

sensor_file     = [Folder '\RawSensorData_' file '.bin'];   % Sensor Test
[datapts, ~]    = readdata(sensor_file);                    % Sensor tests

%% Data Busses
% Set bus sizes
rc = 10; target = 14; raw_sensor = 18; filtered_sensor = 18; estimated_state = 18; error = 30; controller = 4; m = 7; system_mode = 11; clock = 5;  exp = 9;

Bs              = [rc target raw_sensor filtered_sensor estimated_state error controller m system_mode clock exp];     % Bus sizes
Bl              = cumsum(Bs);
l               = 1;

%% Rc
RC              = datapts(1:Bl(l),:);

%% Target
TARGET          = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% Raw Sensor Data
RAW_SENSOR_DATA = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% Filtered Sensor Data
FILTERED_SENSOR_DATA = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% State
STATE           = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% Error
ERROR          = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% Controller
CONTROLLER      = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% Motor
M               = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% System Mode
MODE            = datapts(Bl(l)+1:Bl(l+1),:);
l = l+1;

%% Clock
CLOCK           = datapts(Bl(l)+1:Bl(l+1),:);
CLOCK = CLOCK - CLOCK(:,1);
CLOCK(1,:) = CLOCK(1,:)*10^-6;
CLOCK(2,:) = CLOCK(2,:)*10^-6;
CLOCK(4,:) = CLOCK(4,:)*10^-6;
l = l+1;

%% Experiment data
EXPERIMENTS     = datapts(Bl(l)+1:Bl(l+1),:); % - datapts(Bl(l)+1:Bl(l+1),1);
l = l+1;

%% Data saving
filename = ['AHRS_EXAMPLE.mat'];
save(filename, 'RC',...
               'TARGET',...
               'RAW_SENSOR_DATA',...
               'FILTERED_SENSOR_DATA',... 
               'STATE',... 
               'ERROR',... 
               'CONTROLLER',...
               'M',... 
               'MODE',... 
               'CLOCK',...
               'EXPERIMENTS');

%% RC contains
% 1 RC_roll command
% 2 RC_pitch command
% 3 RC_yaw command
% 4 RC_Thrust command
% 5 RC_Arm command
% 6 RC_Config_1 command
% 7 RC_Config_2 command
% 8 RC_RSSI command
% 9 RC_Failsafe command

%% TARGET contains
% 01 Throttle
% 02 Roll
% 03 Pith
% 04 Yaw
% 05 X
% 06 Y
% 07 Z
% 08 Throttle Rate
% 09 Roll Rate
% 10 Pith Rate
% 11 Yaw Rate
% 12 X Rate
% 13 Y Rate
% 14 Z Rate

%% RAW_SENSOR_DATA Contains raw sensor data
% 01 Magnetometer x
% 02 Magnetometer y
% 03 Magnetometer z
% 04 Accelerometer x
% 05 Accelerometer y
% 06 Accelerometer z
% 07 Gyroscope x
% 08 Gyroscope y
% 09 Gyroscope z
% 10 Latitude
% 11 Longitude
% 12 Altitude
% 13 Number satelites
% 14 Voltage
% 15 Current
% 16 mili Amp Hour
% 17 Sonar (missing)

%% FILTERED_SENSOR_DATA Contains filtered sensor data
% 01 Magnetometer x
% 02 Magnetometer y
% 03 Magnetometer z
% 04 Accelerometer x
% 05 Accelerometer y
% 06 Accelerometer z
% 07 Gyroscope x
% 08 Gyroscope y
% 09 Gyroscope z
% 10 Latitude
% 11 Longitude
% 12 Altitude
% 13 Number satelites
% 14 Voltage
% 15 Current
% 16 mili Amp Hour
% 17 Sonar (missing)

%% STATE contains
% 01 x
% 02 y
% 03 z
% 04 phi
% 05 theta
% 06 psi
% 07 Vx
% 08 Vy
% 09 Vz
% 10 Wphi
% 11 Wtheta
% 12 Wpsi
% 13 ax
% 14 ay
% 15 az
% 16 alpha phi
% 17 alpha theta
% 18 alpha psi

%% Error Contains
% 01 P  roll
% 02 I  roll
% 03 D  roll
% 04 FF roll
% 05 U  roll

% 06 P  pitch
% 07 I  pitch
% 08 D  pitch
% 09 FF pitch
% 10 U  pitch

% 11 P  yaw
% 12 I  yaw
% 13 D  yaw
% 14 FF yaw
% 15 U  yaw

% 16 P  x
% 17 I  x
% 18 D  x
% 19 FF x
% 20 U  x

% 21 P  y
% 22 I  y
% 23 D  y
% 24 FF y
% 25 U  y

% 26 P  z
% 27 I  z
% 28 D  z
% 29 FF z
% 30 U  z

%% CONTROLLER Contains
% 1 U Roll
% 2 U Pitch
% 3 U Yaw
% 4 U Thrust

%% M Contains
% 1 PWM Motor 1
% 2 PWM Motor 2
% 3 PWM Motor 3
% 4 PWM Motor 4
% 5 Solver type 1
% 6 Solver type 2
% 7 Motor mode

%% SYSTEM MODE Contains
% 01 arm command
% 02 pre-arm command
% 03 config command
% 04 Manual flight
% 05 semi-auto flight
% 06 auto flight
% 07 recoding
% 08 config motor 1
% 09 config motor 2
% 10 config motor 3
% 11 config motor 4



