close all; clear all; clc
%% Init function
% Called during update phase before block parameters are evaluated. This is called during model update and simulation.

% Note
%     An InitFcn callback in a top model cannot change parameters used by referenced models.
%     During model compilation, Simulink evaluates variant objects before calling the model InitFcn callback. Do not modify the condition of the variant object in the InitFcn callback. For more information, see Define, Configure, and Activate Variants.

J = inv([0 1 0; 0 0 -1; -1 0 0])*[15380951.52 -1932.80 -19060.64;...
                                  -1932.80 29218288.98 -514860.93;...
                                  -19060.64 -514860.93 14908873.76]*[0 1 0; 0 0 -1; -1 0 0]*10^-9;

%% System specific settings
Ts = double(1/100);
% Ts = single(1/400), 'Ts'} );                                                      % Sample time
MOV_AVG_WINDOW = single(100);                                   % Moving average sample window size
% FILTER_N       = single(10),'FILTER_N'} );                                        % Cut off frequency Derivative
Jxx = single(J(1,1));                                                      % Inertia around xx
Jyy = single(J(2,2));                                                      % Inertia around yy
Jzz = single(J(3,3));                                                      % Inertia around zz
clear J;

%% Sensor Calibration
% Sensor Calibration Magnetometer
CAL_MAG0_XXSCALE = single(  1.01563   );                      % Set Magnetometer scaling in xx-direction
CAL_MAG0_XYSCALE = single( -0.0612061 );                      % Set Magnetometer scaling in xy-direction
CAL_MAG0_XZSCALE = single( -0.011532  );                      % Set Magnetometer scaling in xz-direction
CAL_MAG0_YXSCALE = single( -0.0612061 );                      % Set Magnetometer scaling in yx-direction
CAL_MAG0_YYSCALE = single(  1.00593   );                      % Set Magnetometer scaling in yy-direction
CAL_MAG0_YZSCALE = single(  0.0144582 );                      % Set Magnetometer scaling in yz-direction
CAL_MAG0_ZXSCALE = single( -0.011532  );                      % Set Magnetometer scaling in zx-direction
CAL_MAG0_ZYSCALE = single(  0.0144582 );                      % Set Magnetometer scaling in zy-direction
CAL_MAG0_ZZSCALE = single(  0.986813  );                      % Set Magnetometer scaling in zz-direction

CAL_MAG0_XOFFSET = single( 0.0446372  );                      % Set Magnetometer Hard Iron Corrections
CAL_MAG0_YOFFSET = single( 0.0700776  );                      % Set Magnetometer Hard Iron Corrections
CAL_MAG0_ZOFFSET = single( 0.00201742 );                      % Set Magnetometer Hard Iron Corrections

CAL_MAG0_XCURCOR = single( 0.0017     );                      % Set Magnetometer Current Corrections
CAL_MAG0_YCURCOR = single(-0.0075     );                      % Set Magnetometer Current Corrections
CAL_MAG0_ZCURCOR = single(-0.0080     );                      % Set Magnetometer Current Corrections

% Sensor Calibration Accelerometer
CAL_ACC0_XXSCALE = single(  1.01218    );                     % Set Accelerometer scaling in xx-direction
CAL_ACC0_XYSCALE = single( -0.00443301 );                     % Set Accelerometer scaling in xy-direction
CAL_ACC0_XZSCALE = single( -0.00505793 );                     % Set Accelerometer scaling in xz-direction
CAL_ACC0_YXSCALE = single( -0.00443301 );                     % Set Accelerometer scaling in yx-direction
CAL_ACC0_YYSCALE = single(  1.00234    );                     % Set Accelerometer scaling in yy-direction
CAL_ACC0_YZSCALE = single(  0.0160883  );                     % Set Accelerometer scaling in yz-direction
CAL_ACC0_ZXSCALE = single( -0.00505793 );                     % Set Accelerometer scaling in zx-direction
CAL_ACC0_ZYSCALE = single(  0.0160883  );                     % Set Accelerometer scaling in zy-direction
CAL_ACC0_ZZSCALE = single(  0.983604   );                     % Set Accelerometer scaling in zz-direction

CAL_ACC0_XOFFSET = single( 0.122052  );                       % Set Accelerometer offset in x-direction
CAL_ACC0_YOFFSET = single( 0.00281205);                       % Set Accelerometer offset in y-direction
CAL_ACC0_ZOFFSET = single(-0.0298121 );                       % Set Accelerometer offset in z-direction

% Sensor Calibration Gyroscope
CAL_GYRO0_XXSCALE = single(0.992736);                        % Set Gyroscope scaling in xx-direction
CAL_GYRO0_XYSCALE = single(0.0000);                          % Set Gyroscope scaling in xy-direction   
CAL_GYRO0_XZSCALE = single(0.0000);                          % Set Gyroscope scaling in xz-direction
CAL_GYRO0_YXSCALE = single(0.0000);                          % Set Gyroscope scaling in yx-direction
CAL_GYRO0_YYSCALE = single(1.01609);                         % Set Gyroscope scaling in yy-direction   
CAL_GYRO0_YZSCALE = single(0.0000);                          % Set Gyroscope scaling in yz-direction
CAL_GYRO0_ZXSCALE = single(0.0000);                          % Set Gyroscope scaling in zx-direction
CAL_GYRO0_ZYSCALE = single(0.0000);                          % Set Gyroscope scaling in zy-direction   
CAL_GYRO0_ZZSCALE = single(1.03003);                         % Set Gyroscope scaling in zz-direction

% Sensor Calibration Barometer
CAL_BAR_SCALE  = single(   1.0632);                              % Set Barometer Scaling
CAL_BAR_OFFSET = single( -20.8468);                             % Set Barometer Offset

% Sensor Calibration GPS
CAL_GPS0_XXSCALE = single(  0.0100);                          % Set Magnetometer scaling in xx-direction
CAL_GPS0_XYSCALE = single(  0.0000);                          % Set Magnetometer scaling in xy-direction
CAL_GPS0_XZSCALE = single(  0.0000);                          % Set Magnetometer scaling in xz-direction
CAL_GPS0_YXSCALE = single(  0.0000);                          % Set Magnetometer scaling in yx-direction
CAL_GPS0_YYSCALE = single(  0.0100);                          % Set Magnetometer scaling in yy-direction
CAL_GPS0_YZSCALE = single(  0.0000);                          % Set Magnetometer scaling in yz-direction
CAL_GPS0_ZXSCALE = single(  0.0000);                          % Set Magnetometer scaling in zx-direction
CAL_GPS0_ZYSCALE = single(  0.0000);                          % Set Magnetometer scaling in zy-direction
CAL_GPS0_ZZSCALE = single( -0.0010);                          % Set Magnetometer scaling in zz-direction

CAL_GPS0_XOFFSET = single( 0.0000);                           % Set Magnetometer Hard Iron Corrections
CAL_GPS0_YOFFSET = single( 0.0000);                           % Set Magnetometer Hard Iron Corrections
CAL_GPS0_ZOFFSET = single( 0.0000);                           % Set Magnetometer Hard Iron Corrections

% Sensor Calibration Battery
CAL_BAT0_XXSCALE = single(  1.0000);                          % Set Magnetometer scaling in xx-direction
CAL_BAT0_XYSCALE = single(  0.0000);                          % Set Magnetometer scaling in xy-direction
CAL_BAT0_XZSCALE = single(  0.0000);                          % Set Magnetometer scaling in xz-direction
CAL_BAT0_YXSCALE = single(  0.0000);                          % Set Magnetometer scaling in yx-direction
CAL_BAT0_YYSCALE = single(  1.0000);                          % Set Magnetometer scaling in yy-direction
CAL_BAT0_YZSCALE = single(  0.0000);                          % Set Magnetometer scaling in yz-direction
CAL_BAT0_ZXSCALE = single(  0.0000);                          % Set Magnetometer scaling in zx-direction
CAL_BAT0_ZYSCALE = single(  0.0000);                          % Set Magnetometer scaling in zy-direction
CAL_BAT0_ZZSCALE = single(  1.0000);                          % Set Magnetometer scaling in zz-direction

CAL_BAT0_XOFFSET = single( 0.0000);                           % Set Magnetometer Hard Iron Corrections
CAL_BAT0_YOFFSET = single( 0.0000);                           % Set Magnetometer Hard Iron Corrections
CAL_BAT0_ZOFFSET = single( 0.0000);                           % Set Magnetometer Hard Iron Corrections

% Sensor Calibration Sonar
CAL_SONAR_SCALE  = single(-438.1875);                           % Set Sonar Scaling
CAL_SONAR_OFFSET = single( -6.8730);                          % Set Sonar Offset

%% Sensor Fusion
% Low pass filter on 10Hz
LPF_REF_NUM = single([0.0000  0.09516]);                           % Numerator Low Pass Filter Reference
LPF_REF_DEN = single([1.0000 -0.9048]);                            % Denominator Low Pass Filter Reference

% Low pass filter on 20Hz
LPF_REF_NUM = single([0.0000  0.1813]);                           % Numerator Low Pass Filter Reference
LPF_REF_DEN = single([1.0000 -0.8187]);                            % Denominator Low Pass Filter Reference

%% Kalman filter settings Translational states
pn = [0.1; 0.1; 0.1];                                                                                       % Process noise Magnitude
mn = [0.01; 0.01; 0.01; 0.01];                                                                              % Measurement noise Magnitude
mn_1 = [0.01; 0.01; 0.01];                                                                              % Measurement noise Magnitude

A = single([diag([1 1 1]) Ts*diag([1 1 1]); diag([0 0 0]) diag([1 1 1])]);   % State transition matrix
B = single([Ts^2/2*diag([1 1 1]); Ts*diag([1 1 1])]);                        % Control input matrix
C = single([diag([1 1 1]) 0*diag([1 1 1]); zeros(1,2) 1 zeros(1,3)]);        % Measurement matrix
C_1 = single([diag([1 1 1]) 0*diag([1 1 1])]);        % Measurement matrix
D = single(zeros(size(C,1), size(B,2)));                         % Feedthrough matrix

Q = single(diag([pn.^2; pn.^2])*B*B');                           % Convert the process noise (stdv) into covariance matrix
R = single(diag([mn.^2]));                                                   % Convert the measurement noise (stdv) into covariance matrix
R_1 = single(diag([mn_1.^2]));                                                   % Convert the measurement noise (stdv) into covariance matrix
I = single(eye(size(Q)));                                              % Convert the measurement noise (stdv) into covariance matrix

%% Controller Parameters
C1  = single( 1.08680);                                                      % P-action Roll                             
C4  = single( 0.0500);                                                       % I-action Roll
C7  = single( 0.35937);                                                      % D-action Roll
C10 = single( 0.0000);                                                      % FF-action Roll

C2  = single( 1.10468);                                                      % P-action Pitch
C5  = single( 0.0500);                                                       % I-action Pitch
C8  = single( 0.36528);                                                      % D-action Pitch
C11 = single(-0.2300);                                                      % FF-action Pitch

C3  = single( 1.1980);                                                       % P-action Yaw
C6  = single( 0.0500);                                                       % I-action Yaw
C9  = single( 0.53311);                                                       % D-action Yaw
C12 = single( 0.0000);                                                      % FF-action Yaw

C13 = single( 0.8000);                                                      % P-action X 
C16 = single( 0.1000);                                                      % I-action X
C19 = single( 0.6000);                                                      % D-action X
C22 = single( 0.0000);                                                      % FF-action X

C14 = single( 0.8000);                                                      % P-action Y
C17 = single( 0.1000);                                                      % I-action Y
C20 = single( 0.6000);                                                      % D-action Y
C23 = single( 0.0000);                                                      % FF-action Y

C15 = single( 1.73200);                                                      % P-action Z
C18 = single( 0.0500);                                                      % I-action Z
C21 = single( 1.04558);                                                      % D-action Z
C24 = single(-11.0000);                                                     % FF-action Z

%% RC controller Calibration                                                                                normalizes input range (Roll, Pitch, Yaw -> [-1, 1] Thrust -> [0, 1])
% Roll
RC1_MIN     = single(984);                                            % Minimal value RC channel 1 (Roll)
RC1_MAX     = single(2006);                                           % Maximal value RC channel 1 (Roll)
RC1_TRIM    = single(1496);                                          % Trim value RC channel 1 (Roll) => 0.5 * (RC1_MIN + RC1_MAX)
RC1_DB      = single(5);                                               % RC Channel 1 DeadBand

% Yaw
RC2_MIN     = single(982);                                            % Minimal value RC channel 2 (Yaw)
RC2_MAX     = single(2006);                                           % Maximal value RC channel 2 (Yaw)
RC2_TRIM    = single(1495);                                          % Trim value RC channel 2 (Yaw) => 0.5 * (RC2_MIN + RC2_MAX)
RC2_DB      = single(5);                                               % RC Channel 1 DeadBand

% Pitch
RC3_MIN     = single(982);                                            % Minimal value RC channel 3 (Pitch)
RC3_MAX     = single(2006);                                           % Maximal value RC channel 3 (Pitch)
RC3_TRIM    = single(1522);                                          % Trim value RC channel 3 (Pitch) => 0.5 * (RC3_MIN + RC3_MAX)
RC3_DB      = single(5);                                               % RC Channel 3 DeadBand

% Throttle
RC4_MIN     = single(1000);                                           % Minimal value RC channel 4 (Thrust)
RC4_MAX     = single(2000);                                           % Maximal value RC channel 4 (Thrust)
RC4_TRIM    = single(1500);                                          % Trim value RC channel 4 (Thrust) => 0.5 * (RC4_MIN + RC4_MAX)
RC4_DB      = single(5);                                               % RC Channel 4 DeadBand

%% MIN MAX YAW/PITCH/ROLL                                                                                   Converts normalized inputs to desired units (SI)
PITCH_MAX   = single(pi/4);                                           % Limits Pitch to +/- 1/4pi Rad
ROLL_MAX    = single(pi/4);                                            % Limits Roll to +/- 1/4pi Rad
YAW_MAX     = single(pi);                                               % Limits Yaw to +/- 1 rad

X_MAX = single(1);                                                        % Limits X to +/- 2 Meters
Y_MAX = single(1);                                                        % Limits Y to +/- 2 Meters
Z_MAX = single(1);                                                        % Limits Z to 2 meters max

%% PWM Fit                                                                                                  a PWM^3 + b PWM^2 + c PWM + d
PWM_a = single(-1.0117e-08);                                              % a Constant 3-order polynomial least squares fit
PWM_b = single( 5.0206e-05);                                              % b Constant 3-order polynomial least squares fit 
PWM_c = single(-6.9900e-02);                                              % c Constant 3-order polynomial least squares fit
PWM_d = single( 2.9880e+01);                                              % d Constant 3-order polynomial least squares fit

%% Motor Mixer
% Thrust
T1 = single( 1.0000);                                                        % Thrust distribution motor 1 (right front CCW)
T2 = single( 1.0000);                                                        % Thrust distribution motor 2 (right back   CW)
T3 = single( 1.0000);                                                        % Thrust distribution motor 3 (left  back  CCW)
T4 = single( 1.0000);                                                        % Thrust distribution motor 4 (left  front  CW)

% Roll
R1 = single(-0.1430);                                                        % Roll   distribution motor 1 (right front CCW)
R2 = single(-0.1290);                                                        % Roll   distribution motor 2 (right back   CW)
R3 = single( 0.1290);                                                        % Roll   distribution motor 3 (left  back  CCW)
R4 = single( 0.1430);                                                        % Roll   distribution motor 4 (left  front  CW)

% Pitch
P1 = single( 0.21065);                                                       % Pitch  distribution motor 1 (right front CCW)
P2 = single(-0.17235);                                                       % Pitch  distribution motor 2 (right back   CW)
P3 = single(-0.17235);                                                       % Pitch  distribution motor 3 (left  back  CCW)
P4 = single( 0.21065);                                                       % Pitch  distribution motor 4 (left  front  CW)

% Yaw
Y1 = single( 0.2541);                                                        % Yaw    distribution motor 1 (right front CCW)
Y2 = single(-0.2158);                                                        % Yaw    distribution motor 2 (right back   CW)
Y3 = single( 0.2158);                                                        % Yaw    distribution motor 3 (left  back  CCW)
Y4 = single(-0.2541);                                                        % Yaw    distribution motor 4 (left  front  CW)

%% Data Communication
ReceiveBytes = single( 4);
% COM_RATE     = single(10),'COM_RATE'} );                                             % Communication rate in Hz
COM_RATE = double(10);
%% Madgewick AHRS Parameters
app     = single( [0.05; 0.05; 0.05]);                                      % Maximum expected acceleration during stationary state (peak - peak)
Kapp    = single( 1.2);                                                    % Stationary state detection accelerometer detection
thg     = single( [0.005; 0.005; 0.005]);                                   % Maximum expected gyroscope during stationary state (bound)
m0      = single( 0.452857);                                                 % Expected Magnetometer magnitude
Dip0    = single( 1.2);                                                    % Expected Dip angle under undisturbed operation
thdip   = single( 0.5);                                                   % Scaling dip error
Beta    = single( 0.02);                                                   % Gradient Descent Step size

%% New Observer Design Z-Dynamics
m = 1.4; fs = 1/Ts;  g = 9.81;
P_z  = c2d(ss([0 1; 0 0], [0 0; -1/(0.8*m) 1], [1 0], 0), 1/fs);
Pole_L_z = [0.95 0.9]; 
L_z = place(P_z.a', P_z.c', Pole_L_z).';
O_z = ss(P_z.a - L_z*P_z.c, [L_z], eye(size(P_z.a)), zeros(size(P_z.a,1), size([L_z],2)), 1/fs); Oz0 = zeros(size(P_z.a,1), 1);
O_z0 = zeros(size(P_z.a,1), 1);

Oza  = single(O_z.a);
Ozb  = single(O_z.b);
Ozc  = single(O_z.c);
Ozd  = single(O_z.d);
Oz0  = single(O_z0);

%% New Observer Design X-Dynamics
P_x  = c2d(ss([0 1; 0 0], [0; -g], [1 0], 0), 1/fs);
Pole_L_x = [0.95 0.9]; 
L_x = place(P_x.a', P_x.c', Pole_L_x).';
O_x = ss(P_x.a - L_x*P_x.c, [L_x], eye(size(P_x.a)), zeros(size(P_x.a,1), size([L_x],2)), 1/fs); Ox0 = zeros(size(P_x.a,1), 1);
O_x0 = zeros(size(P_x.a,1), 1);

Oxa  = single(O_x.a);
Oxb  = single(O_x.b);
Oxc  = single(O_x.c);
Oxd  = single(O_x.d);
Ox0  = single(O_x0);

%% New Observer Design Y-Dynamics
P_y  = c2d(ss([0 1; 0 0], [0; g], [1 0], 0), 1/fs);
Pole_L_y = [0.95 0.9]; 
L_y = place(P_y.a', P_y.c', Pole_L_y).';
O_y = ss(P_y.a - L_y*P_y.c, [L_y], eye(size(P_y.a)), zeros(size(P_y.a,1), size([L_y],2)), 1/fs); Oy0 = zeros(size(P_y.a,1), 1);
O_y0 = zeros(size(P_y.a,1), 1);

Oya  = single(O_y.a);
Oyb  = single(O_y.b);
Oyc  = single(O_y.c);
Oyd  = single(O_y.d);
Oy0  = single(O_y0);

%% Additional Control Settings
Kx  = single( [0.60 0.9]);
Ky  = single( [0.4 0.9]);
