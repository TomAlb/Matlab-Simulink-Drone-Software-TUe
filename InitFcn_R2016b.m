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
% Ts = Pixhawk_CSC.Parameter( {single(1/400), 'Ts'} );                                                      % Sample time
MOV_AVG_WINDOW = Pixhawk_CSC.Parameter( {single(100),'MOV_AVG_WINDOW'} );                                   % Moving average sample window size
% FILTER_N       = Pixhawk_CSC.Parameter( {single(10),'FILTER_N'} );                                        % Cut off frequency Derivative
Jxx = Pixhawk_CSC.Parameter( {single(J(1,1)),'Jxx'} );                                                      % Inertia around xx
Jyy = Pixhawk_CSC.Parameter( {single(J(2,2)),'Jyy'} );                                                      % Inertia around yy
Jzz = Pixhawk_CSC.Parameter( {single(J(3,3)),'Jzz'} );                                                      % Inertia around zz
clear J;

%% Sensor Calibration
% Sensor Calibration Magnetometer
CAL_MAG0_XXSCALE = Pixhawk_CSC.Parameter( {single(  1.01563   ),'CAL_MAG0_XXSCALE'} );                      % Set Magnetometer scaling in xx-direction
CAL_MAG0_XYSCALE = Pixhawk_CSC.Parameter( {single( -0.0612061 ),'CAL_MAG0_XYSCALE'} );                      % Set Magnetometer scaling in xy-direction
CAL_MAG0_XZSCALE = Pixhawk_CSC.Parameter( {single( -0.011532  ),'CAL_MAG0_XZSCALE'} );                      % Set Magnetometer scaling in xz-direction
CAL_MAG0_YXSCALE = Pixhawk_CSC.Parameter( {single( -0.0612061 ),'CAL_MAG0_YXSCALE'} );                      % Set Magnetometer scaling in yx-direction
CAL_MAG0_YYSCALE = Pixhawk_CSC.Parameter( {single(  1.00593   ),'CAL_MAG0_YYSCALE'} );                      % Set Magnetometer scaling in yy-direction
CAL_MAG0_YZSCALE = Pixhawk_CSC.Parameter( {single(  0.0144582 ),'CAL_MAG0_YZSCALE'} );                      % Set Magnetometer scaling in yz-direction
CAL_MAG0_ZXSCALE = Pixhawk_CSC.Parameter( {single( -0.011532  ),'CAL_MAG0_ZXSCALE'} );                      % Set Magnetometer scaling in zx-direction
CAL_MAG0_ZYSCALE = Pixhawk_CSC.Parameter( {single(  0.0144582 ),'CAL_MAG0_ZYSCALE'} );                      % Set Magnetometer scaling in zy-direction
CAL_MAG0_ZZSCALE = Pixhawk_CSC.Parameter( {single(  0.986813  ),'CAL_MAG0_ZZSCALE'} );                      % Set Magnetometer scaling in zz-direction

CAL_MAG0_XOFFSET = Pixhawk_CSC.Parameter( {single( 0.0446372  ),'CAL_MAG0_XOFFSET'} );                      % Set Magnetometer Hard Iron Corrections
CAL_MAG0_YOFFSET = Pixhawk_CSC.Parameter( {single( 0.0700776  ),'CAL_MAG0_YOFFSET'} );                      % Set Magnetometer Hard Iron Corrections
CAL_MAG0_ZOFFSET = Pixhawk_CSC.Parameter( {single( 0.00201742 ),'CAL_MAG0_ZOFFSET'} );                      % Set Magnetometer Hard Iron Corrections

CAL_MAG0_XCURCOR = Pixhawk_CSC.Parameter( {single( 0.0017     ),'CAL_MAG0_XCURCOR'} );                      % Set Magnetometer Current Corrections
CAL_MAG0_YCURCOR = Pixhawk_CSC.Parameter( {single(-0.0075     ),'CAL_MAG0_YCURCOR'} );                      % Set Magnetometer Current Corrections
CAL_MAG0_ZCURCOR = Pixhawk_CSC.Parameter( {single(-0.0080     ),'CAL_MAG0_ZCURCOR'} );                      % Set Magnetometer Current Corrections

% Sensor Calibration Accelerometer
CAL_ACC0_XXSCALE = Pixhawk_CSC.Parameter( {single(  1.01218    ),'CAL_ACC0_XXSCALE'} );                     % Set Accelerometer scaling in xx-direction
CAL_ACC0_XYSCALE = Pixhawk_CSC.Parameter( {single( -0.00443301 ),'CAL_ACC0_XYSCALE'} );                     % Set Accelerometer scaling in xy-direction
CAL_ACC0_XZSCALE = Pixhawk_CSC.Parameter( {single( -0.00505793 ),'CAL_ACC0_XZSCALE'} );                     % Set Accelerometer scaling in xz-direction
CAL_ACC0_YXSCALE = Pixhawk_CSC.Parameter( {single( -0.00443301 ),'CAL_ACC0_YXSCALE'} );                     % Set Accelerometer scaling in yx-direction
CAL_ACC0_YYSCALE = Pixhawk_CSC.Parameter( {single(  1.00234    ),'CAL_ACC0_YYSCALE'} );                     % Set Accelerometer scaling in yy-direction
CAL_ACC0_YZSCALE = Pixhawk_CSC.Parameter( {single(  0.0160883  ),'CAL_ACC0_YZSCALE'} );                     % Set Accelerometer scaling in yz-direction
CAL_ACC0_ZXSCALE = Pixhawk_CSC.Parameter( {single( -0.00505793 ),'CAL_ACC0_ZXSCALE'} );                     % Set Accelerometer scaling in zx-direction
CAL_ACC0_ZYSCALE = Pixhawk_CSC.Parameter( {single(  0.0160883  ),'CAL_ACC0_ZYSCALE'} );                     % Set Accelerometer scaling in zy-direction
CAL_ACC0_ZZSCALE = Pixhawk_CSC.Parameter( {single(  0.983604   ),'CAL_ACC0_ZZSCALE'} );                     % Set Accelerometer scaling in zz-direction

CAL_ACC0_XOFFSET = Pixhawk_CSC.Parameter( {single( 0.122052  ),'CAL_ACC0_XOFFSET'} );                       % Set Accelerometer offset in x-direction
CAL_ACC0_YOFFSET = Pixhawk_CSC.Parameter( {single( 0.00281205),'CAL_ACC0_YOFFSET'} );                       % Set Accelerometer offset in y-direction
CAL_ACC0_ZOFFSET = Pixhawk_CSC.Parameter( {single(-0.0298121 ),'CAL_ACC0_ZOFFSET'} );                       % Set Accelerometer offset in z-direction

% Sensor Calibration Gyroscope
CAL_GYRO0_XXSCALE = Pixhawk_CSC.Parameter( {single(0.992736),'CAL_GYRO0_XXSCALE'} );                        % Set Gyroscope scaling in xx-direction
CAL_GYRO0_XYSCALE = Pixhawk_CSC.Parameter( {single(0.0000),'CAL_GYRO0_XYSCALE'} );                          % Set Gyroscope scaling in xy-direction   
CAL_GYRO0_XZSCALE = Pixhawk_CSC.Parameter( {single(0.0000),'CAL_GYRO0_XZSCALE'} );                          % Set Gyroscope scaling in xz-direction
CAL_GYRO0_YXSCALE = Pixhawk_CSC.Parameter( {single(0.0000),'CAL_GYRO0_YXSCALE'} );                          % Set Gyroscope scaling in yx-direction
CAL_GYRO0_YYSCALE = Pixhawk_CSC.Parameter( {single(1.01609),'CAL_GYRO0_YYSCALE'} );                         % Set Gyroscope scaling in yy-direction   
CAL_GYRO0_YZSCALE = Pixhawk_CSC.Parameter( {single(0.0000),'CAL_GYRO0_YZSCALE'} );                          % Set Gyroscope scaling in yz-direction
CAL_GYRO0_ZXSCALE = Pixhawk_CSC.Parameter( {single(0.0000),'CAL_GYRO0_ZXSCALE'} );                          % Set Gyroscope scaling in zx-direction
CAL_GYRO0_ZYSCALE = Pixhawk_CSC.Parameter( {single(0.0000),'CAL_GYRO0_ZYSCALE'} );                          % Set Gyroscope scaling in zy-direction   
CAL_GYRO0_ZZSCALE = Pixhawk_CSC.Parameter( {single(1.03003),'CAL_GYRO0_ZZSCALE'} );                         % Set Gyroscope scaling in zz-direction

% Sensor Calibration Barometer
CAL_BAR_SCALE  = Pixhawk_CSC.Parameter( {single(   1.0632),'CAL_BAR_SCALE'} );                              % Set Barometer Scaling
CAL_BAR_OFFSET = Pixhawk_CSC.Parameter( {single( -20.8468),'CAL_BAR_OFFSET'} );                             % Set Barometer Offset

% Sensor Calibration GPS
CAL_GPS0_XXSCALE = Pixhawk_CSC.Parameter( {single(  0.0100),'CAL_GPS0_XXSCALE'} );                          % Set Magnetometer scaling in xx-direction
CAL_GPS0_XYSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_GPS0_XYSCALE'} );                          % Set Magnetometer scaling in xy-direction
CAL_GPS0_XZSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_GPS0_XZSCALE'} );                          % Set Magnetometer scaling in xz-direction
CAL_GPS0_YXSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_GPS0_YXSCALE'} );                          % Set Magnetometer scaling in yx-direction
CAL_GPS0_YYSCALE = Pixhawk_CSC.Parameter( {single(  0.0100),'CAL_GPS0_YYSCALE'} );                          % Set Magnetometer scaling in yy-direction
CAL_GPS0_YZSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_GPS0_YZSCALE'} );                          % Set Magnetometer scaling in yz-direction
CAL_GPS0_ZXSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_GPS0_ZXSCALE'} );                          % Set Magnetometer scaling in zx-direction
CAL_GPS0_ZYSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_GPS0_ZYSCALE'} );                          % Set Magnetometer scaling in zy-direction
CAL_GPS0_ZZSCALE = Pixhawk_CSC.Parameter( {single( -0.0010),'CAL_GPS0_ZZSCALE'} );                          % Set Magnetometer scaling in zz-direction

CAL_GPS0_XOFFSET = Pixhawk_CSC.Parameter( {single( 0.0000),'CAL_GPS0_XOFFSET'} );                           % Set Magnetometer Hard Iron Corrections
CAL_GPS0_YOFFSET = Pixhawk_CSC.Parameter( {single( 0.0000),'CAL_GPS0_YOFFSET'} );                           % Set Magnetometer Hard Iron Corrections
CAL_GPS0_ZOFFSET = Pixhawk_CSC.Parameter( {single( 0.0000),'CAL_GPS0_ZOFFSET'} );                           % Set Magnetometer Hard Iron Corrections

% Sensor Calibration Battery
CAL_BAT0_XXSCALE = Pixhawk_CSC.Parameter( {single(  1.0000),'CAL_BAT0_XXSCALE'} );                          % Set Magnetometer scaling in xx-direction
CAL_BAT0_XYSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_BAT0_XYSCALE'} );                          % Set Magnetometer scaling in xy-direction
CAL_BAT0_XZSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_BAT0_XZSCALE'} );                          % Set Magnetometer scaling in xz-direction
CAL_BAT0_YXSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_BAT0_YXSCALE'} );                          % Set Magnetometer scaling in yx-direction
CAL_BAT0_YYSCALE = Pixhawk_CSC.Parameter( {single(  1.0000),'CAL_BAT0_YYSCALE'} );                          % Set Magnetometer scaling in yy-direction
CAL_BAT0_YZSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_BAT0_YZSCALE'} );                          % Set Magnetometer scaling in yz-direction
CAL_BAT0_ZXSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_BAT0_ZXSCALE'} );                          % Set Magnetometer scaling in zx-direction
CAL_BAT0_ZYSCALE = Pixhawk_CSC.Parameter( {single(  0.0000),'CAL_BAT0_ZYSCALE'} );                          % Set Magnetometer scaling in zy-direction
CAL_BAT0_ZZSCALE = Pixhawk_CSC.Parameter( {single(  1.0000),'CAL_BAT0_ZZSCALE'} );                          % Set Magnetometer scaling in zz-direction

CAL_BAT0_XOFFSET = Pixhawk_CSC.Parameter( {single( 0.0000),'CAL_BAT0_XOFFSET'} );                           % Set Magnetometer Hard Iron Corrections
CAL_BAT0_YOFFSET = Pixhawk_CSC.Parameter( {single( 0.0000),'CAL_BAT0_YOFFSET'} );                           % Set Magnetometer Hard Iron Corrections
CAL_BAT0_ZOFFSET = Pixhawk_CSC.Parameter( {single( 0.0000),'CAL_BAT0_ZOFFSET'} );                           % Set Magnetometer Hard Iron Corrections

% Sensor Calibration Sonar
CAL_SONAR_SCALE  = Pixhawk_CSC.Parameter( {single(-438.1875),'CAL_SONAR_SCALE'} );                           % Set Sonar Scaling
CAL_SONAR_OFFSET = Pixhawk_CSC.Parameter( {single( -6.8730),'CAL_SONAR_OFFSET'} );                          % Set Sonar Offset

%% Sensor Fusion
% Low pass filter on 10Hz
LPF_REF_NUM = Pixhawk_CSC.Parameter( {single([0.0000  0.09516]),'LPF_REF_NUM'} );                           % Numerator Low Pass Filter Reference
LPF_REF_DEN = Pixhawk_CSC.Parameter( {single([1.0000 -0.9048]),'LPF_REF_DEN'} );                            % Denominator Low Pass Filter Reference

% Low pass filter on 20Hz
LPF_REF_NUM = Pixhawk_CSC.Parameter( {single([0.0000  0.1813]),'LPF_REF_NUM'} );                           % Numerator Low Pass Filter Reference
LPF_REF_DEN = Pixhawk_CSC.Parameter( {single([1.0000 -0.8187]),'LPF_REF_DEN'} );                            % Denominator Low Pass Filter Reference

%% Kalman filter settings Translational states
pn = [0.1; 0.1; 0.1];                                                                                       % Process noise Magnitude
mn = [0.01; 0.01; 0.01; 0.01];                                                                              % Measurement noise Magnitude
mn_1 = [0.01; 0.01; 0.01];                                                                              % Measurement noise Magnitude

A = Pixhawk_CSC.Parameter( {single([diag([1 1 1]) Ts*diag([1 1 1]); diag([0 0 0]) diag([1 1 1])]),'A'} );   % State transition matrix
B = Pixhawk_CSC.Parameter( {single([Ts^2/2*diag([1 1 1]); Ts*diag([1 1 1])]),'B'} );                        % Control input matrix
C = Pixhawk_CSC.Parameter( {single([diag([1 1 1]) 0*diag([1 1 1]); zeros(1,2) 1 zeros(1,3)]),'C'} );        % Measurement matrix
C_1 = Pixhawk_CSC.Parameter( {single([diag([1 1 1]) 0*diag([1 1 1])]),'C1'} );        % Measurement matrix
D = Pixhawk_CSC.Parameter( {single(zeros(size(C.Value,1), size(B.Value,2))),'D'} );                         % Feedthrough matrix

Q = Pixhawk_CSC.Parameter( {single(diag([pn.^2; pn.^2])*B.Value*B.Value'),'Q'} );                           % Convert the process noise (stdv) into covariance matrix
R = Pixhawk_CSC.Parameter( {single(diag([mn.^2])),'R'} );                                                   % Convert the measurement noise (stdv) into covariance matrix
R_1 = Pixhawk_CSC.Parameter( {single(diag([mn_1.^2])),'R_1'} );                                                   % Convert the measurement noise (stdv) into covariance matrix
I = Pixhawk_CSC.Parameter( {single(eye(size(Q.Value))),'I'} );                                              % Convert the measurement noise (stdv) into covariance matrix

%% Controller Parameters
C1  = Pixhawk_CSC.Parameter({single( 1.08680),'C1'} );                                                      % P-action Roll                             
C4  = Pixhawk_CSC.Parameter({single( 0.0500),'C4'} );                                                       % I-action Roll
C7  = Pixhawk_CSC.Parameter({single( 0.35937),'C7'} );                                                      % D-action Roll
C10 = Pixhawk_CSC.Parameter({single( 0.0000),'C10'} );                                                      % FF-action Roll

C2  = Pixhawk_CSC.Parameter({single( 1.10468),'C2'} );                                                      % P-action Pitch
C5  = Pixhawk_CSC.Parameter({single( 0.0500),'C5'} );                                                       % I-action Pitch
C8  = Pixhawk_CSC.Parameter({single( 0.36528),'C8'} );                                                      % D-action Pitch
C11 = Pixhawk_CSC.Parameter({single(-0.2300),'C11'} );                                                      % FF-action Pitch

C3  = Pixhawk_CSC.Parameter({single( 1.1980),'C3'} );                                                       % P-action Yaw
C6  = Pixhawk_CSC.Parameter({single( 0.0500),'C6'} );                                                       % I-action Yaw
C9  = Pixhawk_CSC.Parameter({single( 0.53311),'C9'} );                                                       % D-action Yaw
C12 = Pixhawk_CSC.Parameter({single( 0.0000),'C12'} );                                                      % FF-action Yaw

C13 = Pixhawk_CSC.Parameter({single( 0.8000),'C13'} );                                                      % P-action X 
C16 = Pixhawk_CSC.Parameter({single( 0.1000),'C16'} );                                                      % I-action X
C19 = Pixhawk_CSC.Parameter({single( 0.6000),'C19'} );                                                      % D-action X
C22 = Pixhawk_CSC.Parameter({single( 0.0000),'C22'} );                                                      % FF-action X

C14 = Pixhawk_CSC.Parameter({single( 0.8000),'C14'} );                                                      % P-action Y
C17 = Pixhawk_CSC.Parameter({single( 0.1000),'C17'} );                                                      % I-action Y
C20 = Pixhawk_CSC.Parameter({single( 0.6000),'C20'} );                                                      % D-action Y
C23 = Pixhawk_CSC.Parameter({single( 0.0000),'C23'} );                                                      % FF-action Y

C15 = Pixhawk_CSC.Parameter({single( 1.73200),'C15'} );                                                      % P-action Z
C18 = Pixhawk_CSC.Parameter({single( 0.0500),'C18'} );                                                      % I-action Z
C21 = Pixhawk_CSC.Parameter({single( 1.04558),'C21'} );                                                      % D-action Z
C24 = Pixhawk_CSC.Parameter({single(-11.0000),'C24'} );                                                     % FF-action Z

%% RC controller Calibration                                                                                normalizes input range (Roll, Pitch, Yaw -> [-1, 1] Thrust -> [0, 1])
% Roll
RC1_MIN     = Pixhawk_CSC.Parameter( {single(984),'RC1_MIN'}  );                                            % Minimal value RC channel 1 (Roll)
RC1_MAX     = Pixhawk_CSC.Parameter( {single(2006),'RC1_MAX'}  );                                           % Maximal value RC channel 1 (Roll)
RC1_TRIM    = Pixhawk_CSC.Parameter( {single(1496),'RC1_TRIM'}  );                                          % Trim value RC channel 1 (Roll) => 0.5 * (RC1_MIN + RC1_MAX)
RC1_DB      = Pixhawk_CSC.Parameter( {single(5),'RC1_DB'}  );                                               % RC Channel 1 DeadBand

% Yaw
RC2_MIN     = Pixhawk_CSC.Parameter( {single(982),'RC2_MIN'}  );                                            % Minimal value RC channel 2 (Yaw)
RC2_MAX     = Pixhawk_CSC.Parameter( {single(2006),'RC2_MAX'}  );                                           % Maximal value RC channel 2 (Yaw)
RC2_TRIM    = Pixhawk_CSC.Parameter( {single(1495),'RC2_TRIM'}  );                                          % Trim value RC channel 2 (Yaw) => 0.5 * (RC2_MIN + RC2_MAX)
RC2_DB      = Pixhawk_CSC.Parameter( {single(5),'RC2_DB'}  );                                               % RC Channel 1 DeadBand

% Pitch
RC3_MIN     = Pixhawk_CSC.Parameter( {single(982),'RC3_MIN'}  );                                            % Minimal value RC channel 3 (Pitch)
RC3_MAX     = Pixhawk_CSC.Parameter( {single(2006),'RC3_MAX'}  );                                           % Maximal value RC channel 3 (Pitch)
RC3_TRIM    = Pixhawk_CSC.Parameter( {single(1522),'RC3_TRIM'}  );                                          % Trim value RC channel 3 (Pitch) => 0.5 * (RC3_MIN + RC3_MAX)
RC3_DB      = Pixhawk_CSC.Parameter( {single(5),'RC3_DB'}  );                                               % RC Channel 3 DeadBand

% Throttle
RC4_MIN     = Pixhawk_CSC.Parameter( {single(1000),'RC4_MIN'}  );                                           % Minimal value RC channel 4 (Thrust)
RC4_MAX     = Pixhawk_CSC.Parameter( {single(2000),'RC4_MAX'}  );                                           % Maximal value RC channel 4 (Thrust)
RC4_TRIM    = Pixhawk_CSC.Parameter( {single(1500),'RC4_TRIM'}  );                                          % Trim value RC channel 4 (Thrust) => 0.5 * (RC4_MIN + RC4_MAX)
RC4_DB      = Pixhawk_CSC.Parameter( {single(5),'RC4_DB'}  );                                               % RC Channel 4 DeadBand

%% MIN MAX YAW/PITCH/ROLL                                                                                   Converts normalized inputs to desired units (SI)
PITCH_MAX   = Pixhawk_CSC.Parameter({single(pi/4),'PITCH_MAX'} );                                           % Limits Pitch to +/- 1/4pi Rad
ROLL_MAX    = Pixhawk_CSC.Parameter({single(pi/4),'ROLL_MAX'} );                                            % Limits Roll to +/- 1/4pi Rad
YAW_MAX     = Pixhawk_CSC.Parameter({single(pi),'YAW_MAX'} );                                               % Limits Yaw to +/- 1 rad

X_MAX = Pixhawk_CSC.Parameter({single(1),'X_MAX'} );                                                        % Limits X to +/- 2 Meters
Y_MAX = Pixhawk_CSC.Parameter({single(1),'Y_MAX'} );                                                        % Limits Y to +/- 2 Meters
Z_MAX = Pixhawk_CSC.Parameter({single(1),'Z_MAX'} );                                                        % Limits Z to 2 meters max

%% PWM Fit                                                                                                  a PWM^3 + b PWM^2 + c PWM + d
PWM_a = Pixhawk_CSC.Parameter({single(-1.0117e-08),'PWM_a'} );                                              % a Constant 3-order polynomial least squares fit
PWM_b = Pixhawk_CSC.Parameter({single( 5.0206e-05),'PWM_b'} );                                              % b Constant 3-order polynomial least squares fit 
PWM_c = Pixhawk_CSC.Parameter({single(-6.9900e-02),'PWM_c'} );                                              % c Constant 3-order polynomial least squares fit
PWM_d = Pixhawk_CSC.Parameter({single( 2.9880e+01),'PWM_d'} );                                              % d Constant 3-order polynomial least squares fit

%% Motor Mixer
% Thrust
T1 = Pixhawk_CSC.Parameter({single( 1.0000),'T1'} );                                                        % Thrust distribution motor 1 (right front CCW)
T2 = Pixhawk_CSC.Parameter({single( 1.0000),'T2'} );                                                        % Thrust distribution motor 2 (right back   CW)
T3 = Pixhawk_CSC.Parameter({single( 1.0000),'T3'} );                                                        % Thrust distribution motor 3 (left  back  CCW)
T4 = Pixhawk_CSC.Parameter({single( 1.0000),'T4'} );                                                        % Thrust distribution motor 4 (left  front  CW)

% Roll
R1 = Pixhawk_CSC.Parameter({single(-0.1430),'R1'} );                                                        % Roll   distribution motor 1 (right front CCW)
R2 = Pixhawk_CSC.Parameter({single(-0.1290),'R2'} );                                                        % Roll   distribution motor 2 (right back   CW)
R3 = Pixhawk_CSC.Parameter({single( 0.1290),'R3'} );                                                        % Roll   distribution motor 3 (left  back  CCW)
R4 = Pixhawk_CSC.Parameter({single( 0.1430),'R4'} );                                                        % Roll   distribution motor 4 (left  front  CW)

% Pitch
P1 = Pixhawk_CSC.Parameter({single( 0.21065),'P1'} );                                                       % Pitch  distribution motor 1 (right front CCW)
P2 = Pixhawk_CSC.Parameter({single(-0.17235),'P2'} );                                                       % Pitch  distribution motor 2 (right back   CW)
P3 = Pixhawk_CSC.Parameter({single(-0.17235),'P3'} );                                                       % Pitch  distribution motor 3 (left  back  CCW)
P4 = Pixhawk_CSC.Parameter({single( 0.21065),'P4'} );                                                       % Pitch  distribution motor 4 (left  front  CW)

% Yaw
Y1 = Pixhawk_CSC.Parameter({single( 0.2541),'Y1'} );                                                        % Yaw    distribution motor 1 (right front CCW)
Y2 = Pixhawk_CSC.Parameter({single(-0.2158),'Y2'} );                                                        % Yaw    distribution motor 2 (right back   CW)
Y3 = Pixhawk_CSC.Parameter({single( 0.2158),'Y3'} );                                                        % Yaw    distribution motor 3 (left  back  CCW)
Y4 = Pixhawk_CSC.Parameter({single(-0.2541),'Y4'} );                                                        % Yaw    distribution motor 4 (left  front  CW)

%% Data Communication
ReceiveBytes = Pixhawk_CSC.Parameter({single( 4),'ReceiveBytes'} );
% COM_RATE     = Pixhawk_CSC.Parameter({single(10),'COM_RATE'} );                                             % Communication rate in Hz
COM_RATE = double(10);
%% Madgewick AHRS Parameters
app     = Pixhawk_CSC.Parameter({single( [0.05; 0.05; 0.05]),'app'} );                                      % Maximum expected acceleration during stationary state (peak - peak)
Kapp    = Pixhawk_CSC.Parameter({single( 1.2),'Kapp'} );                                                    % Stationary state detection accelerometer detection
thg     = Pixhawk_CSC.Parameter({single( [0.005; 0.005; 0.005]),'thg'} );                                   % Maximum expected gyroscope during stationary state (bound)
m0      = Pixhawk_CSC.Parameter({single( 0.452857),'m0'} );                                                 % Expected Magnetometer magnitude
Dip0    = Pixhawk_CSC.Parameter({single( 1.2),'Dip0'} );                                                    % Expected Dip angle under undisturbed operation
thdip   = Pixhawk_CSC.Parameter({single( 0.5),'thdip'} );                                                   % Scaling dip error
Beta    = Pixhawk_CSC.Parameter({single( 0.02),'Beta'} );                                                   % Gradient Descent Step size

%% New Observer Design Z-Dynamics
m = 1.4; fs = 1/Ts;  g = 9.81;
P_z  = c2d(ss([0 1; 0 0], [0 0; -1/(0.8*m) 1], [1 0], 0), 1/fs);
Pole_L_z = [0.95 0.9]; 
L_z = place(P_z.a', P_z.c', Pole_L_z).';
O_z = ss(P_z.a - L_z*P_z.c, [L_z], eye(size(P_z.a)), zeros(size(P_z.a,1), size([L_z],2)), 1/fs); Oz0 = zeros(size(P_z.a,1), 1);
O_z0 = zeros(size(P_z.a,1), 1);

Oza  = Pixhawk_CSC.Parameter({single(O_z.a),'Oza'} );
Ozb  = Pixhawk_CSC.Parameter({single(O_z.b),'Ozb'} );
Ozc  = Pixhawk_CSC.Parameter({single(O_z.c),'Ozc'} );
Ozd  = Pixhawk_CSC.Parameter({single(O_z.d),'Ozd'} );
Oz0  = Pixhawk_CSC.Parameter({single(O_z0),'Oz0'} );

%% New Observer Design X-Dynamics
P_x  = c2d(ss([0 1; 0 0], [0; -g], [1 0], 0), 1/fs);
Pole_L_x = [0.95 0.9]; 
L_x = place(P_x.a', P_x.c', Pole_L_x).';
O_x = ss(P_x.a - L_x*P_x.c, [L_x], eye(size(P_x.a)), zeros(size(P_x.a,1), size([L_x],2)), 1/fs); Ox0 = zeros(size(P_x.a,1), 1);
O_x0 = zeros(size(P_x.a,1), 1);

Oxa  = Pixhawk_CSC.Parameter({single(O_x.a),'Oxa'} );
Oxb  = Pixhawk_CSC.Parameter({single(O_x.b),'Oxb'} );
Oxc  = Pixhawk_CSC.Parameter({single(O_x.c),'Oxc'} );
Oxd  = Pixhawk_CSC.Parameter({single(O_x.d),'Oxd'} );
Ox0  = Pixhawk_CSC.Parameter({single(O_x0),'Ox0'} );

%% New Observer Design Y-Dynamics
P_y  = c2d(ss([0 1; 0 0], [0; g], [1 0], 0), 1/fs);
Pole_L_y = [0.95 0.9]; 
L_y = place(P_y.a', P_y.c', Pole_L_y).';
O_y = ss(P_y.a - L_y*P_y.c, [L_y], eye(size(P_y.a)), zeros(size(P_y.a,1), size([L_y],2)), 1/fs); Oy0 = zeros(size(P_y.a,1), 1);
O_y0 = zeros(size(P_y.a,1), 1);

Oya  = Pixhawk_CSC.Parameter({single(O_y.a),'Oya'} );
Oyb  = Pixhawk_CSC.Parameter({single(O_y.b),'Oyb'} );
Oyc  = Pixhawk_CSC.Parameter({single(O_y.c),'Oyc'} );
Oyd  = Pixhawk_CSC.Parameter({single(O_y.d),'Oyd'} );
Oy0  = Pixhawk_CSC.Parameter({single(O_y0),'Oy0'} );

%% Additional Control Settings
Kx  = Pixhawk_CSC.Parameter({single( [0.60 0.9]),'Kx'} );
Ky  = Pixhawk_CSC.Parameter({single( [0.4 0.9]),'Ky'} );
