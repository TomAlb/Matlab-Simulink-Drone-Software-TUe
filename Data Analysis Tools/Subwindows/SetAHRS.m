function varargout = SetAHRS(varargin)
% SETAHRS MATLAB code for SetAHRS.fig
%      SETAHRS, by itself, creates a new SETAHRS or raises the existing
%      singleton*.
%
%      H = SETAHRS returns the handle to a new SETAHRS or the handle to
%      the existing singleton*.
%
%      SETAHRS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETAHRS.M with the given input arguments.
%
%      SETAHRS('Property','Value',...) creates a new SETAHRS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SetAHRS_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SetAHRS_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SetAHRS

% Last Modified by GUIDE v2.5 22-May-2018 20:45:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SetAHRS_OpeningFcn, ...
                   'gui_OutputFcn',  @SetAHRS_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SetAHRS is made visible.
function SetAHRS_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SetAHRS (see VARARGIN)

% Choose default command line output for SetAHRS
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end
handles.AxesPlotFlags = false(11,1);
% Update handles structure

guidata(hObject, handles);
InitializeData(hObject, eventdata, handles);
handles = guidata(hObject);

Madgewick_AHRS_STAT_DETECT(hObject, handles)
handles = guidata(hObject);

Madgewick_AHRS_Dynamics(hObject, handles);
handles = guidata(hObject);

PlotAngles(handles)

% UIWAIT makes SetAHRS wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SetAHRS_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function InitializeData(hObject, eventdata, handles)
% Load Parameters

% Load initial AHRS Settings
handles.AHRS.Ts = str2num(handles.Ts_edit.String);
handles.AHRS.app = [str2num(handles.app_x_edit.String); str2num(handles.app_y_edit.String); str2num(handles.app_z_edit.String)];
handles.AHRS.Kapp  = str2num(handles.Kapp_edit.String);
handles.AHRS.thg = [str2num(handles.thg_x_edit.String); str2num(handles.thg_y_edit.String); str2num(handles.thg_z_edit.String)];
handles.AHRS.m0  = str2num(handles.m0_edit.String);
handles.AHRS.Dip0  = str2num(handles.Dip0_edit.String);
handles.AHRS.thdip  = str2num(handles.thdip_edit.String);
handles.AHRS.Beta  = str2num(handles.Beta_edit.String);

% Load needed data
handles.Data.CLOCK_DATA = handles.MainData.DataSet.CLOCK(5,:);
handles.Data.MAGNETO_DATA  = handles.MainData.DataSet.FILTERED_SENSOR_DATA(1:3,:);
handles.Data.ACC_DATA      = handles.MainData.DataSet.FILTERED_SENSOR_DATA(4:6,:);
handles.Data.GYRO_DATA     = handles.MainData.DataSet.FILTERED_SENSOR_DATA(7:9,:);


% Set primary axes
handles.PlotData.X = handles.Data.CLOCK_DATA;
handles.PlotData.Y1 = handles.Data.ACC_DATA; 
handles.AxesPlotFlags(1) = true;
handles.PlotData.Y2 = handles.Data.GYRO_DATA;
handles.AxesPlotFlags(2) = true;
handles.PlotData.Y8 = handles.Data.MAGNETO_DATA;
handles.AxesPlotFlags(8) = true;
guidata(hObject, handles)


function PlotAngles(handles)
% handles.PlotData.Y4
AxesList = {'Acc_Axes',            'Gyro_Axes',      'Switch_Axes',       'Indep_Switch_Axes',  'IMU_Axes', 'MIMU_Axes', 'MadgeWick_Axes', 'Magneto_Axes',      'Dip_Amp_Axes',         'Lambda_Axes'};
TitleList = {'Accelerometer Data', 'Gyroscope Data', 'Stationary Switch', 'Independent switch', 'IMU AHRS', 'MIMU AHRS', 'Madgewick AHRS', 'Magnetometer Data', 'Disturbance Severity', 'Magnetometer quality'};
YDataList = {'Y1', 'Y2', 'Y3', 'Y4', 'Y5', 'Y6', 'Y7', 'Y8', 'Y9', 'Y10'};
ylabelList = {'Acceleration (m/s^2)', 'Angular rate (rad/s)', '', '', '', 'Flux (G)', '', '', '', ''};
LegendList = {'Legend1', 'Legend2', 'Legend3', 'Legend4', 'Legend5', 'Legend6', 'Legend7', 'Legend8', 'Legend9', 'Legend10'};
FlagList = {'Flags1', 'Flags2', 'Flags3'};
Color = [1 0 0; 0 1 0; 0 0 1; 1 1 0; 1 0 1; 0 1 1; 0.50 0 0; 0 0.50 0; 0 0 0.50; 0.25 0.25 0.25];
MarkerSymbol = {'none','none','.','.','none','none','none','none','none','none'};
LineStyle = {'-','-','none','none','-','-','-','-','-','-'};

handles.PlotData.Legend1 = {'a_x', 'a_y', 'a_z'};
handles.PlotData.Legend2 = {'\omega_x', '\omega_y', '\omega_z'};
handles.PlotData.Legend3 = {'Switching'};
handles.PlotData.Legend4 = {'a_x', 'a_y', 'a_z','\omega_x', '\omega_y', '\omega_z'};
handles.PlotData.Legend5 = {'\phi_{x}', '\theta_{y}', '\psi_{z}'};
handles.PlotData.Legend6 = {'\phi_{x}', '\theta_{y}', '\psi_{z}'};
handles.PlotData.Legend7 = {'\phi_{x}', '\theta_{y}', '\psi_{z}'};
handles.PlotData.Legend8 = {'B_x', 'B_y', 'B_z'};
handles.PlotData.Legend9 = {'Magnitude', 'Dip Angle'};
handles.PlotData.Legend10 = {'\lambda_{Mag}', '\lambda_{Dip}', '\lambda'};
List = [];
% Reset figure and plot basic sensor data
for i = 1:length(AxesList)
    cla(handles.(AxesList{i}), 'reset')  
    
    if(handles.AxesPlotFlags(i) == true)
        hold(handles.(AxesList{i}), 'on')
        for j = 1:size(handles.PlotData.(YDataList{i}), 1)
            plot(handles.(AxesList{i}), handles.PlotData.X, handles.PlotData.(YDataList{i})(j,:), 'color', Color(j, :), 'marker', MarkerSymbol{i}, 'linestyle', LineStyle{i})
        end
        grid(handles.(AxesList{i}), 'on')
        xlabel(handles.(AxesList{i}), 'Time')
        ylabel(handles.(AxesList{i}), ylabelList{i})
        title(handles.(AxesList{i}), TitleList{i})
        axis(handles.(AxesList{i}),  'tight')
        legend(handles.(AxesList{i}), handles.PlotData.(LegendList{i}), 'location', 'best')
    else
        grid(handles.(AxesList{i}), 'on')
        xlabel(handles.(AxesList{i}), 'Time')
        ylabel(handles.(AxesList{i}), 'Y-Data')
        title(handles.(AxesList{i}), TitleList{i})
    end
    List = [List handles.(AxesList{i})];
end
linkaxes(List, 'x');

function Madgewick_AHRS_STAT_DETECT(hObject, handles)
% Stationary state detection
app = handles.AHRS.app;
Kapp = handles.AHRS.Kapp;
tha = Kapp * app;
thg = handles.AHRS.thg;
Index = zeros(6, length(handles.Data.CLOCK_DATA));
Ind = zeros(1, length(handles.Data.CLOCK_DATA));

for i = 2:length(handles.Data.CLOCK_DATA)
    Index(1,i) = max(abs(diff(handles.Data.ACC_DATA(1,i-1:i)))) < tha(1);
    Index(2,i) = max(abs(diff(handles.Data.ACC_DATA(2,i-1:i)))) < tha(2);
    Index(3,i) = max(abs(diff(handles.Data.ACC_DATA(3,i-1:i)))) < tha(2);
    Index(4,i) = abs(handles.Data.GYRO_DATA(1,i)) < thg(1);
    Index(5,i) = abs(handles.Data.GYRO_DATA(1,i)) < thg(2);
    Index(6,i) = abs(handles.Data.GYRO_DATA(1,i)) < thg(3);
    Ind(:,i) = all(Index(:,i));
end
handles.PlotData.Y4 = [1; 2; 3; 4; 5; 6].*(2.*Index-1); 
handles.PlotData.Y3 = Ind; 
handles.AHRS.Index = Ind;
handles.AxesPlotFlags(4) = true;
handles.AxesPlotFlags(3) = true;
guidata(hObject, handles)

function Madgewick_AHRS_Dynamics(hObject, handles)
% Magnetic Disturbance detection

q0IMU = [ones(1, length(handles.Data.CLOCK_DATA)); zeros(3, length(handles.Data.CLOCK_DATA))];
q0MIMU = [ones(1, length(handles.Data.CLOCK_DATA)); zeros(3, length(handles.Data.CLOCK_DATA))];
q0AHRSIMU = [ones(1, length(handles.Data.CLOCK_DATA)); zeros(3, length(handles.Data.CLOCK_DATA))];
q0AHRSMIMU = [ones(1, length(handles.Data.CLOCK_DATA)); zeros(3, length(handles.Data.CLOCK_DATA))];
q0AHRS = [ones(1, length(handles.Data.CLOCK_DATA)); zeros(3, length(handles.Data.CLOCK_DATA))];

EulerIMU = zeros(3, length(handles.Data.CLOCK_DATA));
EulerMIMU = zeros(3, length(handles.Data.CLOCK_DATA));
EulerAHRS = zeros(3, length(handles.Data.CLOCK_DATA)); 

ThetaDip = zeros(1, length(handles.Data.CLOCK_DATA));
MagDist = zeros(1, length(handles.Data.CLOCK_DATA));

Index = handles.AHRS.Index;
m0 = handles.AHRS.m0;
Ts = handles.AHRS.Ts;
Beta = handles.AHRS.Beta;
Dip0 = handles.AHRS.Dip0;
thdip = handles.AHRS.thdip;
Lambda = zeros(3, length(handles.Data.CLOCK_DATA));

% Set initial Psi
Psi = -atan2(handles.Data.MAGNETO_DATA(2,1), handles.Data.MAGNETO_DATA(1,1));
Rz = [cos(Psi) -sin(Psi) 0; sin(Psi) cos(Psi) 0; 0 0 1];

for i = 2:length(handles.Data.CLOCK_DATA)
    %% IMU
    Accelerometer = handles.Data.ACC_DATA(:,i)';
    Gyroscope = handles.Data.GYRO_DATA(:,i)';
    q = q0IMU(:,i-1)'; % short name local variable for readability
    
    % Normalise accelerometer measurement
    Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude
    
    % Gradient decent algorithm corrective step
    F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
        2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
        2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)];
    J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
        2*q(2),     2*q(1),     2*q(4),	2*q(3)
        0,         -4*q(2),    -4*q(3),	0    ];
    step = (J'*F);
    step = step / norm(step);	% normalise step magnitude
    
    % Compute rate of change of quaternion
    a = q;
    b = [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)];
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    qDot = 0.5 * ab - Beta * step';
    
    % Integrate to yield quaternion
    q = q + qDot * Ts;
    q0IMU(:,i) = q / norm(q); % normalise quaternion
    
    %% MIMU
    Accelerometer = handles.Data.ACC_DATA(:,i)';
    Gyroscope = handles.Data.GYRO_DATA(:,i)';
    Magnetometer = (Rz * handles.Data.MAGNETO_DATA(:,i))';
    
    q = q0MIMU(:,i-1)'; % short name local variable for readability
    
    % Normalise accelerometer measurement
    Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude
    
    % Normalise magnetometer measurement
    Magnetometer = Magnetometer / norm(Magnetometer);	% normalise magnitude
    
    % Reference direction of Earth's magnetic feild
    qConj = [q(1) -q(2) -q(3) -q(4)];
    
    a = [0 Magnetometer];
    b = qConj;
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    
    a = q;
    b = ab;
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    
    h = ab;
    b = [0 norm([h(2) h(3)]) 0 h(4)];
    
    % Gradient decent algorithm corrective step
    F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
          2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
          2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)
          2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - Magnetometer(1)
          2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - Magnetometer(2)
          2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - Magnetometer(3)];
    J = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
          2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
          0,                         -4*q(2),                    -4*q(3),                         0
          -2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
          -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
          2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];
    step = (J'*F);
    step = step / norm(step);	% normalise step magnitude
    
    % Compute rate of change of quaternion    
    a = q;
    b = [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)];
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    
    qDot = 0.5 * ab - Beta * step';
    
    % Integrate to yield quaternion
    q = q + qDot * Ts;
    q0MIMU(:,i) = q / norm(q); % normalise quaternion
    
    %% AHRS
    % IMU
    Accelerometer = handles.Data.ACC_DATA(:,i)';
    Gyroscope = handles.Data.GYRO_DATA(:,i)';
    q = q0AHRS(:,i-1)'; % short name local variable for readability
    
    % Normalise accelerometer measurement
    Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude
    
    % Gradient decent algorithm corrective step
    F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
        2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
        2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)];
    J = [-2*q(3),	2*q(4),    -2*q(1),	2*q(2)
        2*q(2),     2*q(1),     2*q(4),	2*q(3)
        0,         -4*q(2),    -4*q(3),	0    ];
    step = (J'*F);
    step = step / norm(step);	% normalise step magnitude
    
    % Compute rate of change of quaternion
    a = q;
    b = [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)];
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    
    qDot = 0.5 * ab - Beta * step';
    
    % Integrate to yield quaternion
    q = q + qDot * Ts;
    q0AHRSIMU(:,i) = q / norm(q); % normalise quaternion
    
    % MIMU
    Accelerometer = handles.Data.ACC_DATA(:,i)';
    Gyroscope = handles.Data.GYRO_DATA(:,i)';
    Magnetometer = (Rz * handles.Data.MAGNETO_DATA(:,i))';
    
    q = q0AHRS(:,i-1)'; % short name local variable for readability
    
    % Normalise accelerometer measurement
    Accelerometer = Accelerometer / norm(Accelerometer);	% normalise magnitude
    
    % Normalise magnetometer measurement
    Magnetometer = Magnetometer / norm(Magnetometer);	% normalise magnitude
    
    % Reference direction of Earth's magnetic feild
    qConj = [q(1) -q(2) -q(3) -q(4)];
    
    a = [0 Magnetometer];
    b = qConj;
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    
    a = q;
    b = ab;
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    
    h = ab;
    b = [0 norm([h(2) h(3)]) 0 h(4)];
    
    % Gradient decent algorithm corrective step
    F = [2*(q(2)*q(4) - q(1)*q(3)) - Accelerometer(1)
          2*(q(1)*q(2) + q(3)*q(4)) - Accelerometer(2)
          2*(0.5 - q(2)^2 - q(3)^2) - Accelerometer(3)
          2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - Magnetometer(1)
          2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - Magnetometer(2)
          2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - Magnetometer(3)];
    J = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
          2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
          0,                         -4*q(2),                    -4*q(3),                         0
          -2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
          -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
          2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),        2*b(2)*q(2)];
    step = (J'*F);
    step = step / norm(step);	% normalise step magnitude
    
    % Compute rate of change of quaternion    
    a = q;
    b = [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)];
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
    
    qDot = 0.5 * ab - Beta * step';
    
    % Integrate to yield quaternion
    q = q + qDot * Ts;
    q0AHRSMIMU(:,i) = q / norm(q); % normalise quaternion
    
    % Evaluate magnetic disturbance
    MagDist(1,i) = sqrt(sum(handles.Data.MAGNETO_DATA(:,i).^2)) - m0;
    Lambda(1,i) = abs(sqrt(sum(handles.Data.MAGNETO_DATA(:,i).^2)) - m0)/m0;
    if(Lambda(1,i) > 1)
        Lambda(1,i) = 1;
    end
    
    q_rot = q0AHRS(:,i-1)';
    R(1,1,:) = 2.*q_rot(:,1).^2-1+2.*q_rot(:,2).^2;
    R(1,2,:) = 2.*(q_rot(:,2).*q_rot(:,3)+q_rot(:,1).*q_rot(:,4));
    R(1,3,:) = 2.*(q_rot(:,2).*q_rot(:,4)-q_rot(:,1).*q_rot(:,3));
    R(2,1,:) = 2.*(q_rot(:,2).*q_rot(:,3)-q_rot(:,1).*q_rot(:,4));
    R(2,2,:) = 2.*q_rot(:,1).^2-1+2.*q_rot(:,3).^2;
    R(2,3,:) = 2.*(q_rot(:,3).*q_rot(:,4)+q_rot(:,1).*q_rot(:,2));
    R(3,1,:) = 2.*(q_rot(:,2).*q_rot(:,4)+q_rot(:,1).*q_rot(:,3));
    R(3,2,:) = 2.*(q_rot(:,3).*q_rot(:,4)-q_rot(:,1).*q_rot(:,2));
    R(3,3,:) = 2.*q_rot(:,1).^2-1+2.*q_rot(:,4).^2;
    
    h = R * Magnetometer';
    g = [0; 0; 1];
%     ThetaDip(:,i) = (pi/2 - acos( dot(h,g)/( sqrt(dot(h,h)) * sqrt(dot(g,g)) ) ) ) - Dip0;
    ThetaDip(:,i) = (pi/2 - acos( h'*g/( sqrt(h'*h) * sqrt(g'*g) ) ) ) - Dip0;
    Lambda(2,i) = abs(ThetaDip(:,i))/thdip;
    if(Lambda(2,i) > 1) 
        Lambda(2,i) = 1; 
    end
    
    Lambda(3,i) = (Lambda(1,i) + Lambda(2,i))/2;
    
    if( all(Index(:,i)) )
        q0AHRS(:,i) = q0AHRS(:,i-1);
    else
        q0AHRS(:,i) = Lambda(3,i) * q0AHRSIMU(:,i) + (1-Lambda(3,i)) * q0AHRSMIMU(:,i);
    end
    
          
    %% Convert to euler angles
    % Imu
    q = q0IMU(:,i)';
    qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
    
    q = qConj;
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
    psi = atan2(R(2,1,:), R(1,1,:) );
    
    EulerIMU(:,i) = [phi(1,:); theta(1,:); psi(1,:)];
    
    % MIMU
    q = q0MIMU(:,i)';
    qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
    
    q = qConj;
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
    psi = atan2(R(2,1,:), R(1,1,:) );
    
    EulerMIMU(:,i) = [phi(1,:); theta(1,:); psi(1,:)];
    
    % MIMU
    q = q0AHRS(:,i)';
    qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
    
    q = qConj;
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
    psi = atan2(R(2,1,:), R(1,1,:) );
    
    EulerAHRS(:,i) = [phi(1,:); theta(1,:); psi(1,:)];

end

handles.PlotData.Y9 = [MagDist; ThetaDip]; 
handles.AxesPlotFlags(9) = true;

handles.PlotData.Y10 = Lambda; 
handles.AxesPlotFlags(10) = true;

handles.PlotData.Y5 = EulerIMU;
handles.AxesPlotFlags(5) = true;

handles.PlotData.Y6 = EulerMIMU;
handles.AxesPlotFlags(6) = true;

handles.PlotData.Y7 = EulerAHRS;
handles.AxesPlotFlags(7) = true;

guidata(hObject, handles)

function Ts_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Ts_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ts_edit as text
%        str2double(get(hObject,'String')) returns contents of Ts_edit as a double


% --- Executes during object creation, after setting all properties.
function Ts_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ts_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Set_Btn.
function Set_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to Set_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Static
app = [str2num(handles.app_x_edit.String); str2num(handles.app_y_edit.String); str2num(handles.app_z_edit.String)];
Kapp  = str2num(handles.Kapp_edit.String);
thg = [str2num(handles.thg_x_edit.String); str2num(handles.thg_y_edit.String); str2num(handles.thg_z_edit.String)];

if(any([(handles.AHRS.app ~= app); (handles.AHRS.Kapp ~= Kapp); (handles.AHRS.thg ~= thg)]))
    handles.AHRS.app = app;
    handles.AHRS.Kapp = Kapp;
    handles.AHRS.thg = thg;

    guidata(hObject, handles);
    Madgewick_AHRS_STAT_DETECT(hObject, handles);
    handles = guidata(hObject);

    Madgewick_AHRS_Dynamics(hObject, handles)
    handles = guidata(hObject);

    PlotAngles(handles);    
end

%% Dynamics
Ts = str2num(handles.Ts_edit.String);
m0  = str2num(handles.m0_edit.String);
Dip0  = str2num(handles.Dip0_edit.String);
thdip  = str2num(handles.thdip_edit.String);
Beta  = str2num(handles.Beta_edit.String);

if(any([(handles.AHRS.Ts ~= Ts); (handles.AHRS.m0 ~= m0); (handles.AHRS.Dip0 ~= Dip0); (handles.AHRS.thdip ~= thdip); (handles.AHRS.Beta ~= Beta)]))
    handles.AHRS.Ts = Ts;
    handles.AHRS.m0 = m0;
    handles.AHRS.Dip0 = Dip0;
    handles.AHRS.thdip = thdip;
    handles.AHRS.Beta = Beta;
    
    guidata(hObject, handles);
    Madgewick_AHRS_Dynamics(hObject, handles)
    handles = guidata(hObject);
    
    PlotAngles(handles);
end

guidata(hObject, handles);


function app_x_edit_Callback(hObject, eventdata, handles)
% hObject    handle to app_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of app_x_edit as text
%        str2double(get(hObject,'String')) returns contents of app_x_edit as a double


% --- Executes during object creation, after setting all properties.
function app_x_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to app_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function app_y_edit_Callback(hObject, eventdata, handles)
% hObject    handle to app_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of app_y_edit as text
%        str2double(get(hObject,'String')) returns contents of app_y_edit as a double


% --- Executes during object creation, after setting all properties.
function app_y_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to app_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function app_z_edit_Callback(hObject, eventdata, handles)
% hObject    handle to app_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of app_z_edit as text
%        str2double(get(hObject,'String')) returns contents of app_z_edit as a double


% --- Executes during object creation, after setting all properties.
function app_z_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to app_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Kapp_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Kapp_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Kapp_edit as text
%        str2double(get(hObject,'String')) returns contents of Kapp_edit as a double


% --- Executes during object creation, after setting all properties.
function Kapp_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Kapp_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function thg_x_edit_Callback(hObject, eventdata, handles)
% hObject    handle to thg_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thg_x_edit as text
%        str2double(get(hObject,'String')) returns contents of thg_x_edit as a double


% --- Executes during object creation, after setting all properties.
function thg_x_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thg_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function thg_y_edit_Callback(hObject, eventdata, handles)
% hObject    handle to thg_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thg_y_edit as text
%        str2double(get(hObject,'String')) returns contents of thg_y_edit as a double


% --- Executes during object creation, after setting all properties.
function thg_y_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thg_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function thg_z_edit_Callback(hObject, eventdata, handles)
% hObject    handle to thg_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thg_z_edit as text
%        str2double(get(hObject,'String')) returns contents of thg_z_edit as a double


% --- Executes during object creation, after setting all properties.
function thg_z_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thg_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function m0_edit_Callback(hObject, eventdata, handles)
% hObject    handle to m0_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of m0_edit as text
%        str2double(get(hObject,'String')) returns contents of m0_edit as a double


% --- Executes during object creation, after setting all properties.
function m0_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to m0_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Dip0_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Dip0_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Dip0_edit as text
%        str2double(get(hObject,'String')) returns contents of Dip0_edit as a double


% --- Executes during object creation, after setting all properties.
function Dip0_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Dip0_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function thdip_edit_Callback(hObject, eventdata, handles)
% hObject    handle to thdip_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of thdip_edit as text
%        str2double(get(hObject,'String')) returns contents of thdip_edit as a double


% --- Executes during object creation, after setting all properties.
function thdip_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thdip_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Beta_edit_Callback(hObject, eventdata, handles)
% hObject    handle to Beta_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Beta_edit as text
%        str2double(get(hObject,'String')) returns contents of Beta_edit as a double


% --- Executes during object creation, after setting all properties.
function Beta_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Beta_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in Acc_btn.
function Acc_btn_Callback(hObject, eventdata, handles)
% hObject    handle to Acc_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Acc_btn


% --- Executes on button press in Gyro_btn.
function Gyro_btn_Callback(hObject, eventdata, handles)
% hObject    handle to Gyro_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Gyro_btn


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% OuterPosition = [X Y Width Height]
%% Column 1
handles.Acc_Axes.Units = 'normalized';
handles.Acc_Axes.OuterPosition = [0 3/4 1/3 1/4];

handles.Gyro_Axes.Units = 'normalized';
handles.Gyro_Axes.OuterPosition = [0 2/4 1/3 1/4];

handles.Switch_Axes.Units = 'normalized';
handles.Switch_Axes.OuterPosition = [0 1/4 1/3 1/4];

handles.Indep_Switch_Axes.Units = 'normalized';
handles.Indep_Switch_Axes.OuterPosition = [0 0 1/3 1/4];
%% Column 2
handles.uipanel1.Units = 'normalized';
handles.uipanel1.OuterPosition = [1/3 3/4 1/3 1/4];

handles.IMU_Axes.Units = 'normalized';
handles.IMU_Axes.OuterPosition = [1/3 2/4 1/3 1/4];

handles.MIMU_Axes.Units = 'normalized';
handles.MIMU_Axes.OuterPosition = [1/3 1/4 1/3 1/4];

handles.MadgeWick_Axes.Units = 'normalized';
handles.MadgeWick_Axes.OuterPosition = [1/3 0 1/3 1/4];

%% Column 3
handles.Magneto_Axes.Units = 'normalized';
handles.Magneto_Axes.OuterPosition = [2/3 2/3 1/3 1/3];

handles.Dip_Amp_Axes.Units = 'normalized';
handles.Dip_Amp_Axes.OuterPosition = [2/3 1/3 1/3 1/3];

handles.Lambda_Axes.Units = 'normalized';
handles.Lambda_Axes.OuterPosition = [2/3 0 1/3 1/3];


function quatern2euler(hObject, eventdata, handles)
q = handles.quatern2euler.q;
R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;

phi = atan2(R(3,2,:), R(3,3,:) );
theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
psi = atan2(R(2,1,:), R(1,1,:) );

euler = [phi(1,:)' theta(1,:)' psi(1,:)'];

handles.quatern2euler.euler = euler;
guidata(hObject, handles)
