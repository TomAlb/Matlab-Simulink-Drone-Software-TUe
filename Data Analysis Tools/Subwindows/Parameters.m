function varargout = Parameters(varargin)
% PARAMETERS MATLAB code for Parameters.fig
%      PARAMETERS, by itself, creates a new PARAMETERS or raises the existing
%      singleton*.
%
%      H = PARAMETERS returns the handle to a new PARAMETERS or the handle to
%      the existing singleton*.
%
%      PARAMETERS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PARAMETERS.M with the given input arguments.
%
%      PARAMETERS('Property','Value',...) creates a new PARAMETERS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Parameters_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Parameters_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Parameters

% Last Modified by GUIDE v2.5 27-Jun-2018 10:36:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Parameters_OpeningFcn, ...
    'gui_OutputFcn',  @Parameters_OutputFcn, ...
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


% --- Executes just before Parameters is made visible.
function Parameters_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Parameters (see VARARGIN)

% Choose default command line output for Parameters
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end
CreateDataTable(handles)

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Parameters wait for user response (see UIRESUME)
% uiwait(handles.ParameterWindow);


% --- Outputs from this function are returned to the command line.
function varargout = Parameters_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function CreateDataTable(handles)
PARAM = handles.MainData.Params;
ls = 0;
dl = 6;
% General constants
LabelParams.General = {'Ts', 'MA Window', 'Filter N', 'Jxx', 'Jyy', 'Jzz'};
Data(1:6,1) = LabelParams.General;
Data(1:6,2) = num2cell(PARAM(ls+1:dl)); ls = ls + dl;

% Set Sensor Settings
LabelParams.Sensor = {'Scale Mag', 'Scale Acc', 'Scale Gyro', 'Scale GPS', 'Scale Battery'};
LabelParams.Label = {'X','Y','Z','OFFSET','Current Cor'};
% Magnetometer
dl = 15; i = 1;
Data{1,1+(i-1)*5+4} = LabelParams.Sensor{i};
Data(1,2+(i-1)*5+4:4+(i-1)*5+4) = LabelParams.Label(1:3);
Data(2:6,1+(i-1)*5+4) = LabelParams.Label;
Data(2:6,2+(i-1)*5+4:4+(i-1)*5+4) = num2cell(reshape(PARAM(ls+1+(i-1)*12:dl+ls+(i-1)*12),[3,5])'); ls = ls + dl;

% Sensors (Accelerometer Gyroscope)
dl = 12;
for i = 2:3 %length(LabelParams.Sensor)
    Data{1,1+(i-1)*5+4} = LabelParams.Sensor{i};
    Data(1,2+(i-1)*5+4:4+(i-1)*5+4) = LabelParams.Label(1:3);
    Data(2:5,1+(i-1)*5+4) = LabelParams.Label(1:4);
    Data(2:5,2+(i-1)*5+4:4+(i-1)*5+4) = num2cell(reshape(PARAM(ls+1:ls+dl),[3,4])'); ls = ls + dl;
end

% Barometer
dl = 2;
Data{8,17} = 'Barometer';
Data{9,15} = 'SCALE';
Data{10,15} = 'OFFSET';
Data(9:10,17) = num2cell(PARAM(ls+1:ls+dl));
ls = ls + dl;

% Other Sensors (Accelerometer, Gyroscope, GPS, Battery)
dl = 12;
for i = 4:length(LabelParams.Sensor)
    Data{8,1+(i-4)*5+4} = LabelParams.Sensor{i};
    Data(8,2+(i-4)*5+4:4+(i-4)*5+4) = LabelParams.Label(1:3);
    Data(9:12,1+(i-4)*5+4) = LabelParams.Label(1:4);
    Data(9:12,2+(i-4)*5+4:4+(i-4)*5+4) = num2cell(reshape(PARAM(ls+1:ls+dl),[3,4])'); ls = ls + dl;
end

% Sonar
dl = 2;
Data{8,16} = 'Sonar';
Data{9,15} = 'SCALE';
Data{10,15} = 'OFFSET';
Data(9:10,16) = num2cell(PARAM(ls+1:ls+dl));
ls = ls + dl;

% Set Filters
dl = 4;
LabelParams.Filter = {'LP Ref'};
LabelParams.ND = {'Numerator','Denominator','Cut off'};
Data(14,2) = LabelParams.Filter;
Data(15:17,1) = LabelParams.ND;
Data(15:16,2:3) = num2cell(reshape(PARAM(ls+1:ls+dl), [2,2])'); 
Filt = reshape(PARAM(ls+1:ls+dl), [4,size(ls+1:ls+dl, 2)/4])'; ls = ls + dl;

for i = 1:size(Filt,1)

    if(Filt(i,3:4) ~= 0)
        Filters(i) = d2c(tf(Filt(i,1:2), Filt(i,3:4), PARAM(1)));
        
        if(Filters(i).num{1}(1) == 0) % Low Pass Filter
            Fc(i) = (Filters(i).num{1}(2) + Filters(i).den{1}(2))/2;
            
        elseif(Filters(i).num{1}(1) == 1) % High Pass Filter
            Fc(i) = Filters(i).den{1}(2);
            
        else
            Fc(i) = 0;
        end
    else
        Fc(i) = 0;
    end
end
Data(17,3) = num2cell(Fc);

% Madgewick Filter
dl = 11;
LabelParams.MWfil = {'app','Kapp','thg','m0','Dip0','thdip','Beta'};
LabelParams.MWfiltitle = {'Madgewick','Filter'};
Data(14,8:9) = LabelParams.MWfiltitle;
Data(15,9:15) = LabelParams.MWfil;
Data{16,8} = 'Values';
Data(16:18,9) = num2cell(PARAM(ls+1:ls+3));
Data(16,10) = num2cell(PARAM(ls+4:ls+4));
Data(16:18,11) = num2cell(PARAM(ls+5:ls+7));
Data(16,12:15) = num2cell(PARAM(ls+8:ls+11));
ls = ls + dl;

% Kalman Filter
dl = 149;
Data(36,1:2) = {'Kalman', 'Filter'};
Data(37,1) = {'A'};
Data(38:43,1:6) = num2cell(reshape(PARAM(ls+1:ls+36), [6,6]));

Data(37,8) = {'B'};
Data(38:43,8:10) = num2cell(reshape(PARAM(ls+37:ls+54), [6,3]));

Data(44,1) = {'C'};
Data(45:48,1:6) = num2cell(reshape(PARAM(ls+55:ls+78), [4,6]));

Data(44,8) = {'D'};
Data(45:48,8:10) = num2cell(reshape(PARAM(ls+79:ls+90), [4,3]));

Data(37,12) = {'Q'};
Data(38:43,12:17) = num2cell(reshape(PARAM(ls+91:ls+126), [6,6]));

Data(44,12) = {'R'};
Data(45:48,12:15) = num2cell(reshape(PARAM(ls+127:ls+142), [4,4]));

Data(30:31,15) = {'Process','noise'};
Data(32:34,15) = num2cell(PARAM(ls+143:ls+145));

Data(30:31,16) = {'Measurement','noise'};
Data(32:35,16) = num2cell(PARAM(ls+146:ls+149));
ls = ls + dl;

% Control Gains
% Rotational states
dl = 12;
Data{21,3} = 'Control gains';
LabelParams.CG = {'P','I','D','FF'};
LabelParams.CGS = {'Roll','Pitch','Yaw','X','Y','Z'};
Data(22,2:5) = LabelParams.CG;
Data(23:28,1) = LabelParams.CGS;
Data(23:25,2:5) = num2cell(reshape(PARAM(ls+1:ls+dl), [3,4])); ls = ls + dl;
% translational states
dl = 12;
Data(26:28,2:5) = num2cell(reshape(PARAM(ls+1:ls+dl), [3,4])); ls = ls + dl;

% RC Settings
dl = 16;
LabelParams.RC = {'RC 1', 'RC 2', 'RC 3', 'RC 4'};
LabelParams.RCSET = {'MIN', 'MAX', 'TRIM', 'DB'};
Data(22,8:11) = LabelParams.RC;
Data(23:26,7) = LabelParams.RCSET;
Data(23:26,8:11) = num2cell(reshape([PARAM(ls+1:ls+dl)], [4,4])); ls = ls + dl;

% Roll Pitch Yaw settings
dl = 6;
LabelParams.RPY = {'Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z'};
Data(23:28,13) = LabelParams.RPY;
Data{22,14} = 'Limit';
Data(23:28,14) = num2cell(PARAM(ls+1:ls+dl)); ls = ls + dl;

% % Pwm 2 Thrust fit constants
dl = 4;
LabelParams.PWMC = {'A', 'B', 'C', 'D'};
Data(23:26,16) = LabelParams.PWMC;
Data{22,17} = 'Value';
Data(23:26,17) = num2cell(PARAM(ls+1:ls+dl))'; ls = ls + dl;

% Motor Mixing
dl = 16;
LabelParams.Motors = {'M1','M2','M3','M4'};
LabelParams.MotorsMixing = {'Thrust','Roll','Pitch','Yaw'};
Data(31:34,1) = LabelParams.MotorsMixing;
Data(30,2:5) = LabelParams.Motors;
Data(31:34,2:5) = num2cell(reshape(PARAM(ls+1:ls+dl), [4,4])'); ls = ls + dl;

% % State constants
dl = 18;
LabelParams.State = {'Position','Angle','Velocity','Angular velocity','Acceleration','Angular Acceleration'};
LabelParams.StateDir = {'X','Y','Z'};
Data(30,8:13) = LabelParams.State;
Data(31:33,7) = LabelParams.StateDir;
Data(31:33,8:13) = num2cell(reshape(PARAM(ls+1:ls+dl),[3,6])); ls = ls + dl;

% Output Data
handles.DataFrame = Data;
set(handles.ParamTable, 'Data', handles.DataFrame);


% --- Executes when ParameterWindow is resized.
function ParameterWindow_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ParameterWindow (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% OuterPosition = [X Y Width Height]
handles.ParamTable.Units = 'normalized';
handles.ParamTable.OuterPosition = [0 0 1 1];
