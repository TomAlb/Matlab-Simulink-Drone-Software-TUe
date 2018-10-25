function varargout = SetKalman(varargin)
% SETKALMAN MATLAB code for SetKalman.fig
%      SETKALMAN, by itself, creates a new SETKALMAN or raises the existing
%      singleton*.
%
%      H = SETKALMAN returns the handle to a new SETKALMAN or the handle to
%      the existing singleton*.
%
%      SETKALMAN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETKALMAN.M with the given input arguments.
%
%      SETKALMAN('Property','Value',...) creates a new SETKALMAN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SetKalman_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SetKalman_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SetKalman

% Last Modified by GUIDE v2.5 19-Sep-2018 09:41:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SetKalman_OpeningFcn, ...
                   'gui_OutputFcn',  @SetKalman_OutputFcn, ...
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


% --- Executes just before SetKalman is made visible.
function SetKalman_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SetKalman (see VARARGIN)

% Choose default command line output for SetKalman
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.Acc_Btn.Value = true;
handles.GPS_Btn.Value = true;
handles.Baro_Btn.Value = true;
handles.Sonar_Btn.Value = true;

PlotData(hObject, eventdata, handles);
handles = guidata(hObject);

% PlotAngles(handles)

% UIWAIT makes SetKalman wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function PlotData(hObject, eventdata, handles)
% Load needed data
handles.Data.CLOCK_DATA     = handles.MainData.DataSet.CLOCK(end,:);
handles.Data.ACC_DATA       = handles.MainData.DataSet.FILTERED_SENSOR_DATA(4:6,:);
handles.Data.BAR            = handles.MainData.DataSet.FILTERED_SENSOR_DATA(10,:);
handles.Data.SONAR          = handles.MainData.DataSet.FILTERED_SENSOR_DATA(18,:);
handles.Data.GPS_DATA       = handles.MainData.DataSet.FILTERED_SENSOR_DATA(11:13,:);
handles.Data.ANGLES         = handles.MainData.DataSet.STATE(4:6,:);

%% Plot main data
% Plot Position data
cla(handles.GPS_Axes, 'reset');
hold(handles.GPS_Axes, 'on');

plot(handles.GPS_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(1,:), 'DisplayName', 'X_{GPS}')
plot(handles.GPS_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(2,:), 'DisplayName', 'Y_{GPS}')
plot(handles.GPS_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(3,:), 'DisplayName', 'Z_{GPS}')
plot(handles.GPS_Axes, handles.Data.CLOCK_DATA, handles.Data.BAR, 'DisplayName', 'Z_{Bar}')
plot(handles.GPS_Axes, handles.Data.CLOCK_DATA, handles.Data.SONAR, 'DisplayName', 'Z_{Sonar}')

grid(handles.GPS_Axes, 'on');
axis(handles.GPS_Axes, 'tight');
legend(handles.GPS_Axes, 'show', 'location', 'best');
title(handles.GPS_Axes, 'Position-data');
xlabel(handles.GPS_Axes, 'Time');
ylabel(handles.GPS_Axes, 'Position [m]');

% Plot accelerations
cla(handles.Acc_Axes, 'reset');
hold(handles.Acc_Axes, 'on');

plot(handles.Acc_Axes, handles.Data.CLOCK_DATA, handles.Data.ACC_DATA(1,:), 'DisplayName', 'X')
plot(handles.Acc_Axes, handles.Data.CLOCK_DATA, handles.Data.ACC_DATA(2,:), 'DisplayName', 'Y')
plot(handles.Acc_Axes, handles.Data.CLOCK_DATA, handles.Data.ACC_DATA(3,:), 'DisplayName', 'Z')

grid(handles.Acc_Axes, 'on');
axis(handles.Acc_Axes, 'tight');
legend(handles.Acc_Axes, 'show', 'location', 'best');
title(handles.Acc_Axes, 'Accelerometer-data');
xlabel(handles.Acc_Axes, 'Time');
ylabel(handles.Acc_Axes, 'acc [m/s^2]');

% Empty Kalman results
cla(handles.KM_Position_Axes, 'reset');
grid(handles.KM_Position_Axes, 'on');
axis(handles.KM_Position_Axes, 'tight');
title(handles.KM_Position_Axes, 'Position');
xlabel(handles.KM_Position_Axes, 'Time');
ylabel(handles.KM_Position_Axes, 'p [m]');

cla(handles.KM_Velocity_Axes, 'reset');
grid(handles.KM_Velocity_Axes, 'on');
axis(handles.KM_Velocity_Axes, 'tight');
title(handles.KM_Velocity_Axes, 'Velocity');
xlabel(handles.KM_Velocity_Axes, 'Time');
ylabel(handles.KM_Velocity_Axes, 'v [m/s]');

cla(handles.KM_Acc_Axes, 'reset');
grid(handles.KM_Acc_Axes, 'on');
axis(handles.KM_Acc_Axes, 'tight');
title(handles.KM_Acc_Axes, 'Acceleration');
xlabel(handles.KM_Acc_Axes, 'Time');
ylabel(handles.KM_Acc_Axes, 'a [m/s^2]');

linkaxes([handles.GPS_Axes handles.Acc_Axes handles.KM_Position_Axes handles.KM_Velocity_Axes handles.KM_Acc_Axes handles.Pos_Axes],'x')

for i = 1:length(handles.Data.CLOCK_DATA)
    phi = handles.Data.ANGLES(1,i);
    theta = handles.Data.ANGLES(2,i);
    psi = handles.Data.ANGLES(3,i);
    
    Rx = [1 0        0;...
          0 cos(phi) -sin(phi);...
          0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta);...
          0 1 0;...
          -sin(theta) 0 cos(theta)];
  
    Rz = [cos(psi) -sin(psi) 0;...
          sin(psi) cos(psi) 0;...
          0 0 1];
    R = Rz * Ry * Rx;
    
    handles.Data.KM_ACC(:,i) = R*handles.Data.ACC_DATA(:,i) - [0; 0; 9.81748];
end

% Plot KM Accelerations
cla(handles.KM_Acc_Axes, 'reset');
hold(handles.KM_Acc_Axes, 'on');

plot(handles.KM_Acc_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_ACC(1,:), 'DisplayName', 'X')
plot(handles.KM_Acc_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_ACC(2,:), 'DisplayName', 'Y')
plot(handles.KM_Acc_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_ACC(3,:), 'DisplayName', 'Z')

grid(handles.KM_Acc_Axes, 'on');
axis(handles.KM_Acc_Axes, 'tight');
legend(handles.KM_Acc_Axes, 'show', 'location', 'best');
title(handles.KM_Acc_Axes, 'Accelerometer-data');
xlabel(handles.KM_Acc_Axes, 'Time');
ylabel(handles.KM_Acc_Axes, 'acc [m/s^2]');

% Plot Position results VS Kalman
cla(handles.Pos_Axes, 'reset');
hold(handles.Pos_Axes, 'on');

plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(1,:), 'DisplayName', 'X_{GPS}')
plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(2,:), 'DisplayName', 'Y_{GPS}')
plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(3,:), 'DisplayName', 'Z_{GPS}')
plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.BAR, 'DisplayName', 'Z_{BAR}')
plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.SONAR, 'DisplayName', 'Z_{SONAR}')

grid(handles.Pos_Axes, 'on');
axis(handles.Pos_Axes, 'tight');
legend(handles.Pos_Axes, 'show', 'location', 'eastoutside');
title(handles.Pos_Axes, 'Position-data');
xlabel(handles.Pos_Axes, 'Time');
ylabel(handles.Pos_Axes, 'GPS');

guidata(hObject, handles)


% --- Outputs from this function are returned to the command line.
function varargout = SetKalman_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

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

Use_Acc = handles.Acc_Btn.Value;
Use_GPS = handles.GPS_Btn.Value;
Use_Baro = handles.Baro_Btn.Value;
Use_Sonar = handles.Sonar_Btn.Value;

%% Static
mn_GPS = diag([str2num(handles.mn_GPS_x_edit.String); str2num(handles.mn_GPS_y_edit.String); str2num(handles.mn_GPS_z_edit.String)]);
mn_Baro = str2num(handles.mn_BARO_z_edit.String);
mn_Sonar = str2num(handles.mn_SONAR_z_edit.String);

if(Use_Acc)
    pn = [str2num(handles.pn_x_edit.String); str2num(handles.pn_y_edit.String); str2num(handles.pn_z_edit.String)];
else
    pn = ones(3,1);
end
Ts = str2num(handles.Ts_edit.String);

%% Set Model
A = [diag([1 1 1]) Ts*diag([1 1 1]); diag([0 0 0]) diag([1 1 1])];
B = [Ts^2/2*diag([1 1 1]); Ts*diag([1 1 1])];

C_GPS   = [diag([1 1 1]) diag([0 0 0])];
C_Baro  = [0 0 1 0 0 0];
C_Sonar = [0 0 1 0 0 0];

C = [];
mn = [];
if(Use_GPS)
    C = [C; C_GPS];
    mn = blkdiag(mn, mn_GPS);
end
if(Use_Baro)
    C = [C; C_Baro];
    mn = blkdiag(mn, mn_Baro);
end
if(Use_Sonar)
    C = [C; C_Sonar];
    mn = blkdiag(mn, mn_Sonar);
end

q = [Ts^4/4 Ts^3/2; Ts^3/2 Ts^2];
Q = diag([pn.^2; pn.^2])*(B*B');
P = Q;

R = mn.^2;
Xhat = zeros(size(A,1),1);

% Run Kalman Filter
for i = 1:length(handles.Data.CLOCK_DATA)
    % Prediction
    if(Use_Acc)
        Xhat = A*Xhat + B*handles.Data.KM_ACC(:,i);
    else
        Xhat = A*Xhat + B*[0; 0; 0];
    end
    P = A * P * A' + Q;
    
    % Get Kalman Gain
    K = P * C' * inv(C * P * C' + R);
    
    % Measurement Data
    Measurement = [];
    if(Use_GPS)
        Measurement = [Measurement; handles.Data.GPS_DATA(:,i)];
    end
    if(Use_Baro)
        Measurement = [Measurement; handles.Data.BAR(:,i)];
    end
    if(Use_Sonar)
        Measurement = [Measurement; handles.Data.SONAR(:,i)];
    end
    
    % Update
    Xhat = Xhat + K * (Measurement - C * Xhat);
    P = (eye(size(K * C)) - K * C) * P;
    
    handles.Data.KM_POS(:,i) = Xhat(1:3,:);
    handles.Data.KM_VEL(:,i) = Xhat(4:6,:);
end

cla(handles.KM_Position_Axes, 'reset');
hold(handles.KM_Position_Axes, 'on');
for i = 1:size(handles.Data.GPS_DATA, 1)
    plot(handles.KM_Position_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_POS(i,:))
end
grid(handles.KM_Position_Axes, 'on');
axis(handles.KM_Position_Axes, 'tight');
legend(handles.KM_Position_Axes, 'X', 'Y', 'Z', 'location', 'best');
title(handles.KM_Position_Axes, 'KM-filter');
xlabel(handles.KM_Position_Axes, 'Time');
ylabel(handles.KM_Position_Axes, 'KM');

cla(handles.KM_Velocity_Axes, 'reset');
hold(handles.KM_Velocity_Axes, 'on');
for i = 1:size(handles.Data.GPS_DATA, 1)
    plot(handles.KM_Velocity_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_VEL(i,:))
end
grid(handles.KM_Velocity_Axes, 'on');
axis(handles.KM_Velocity_Axes, 'tight');
legend(handles.KM_Velocity_Axes, 'X', 'Y', 'Z', 'location', 'best');
title(handles.KM_Velocity_Axes, 'KM-filter');
xlabel(handles.KM_Velocity_Axes, 'Time');
ylabel(handles.KM_Velocity_Axes, 'KM');

cla(handles.Pos_Axes, 'reset');
hold(handles.Pos_Axes, 'on');

if(Use_GPS)
    plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(1,:), 'DisplayName', 'X_{GPS}')
    plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(2,:), 'DisplayName', 'Y_{GPS}')
    plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.GPS_DATA(3,:), 'DisplayName', 'Z_{GPS}')
end

if(Use_Baro)
    plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.BAR, 'DisplayName', 'Z_{BAR}')
end
if(Use_Sonar)
    plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.SONAR, 'DisplayName', 'Z_{SONAR}')
end

plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_POS(1,:), '--', 'linewidth', 1, 'DisplayName', 'X_{KM}')
plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_POS(2,:), '--', 'linewidth', 1, 'DisplayName', 'Y_{KM}')
plot(handles.Pos_Axes, handles.Data.CLOCK_DATA, handles.Data.KM_POS(3,:), '--', 'linewidth', 1, 'DisplayName', 'Z_{KM}')
grid(handles.Pos_Axes, 'on');
axis(handles.Pos_Axes, 'tight');
legend(handles.Pos_Axes, 'show', 'location', 'eastoutside');
title(handles.Pos_Axes, 'Position-data');
xlabel(handles.Pos_Axes, 'Time [s]');
ylabel(handles.Pos_Axes, 'Position [m]');

guidata(hObject, handles);


function mn_GPS_x_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mn_GPS_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mn_GPS_x_edit as text
%        str2double(get(hObject,'String')) returns contents of mn_GPS_x_edit as a double


% --- Executes during object creation, after setting all properties.
function mn_GPS_x_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mn_GPS_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mn_GPS_y_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mn_GPS_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mn_GPS_y_edit as text
%        str2double(get(hObject,'String')) returns contents of mn_GPS_y_edit as a double


% --- Executes during object creation, after setting all properties.
function mn_GPS_y_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mn_GPS_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mn_GPS_z_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mn_GPS_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mn_GPS_z_edit as text
%        str2double(get(hObject,'String')) returns contents of mn_GPS_z_edit as a double


% --- Executes during object creation, after setting all properties.
function mn_GPS_z_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mn_GPS_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function pn_x_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pn_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pn_x_edit as text
%        str2double(get(hObject,'String')) returns contents of pn_x_edit as a double


% --- Executes during object creation, after setting all properties.
function pn_x_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pn_x_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pn_y_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pn_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pn_y_edit as text
%        str2double(get(hObject,'String')) returns contents of pn_y_edit as a double


% --- Executes during object creation, after setting all properties.
function pn_y_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pn_y_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pn_z_edit_Callback(hObject, eventdata, handles)
% hObject    handle to pn_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pn_z_edit as text
%        str2double(get(hObject,'String')) returns contents of pn_z_edit as a double


% --- Executes during object creation, after setting all properties.
function pn_z_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pn_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Column 1
handles.GPS_Axes.Units = 'normalized';
handles.Acc_Axes.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.GPS_Axes.OuterPosition = [0 1/2 1/3 1/2];
handles.Acc_Axes.OuterPosition = [0 0/2 1/3 1/2];

%% Column 2
handles.KM_Position_Axes.Units  = 'normalized';
handles.KM_Velocity_Axes.Units  = 'normalized';
handles.KM_Acc_Axes.Units       = 'normalized';

% OuterPosition = [X Y Width Height]
handles.KM_Position_Axes.OuterPosition  = [1/3 2/3 1/3 1/4];
handles.KM_Velocity_Axes.OuterPosition  = [1/3 1/3 1/3 1/4];
handles.KM_Acc_Axes.OuterPosition       = [1/3 0/3 1/3 1/4];

%% Column 3
handles.SettingsPanel.Units = 'normalized';
handles.Pos_Axes.Units      = 'normalized';

% OuterPosition = [X Y Width Height]
handles.SettingsPanel.OuterPosition = [2/3 1/2 1/3 1/2];
handles.Pos_Axes.OuterPosition      = [2/3 0/2 1/3 1/2];


% --- Executes when SettingsPanel is resized.
function SettingsPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to SettingsPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.pn_txt.Units            = 'normalized';
handles.pn_x_edit.Units         = 'normalized';
handles.pn_y_edit.Units         = 'normalized';
handles.pn_z_edit.Units         = 'normalized';

handles.mn_GPS_txt.Units        = 'normalized';
handles.mn_GPS_x_edit.Units     = 'normalized';
handles.mn_GPS_y_edit.Units     = 'normalized';
handles.mn_GPS_z_edit.Units     = 'normalized';

handles.mn_BARO_txt.Units       = 'normalized';
handles.mn_BARO_z_edit.Units    = 'normalized';

handles.mn_SONAR_txt.Units      = 'normalized';
handles.mn_SONAR_z_edit.Units   = 'normalized';

handles.Ts_txt.Units            = 'normalized';
handles.Ts_edit.Units           = 'normalized';
handles.Set_Btn.Units           = 'normalized';

handles.Acc_Btn.Units           = 'normalized';
handles.GPS_Btn.Units           = 'normalized';
handles.Baro_Btn.Units          = 'normalized';
handles.Sonar_Btn.Units         = 'normalized';

% OuterPosition = [X Y Width Height]
handles.pn_txt.OuterPosition            = [0.00 0.95 1.00 0.05];
handles.pn_x_edit.OuterPosition         = [0.00 0.90 0.25 0.05];
handles.pn_y_edit.OuterPosition         = [0.25 0.90 0.25 0.05];
handles.pn_z_edit.OuterPosition         = [0.50 0.90 0.25 0.05];

handles.mn_GPS_txt.OuterPosition        = [0.00 0.85 1.00 0.05];
handles.mn_GPS_x_edit.OuterPosition     = [0.00 0.80 0.25 0.05];
handles.mn_GPS_y_edit.OuterPosition     = [0.25 0.80 0.25 0.05];
handles.mn_GPS_z_edit.OuterPosition     = [0.50 0.80 0.25 0.05];

handles.mn_BARO_txt.OuterPosition       = [0.00 0.75 1.00 0.05];
handles.mn_BARO_z_edit.OuterPosition    = [0.00 0.70 0.25 0.05];

handles.mn_SONAR_txt.OuterPosition      = [0.00 0.65 1.00 0.05];
handles.mn_SONAR_z_edit.OuterPosition   = [0.00 0.60 0.25 0.05];

handles.Ts_txt.OuterPosition            = [0.00 0.55 1.00 0.05];
handles.Ts_edit.OuterPosition           = [0.00 0.50 0.25 0.05];
handles.Set_Btn.OuterPosition           = [0.00 0.45 0.25 0.05];

handles.Acc_Btn.OuterPosition           = [0.00 0.40 1.00 0.05];
handles.GPS_Btn.OuterPosition           = [0.00 0.35 1.00 0.05];
handles.Baro_Btn.OuterPosition          = [0.00 0.30 1.00 0.05];
handles.Sonar_Btn.OuterPosition         = [0.00 0.25 1.00 0.05];



function mn_BARO_z_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mn_BARO_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mn_BARO_z_edit as text
%        str2double(get(hObject,'String')) returns contents of mn_BARO_z_edit as a double


% --- Executes during object creation, after setting all properties.
function mn_BARO_z_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mn_BARO_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mn_SONAR_z_edit_Callback(hObject, eventdata, handles)
% hObject    handle to mn_SONAR_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mn_SONAR_z_edit as text
%        str2double(get(hObject,'String')) returns contents of mn_SONAR_z_edit as a double


% --- Executes during object creation, after setting all properties.
function mn_SONAR_z_edit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mn_SONAR_z_edit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Acc_Btn.
function Acc_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to Acc_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Acc_Btn


% --- Executes on button press in GPS_Btn.
function GPS_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to GPS_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of GPS_Btn


% --- Executes on button press in Baro_Btn.
function Baro_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to Baro_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Baro_Btn


% --- Executes on button press in Sonar_Btn.
function Sonar_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to Sonar_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Sonar_Btn
