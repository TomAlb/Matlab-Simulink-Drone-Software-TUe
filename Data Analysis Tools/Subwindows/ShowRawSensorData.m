function varargout = ShowRAwSensorData(varargin)
% SHOWRAWSENSORDATA MATLAB code for ShowRAwSensorData.fig
%      SHOWRAWSENSORDATA, by itself, creates a new SHOWRAWSENSORDATA or raises the existing
%      singleton*.
%
%      H = SHOWRAWSENSORDATA returns the handle to a new SHOWRAWSENSORDATA or the handle to
%      the existing singleton*.
%
%      SHOWRAWSENSORDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWRAWSENSORDATA.M with the given input arguments.
%
%      SHOWRAWSENSORDATA('Property','Value',...) creates a new SHOWRAWSENSORDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ShowRAwSensorData_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ShowRAwSensorData_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ShowRAwSensorData

% Last Modified by GUIDE v2.5 20-Aug-2018 09:55:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ShowRAwSensorData_OpeningFcn, ...
                   'gui_OutputFcn',  @ShowRAwSensorData_OutputFcn, ...
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


% --- Executes just before ShowRAwSensorData is made visible.
function ShowRAwSensorData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ShowRAwSensorData (see VARARGIN)

% Choose default command line output for ShowRAwSensorData
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(18,1);
set(handles.XRadioBtn, 'value', 1);
set(handles.YRadioBtn, 'value', 1);
set(handles.ZRadioBtn, 'value', 1);
set(handles.OtherRadioBtn, 'value', 1);

set(handles.MagRadioBtn, 'value', 0);
set(handles.AccRadioBtn, 'value', 0);
set(handles.GyroRadioBtn, 'value', 0);
set(handles.BarRadioBtn, 'value', 0);
set(handles.GPSRadioBtn, 'value', 0);
set(handles.BatteryRadioBtn, 'value', 0);

cla(handles.RawSensorDataAxes, 'reset')
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('Raw Sensor Data')
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ShowRAwSensorData wait for user response (see UIRESUME)
% uiwait(handles.ShowRawSensorData);


% --- Outputs from this function are returned to the command line.
function varargout = ShowRAwSensorData_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in XRadioBtn.
function XRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to XRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.MagRadioBtn, 'value') get(handles.AccRadioBtn, 'value') get(handles.GyroRadioBtn, 'value') get(handles.GPSRadioBtn, 'value') get(handles.BatteryRadioBtn, 'value')];
handles.RequestDataFlags([1 4 7 11 15]) = false(5,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([1 4 7 11 15]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of XRadioBtn


% --- Executes on button press in YRadioBtn.
function YRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.MagRadioBtn, 'value') get(handles.AccRadioBtn, 'value') get(handles.GyroRadioBtn, 'value') get(handles.GPSRadioBtn, 'value') get(handles.BatteryRadioBtn, 'value')];
handles.RequestDataFlags([2 5 8 12 16]) = false(5,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([2 5 8 12 16]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of YRadioBtn


% --- Executes on button press in ZRadioBtn.
function ZRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ZRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.MagRadioBtn, 'value') get(handles.AccRadioBtn, 'value') get(handles.GyroRadioBtn, 'value') get(handles.BarRadioBtn, 'value') get(handles.GPSRadioBtn, 'value') get(handles.BatteryRadioBtn, 'value') get(handles.SonarRadioBtn, 'value')];
handles.RequestDataFlags([3 6 9 10 13 17 18]) = false(7,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([3 6 9 10 13 17 18]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ZRadioBtn


% --- Executes on button press in OtherRadioBtn.
function OtherRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to OtherRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = get(handles.GPSRadioBtn, 'value');
handles.RequestDataFlags(14) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(14) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of OtherRadioBtn


% --- Executes on button press in MagRadioBtn.
function MagRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to MagRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(1:3) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(1:3) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of MagRadioBtn


% --- Executes on button press in AccRadioBtn.
function AccRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to AccRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(4:6) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(4:6) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of AccRadioBtn


% --- Executes on button press in GyroRadioBtn.
function GyroRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to GyroRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(7:9) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(7:9) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of GyroRadioBtn



% --- Executes on button press in BarRadioBtn.
function BarRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to BarRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(10) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(10) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of BarRadioBtn


% --- Executes on button press in GPSRadioBtn.
function GPSRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to GPSRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value') get(handles.OtherRadioBtn, 'value')];
handles.RequestDataFlags(11:14) = false(1,4);
if(get(hObject,'Value'))
    handles.RequestDataFlags(11:14) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of GPSRadioBtn


% --- Executes on button press in BatteryRadioBtn.
function BatteryRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to BatteryRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(15:17) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(15:17) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of BatteryRadioBtn
PlotRequestedData(handles)

% --- Executes on button press in SonarRadioBtn.
function SonarRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to SonarRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(18) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(18) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of SonarRadioBtn

% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to BatteryRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Time = handles.MainData.DataSet.CLOCK(5,:);
Data = handles.MainData.DataSet.RAW_SENSOR_DATA(handles.RequestDataFlags, :);
cla(handles.RawSensorDataAxes, 'reset')
LegendString = {'Mag_x', 'Mag_y', 'Mag_z', 'Acc_x', 'Acc_y', 'Acc_z', 'Gyro_\phi', 'Gyro_\theta', 'Gyro_\psi', 'Pressure', 'Latitude', 'Longitude', 'Altitude', 'N Sats', 'Voltage', 'Current', 'mAh', 'Sonar'};
if(any(handles.RequestDataFlags))
    plot(Time, Data)
    legend(LegendString{handles.RequestDataFlags},'location', 'best')
end
    grid on
    axis tight
    xlabel('Time')
    ylabel('Data')
    title('Raw Sensor Data')


% --- Executes when ShowRawSensorData is resized.
function ShowRawSensorData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowRawSensorData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% OuterPosition = [X Y Width Height]
handles.RawSensorDataAxes.Units = 'normalized';
handles.Sensor_Panel.Units      = 'normalized';
handles.Direction_Panel.Units   = 'normalized';

handles.RawSensorDataAxes.OuterPosition = [0 0 0.8 1];
handles.Sensor_Panel.OuterPosition      = [0.8 0.5 0.2 0.5];
handles.Direction_Panel.OuterPosition   = [0.8 0 0.2 0.5];


% --- Executes when Sensor_Panel is resized.
function Sensor_Panel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to Sensor_Panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.MagRadioBtn.Units       = 'normalized';
handles.AccRadioBtn.Units       = 'normalized';
handles.GyroRadioBtn.Units      = 'normalized';
handles.GPSRadioBtn.Units       = 'normalized';
handles.BatteryRadioBtn.Units   = 'normalized';
handles.SonarRadioBtn.Units     = 'normalized';
handles.BarRadioBtn.Units       = 'normalized';

handles.MagRadioBtn.OuterPosition       = [0.00 0.85 1 0.1];
handles.AccRadioBtn.OuterPosition       = [0.00 0.75 1 0.1];
handles.GyroRadioBtn.OuterPosition      = [0.00 0.65 1 0.1];
handles.BarRadioBtn.OuterPosition       = [0.00 0.55 1 0.1];
handles.GPSRadioBtn.OuterPosition       = [0.00 0.45 1 0.1];
handles.BatteryRadioBtn.OuterPosition   = [0.00 0.35 1 0.1];
handles.SonarRadioBtn.OuterPosition     = [0.00 0.25 1 0.1];


% --- Executes when Direction_Panel is resized.
function Direction_Panel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to Direction_Panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.XRadioBtn.Units     = 'normalized';
handles.YRadioBtn.Units     = 'normalized';
handles.ZRadioBtn.Units     = 'normalized';
handles.OtherRadioBtn.Units = 'normalized';

handles.XRadioBtn.OuterPosition     = [0.00 0.85 1 0.1];
handles.YRadioBtn.OuterPosition     = [0.00 0.75 1 0.1];
handles.ZRadioBtn.OuterPosition     = [0.00 0.65 1 0.1];
handles.OtherRadioBtn.OuterPosition = [0.00 0.55 1 0.1];

