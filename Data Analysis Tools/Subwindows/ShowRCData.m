function varargout = ShowRCData(varargin)
% SHOWRCDATA MATLAB code for showrcdata.fig
%      SHOWRCDATA, by itself, creates a new SHOWRCDATA or raises the existing
%      singleton*.
%
%      H = SHOWRCDATA returns the handle to a new SHOWRCDATA or the handle to
%      the existing singleton*.
%
%      SHOWRCDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWRCDATA.M with the given input arguments.
%
%      SHOWRCDATA('Property','Value',...) creates a new SHOWRCDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before showrcdata_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to showrcdata_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help showrcdata

% Last Modified by GUIDE v2.5 20-Aug-2018 14:43:44

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @ShowRCData_OpeningFcn, ...
    'gui_OutputFcn',  @ShowRCData_OutputFcn, ...
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


% --- Executes just before showrcdata is made visible.
function ShowRCData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to showrcdata (see VARARGIN)

% Choose default command line output for showrcdata
handles.output = hObject;

h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(14,1);

set(handles.RollRadioBtn, 'value', 0);
set(handles.PitchRadioBtn, 'value', 0);
set(handles.YawRadioBtn, 'value', 0);
set(handles.ThrustRadioBtn, 'value', 0);
set(handles.ArmRadioBtn, 'value', 0);
set(handles.Con1RadioBtn, 'value', 0);
set(handles.Con2RadioBtn, 'value', 0);
set(handles.OptRadioBtn, 'value', 0);
set(handles.RSSIRadioBtn, 'value', 0);
set(handles.FailSafeRadioBtn, 'value', 0);

set(handles.RawRCDataBtn, 'value', 1);
set(handles.FilRCDataBtn, 'value', 1);

handles.PlotDataTypeFlags = true(2,1);
cla(handles.RCDataAxes, 'reset')
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('RC Data')
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes showrcdata wait for user response (see UIRESUME)
% uiwait(handles.ShowRCData);


% --- Outputs from this function are returned to the command line.
function varargout = ShowRCData_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in RollRadioBtn.
function RollRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RollRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(get(hObject,'Value'))
    handles.RequestDataFlags(1) = handles.RawRCDataBtn.Value;
    handles.RequestDataFlags(2) = handles.FilRCDataBtn.Value;
else
    handles.RequestDataFlags(1) = get(hObject,'Value');
    handles.RequestDataFlags(2) = get(hObject,'Value');
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of RollRadioBtn


% --- Executes on button press in PitchRadioBtn.
function PitchRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PitchRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(get(hObject,'Value'))
    handles.RequestDataFlags(3) = handles.RawRCDataBtn.Value;
    handles.RequestDataFlags(4) = handles.FilRCDataBtn.Value;
else
    handles.RequestDataFlags(3) = get(hObject,'Value');
    handles.RequestDataFlags(4) = get(hObject,'Value');
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PitchRadioBtn


% --- Executes on button press in YawRadioBtn.
function YawRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YawRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(get(hObject,'Value'))
    handles.RequestDataFlags(5) = handles.RawRCDataBtn.Value;
    handles.RequestDataFlags(6) = handles.FilRCDataBtn.Value;
    else
    handles.RequestDataFlags(5) = get(hObject,'Value');
    handles.RequestDataFlags(6) = get(hObject,'Value');
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of YawRadioBtn


% --- Executes on button press in ThrustRadioBtn.
function ThrustRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ThrustRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(get(hObject,'Value'))
    handles.RequestDataFlags(7) = handles.RawRCDataBtn.Value;
    handles.RequestDataFlags(8) = handles.FilRCDataBtn.Value;
    else
    handles.RequestDataFlags(7) = get(hObject,'Value');
    handles.RequestDataFlags(8) = get(hObject,'Value');
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ThrustRadioBtn


% --- Executes on button press in ArmRadioBtn.
function ArmRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ArmRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(9) = get(hObject,'Value');
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ArmRadioBtn


% --- Executes on button press in Con1RadioBtn.
function Con1RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Con1RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(10) = get(hObject,'Value');
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of Con1RadioBtn


% --- Executes on button press in Con2RadioBtn.
function Con2RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Con2RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(11) = get(hObject,'Value');
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of Con2RadioBtn


% --- Executes on button press in OptRadioBtn.
function OptRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to OptRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(12) = get(hObject,'Value');
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of OptRadioBtn


% --- Executes on button press in RSSIRadioBtn.
function RSSIRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RSSIRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(13) = get(hObject,'Value');
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of RSSIRadioBtn


% --- Executes on button press in FailSafeRadioBtn.
function FailSafeRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FailSafeRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(14) = get(hObject,'Value');
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of FailSafeRadioBtn


% --- Executes on button press in RawRCDataBtn.
function RawRCDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RawRCDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(get(hObject,'Value'))
    handles.RequestDataFlags(1) = handles.RollRadioBtn.Value;
    handles.RequestDataFlags(3) = handles.PitchRadioBtn.Value;
    handles.RequestDataFlags(5) = handles.YawRadioBtn.Value;
    handles.RequestDataFlags(7) = handles.ThrustRadioBtn.Value;
else
    handles.RequestDataFlags(1) = get(hObject,'Value');
    handles.RequestDataFlags(3) = get(hObject,'Value');
    handles.RequestDataFlags(5) = get(hObject,'Value');
    handles.RequestDataFlags(7) = get(hObject,'Value');
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of RawRCDataBtn


% --- Executes on button press in FilRCDataBtn.
function FilRCDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FilRCDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if(get(hObject,'Value'))
    handles.RequestDataFlags(2) = handles.RollRadioBtn.Value;
    handles.RequestDataFlags(4) = handles.PitchRadioBtn.Value;
    handles.RequestDataFlags(6) = handles.YawRadioBtn.Value;
    handles.RequestDataFlags(8) = handles.ThrustRadioBtn.Value;
else
    handles.RequestDataFlags(2) = get(hObject,'Value');
    handles.RequestDataFlags(4) = get(hObject,'Value');
    handles.RequestDataFlags(6) = get(hObject,'Value');
    handles.RequestDataFlags(8) = get(hObject,'Value');
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of FilRCDataBtn

% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to ArmRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Time = handles.MainData.DataSet.CLOCK(5,:);
Data = zeros(13,length(Time));
RCIndex = [1; 3; 5; 7; 9; 10; 11; 12; 13; 14];
TARGETIndex = [8; 2; 4; 6];
Data(RCIndex, :) = handles.MainData.DataSet.RC;
Data(TARGETIndex, :) = handles.MainData.DataSet.TARGET(1:4,:);

cla(handles.RCDataAxes, 'reset')
LegendString = {'Roll Raw', 'Roll Fil', 'Pitch Raw', 'Pitch Fil', 'Yaw Raw', 'Yaw Fil', 'Thrust Raw', 'Thrust Fil', 'Arm', 'Config_1', 'Config_2', 'Optional', 'RSSI', 'FailSafe'};
hold(handles.RCDataAxes, 'on')
if(any(handles.RequestDataFlags))
    for i = 1:length(handles.RequestDataFlags)
        if(handles.RequestDataFlags(i,:))
            plot(Time, Data(i,:))
        end
    end
    legend(LegendString{handles.RequestDataFlags},'location', 'best')
end
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('RC Data')


% --- Executes when ShowRawSensorData is resized.
function ShowRawSensorData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowRawSensorData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RCDataAxes.Units        = 'normalized';
handles.RCChannelsPanel.Units   = 'normalized';
handles.RawFilPanel.Units       = 'normalized';

% OuterPosition = [X Y Width Height]
handles.RCDataAxes.OuterPosition        = [0.00 0.00 0.80 1.00];
handles.RCChannelsPanel.OuterPosition   = [0.80 0.50 0.20 0.50];
handles.RawFilPanel.OuterPosition       = [0.80 0.00 0.20 0.50];


% --- Executes when RCChannelsPanel is resized.
function RCChannelsPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to RCChannelsPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RollRadioBtn.Units      = 'normalized';
handles.PitchRadioBtn.Units     = 'normalized';
handles.YawRadioBtn.Units       = 'normalized';
handles.ThrustRadioBtn.Units    = 'normalized';
handles.ArmRadioBtn.Units       = 'normalized';
handles.Con1RadioBtn.Units      = 'normalized';
handles.Con2RadioBtn.Units      = 'normalized';
handles.OptRadioBtn.Units       = 'normalized';
handles.RSSIRadioBtn.Units      = 'normalized';
handles.FailSafeRadioBtn.Units  = 'normalized';

% OuterPosition = [X Y Width Height]
handles.RollRadioBtn.OuterPosition      = [0.00 0.90 1.00 0.10];
handles.PitchRadioBtn.OuterPosition     = [0.00 0.80 1.00 0.10];
handles.YawRadioBtn.OuterPosition       = [0.00 0.70 1.00 0.10];
handles.ThrustRadioBtn.OuterPosition    = [0.00 0.60 1.00 0.10];
handles.ArmRadioBtn.OuterPosition       = [0.00 0.50 1.00 0.10];
handles.Con1RadioBtn.OuterPosition      = [0.00 0.40 1.00 0.10];
handles.Con2RadioBtn.OuterPosition      = [0.00 0.30 1.00 0.10];
handles.OptRadioBtn.OuterPosition       = [0.00 0.20 1.00 0.10];
handles.RSSIRadioBtn.OuterPosition      = [0.00 0.10 1.00 0.10];
handles.FailSafeRadioBtn.OuterPosition  = [0.00 0.00 1.00 0.10];


% --- Executes when RawFilPanel is resized.
function RawFilPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to RawFilPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RawRCDataBtn.Units = 'normalized';
handles.FilRCDataBtn.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.RawRCDataBtn.OuterPosition = [0.00 0.90 1.00 0.10];
handles.FilRCDataBtn.OuterPosition = [0.00 0.80 1.00 0.10];
