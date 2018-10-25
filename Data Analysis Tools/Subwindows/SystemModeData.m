function varargout = SystemModeData(varargin)
% SYSTEMMODEDATA MATLAB code for systemmodedata.fig
%      SYSTEMMODEDATA, by itself, creates a new SYSTEMMODEDATA or raises the existing
%      singleton*.
%
%      H = SYSTEMMODEDATA returns the handle to a new SYSTEMMODEDATA or the handle to
%      the existing singleton*.
%
%      SYSTEMMODEDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SYSTEMMODEDATA.M with the given input arguments.
%
%      SYSTEMMODEDATA('Property','Value',...) creates a new SYSTEMMODEDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before systemmodedata_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to systemmodedata_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help systemmodedata

% Last Modified by GUIDE v2.5 26-Jun-2018 12:23:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SystemModeData_OpeningFcn, ...
                   'gui_OutputFcn',  @SystemModeData_OutputFcn, ...
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


% --- Executes just before systemmodedata is made visible.
function SystemModeData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to systemmodedata (see VARARGIN)

% Choose default command line output for systemmodedata
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(11,1);

set(handles.Arm_RadioBtn, 'value', 0);
set(handles.Pre_Arm_RadioBtn, 'value', 0);
set(handles.Config_RadioBtn, 'value', 0);
set(handles.Man_Flight_RadioBtn, 'value', 0);
set(handles.Semi_Auto_Flight_RadioBtn, 'value', 0);
set(handles.Auto_Flight_RadioBtn, 'value', 0);
set(handles.Recording_RadioBtn, 'value', 0);
set(handles.C1_RadioBtn, 'value', 0);
set(handles.C2_RadioBtn, 'value', 0);
set(handles.C3_RadioBtn, 'value', 0);
set(handles.C4_RadioBtn, 'value', 0);

cla(handles.SystemModeDataAxes, 'reset')
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('System Modes')
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes systemmodedata wait for user response (see UIRESUME)
% uiwait(handles.SystemModeData);


% --- Outputs from this function are returned to the command line.
function varargout = SystemModeData_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in Arm_RadioBtn.
function Arm_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Arm_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(1,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of Arm_RadioBtn


% --- Executes on button press in Pre_Arm_RadioBtn.
function Pre_Arm_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Pre_Arm_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(2,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of Pre_Arm_RadioBtn


% --- Executes on button press in Config_RadioBtn.
function Config_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Config_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(3,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of Config_RadioBtn


% --- Executes on button press in Man_Flight_RadioBtn.
function Man_Flight_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Man_Flight_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(4,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of Man_Flight_RadioBtn


% --- Executes on button press in Semi_Auto_Flight_RadioBtn.
function Semi_Auto_Flight_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Semi_Auto_Flight_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(5,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of Semi_Auto_Flight_RadioBtn


% --- Executes on button press in Auto_Flight_RadioBtn.
function Auto_Flight_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Auto_Flight_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(6,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of Auto_Flight_RadioBtn


% --- Executes on button press in Recording_RadioBtn.
function Recording_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to Recording_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(7,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of Recording_RadioBtn


% --- Executes on button press in C1_RadioBtn.
function C1_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to C1_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(8,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of C1_RadioBtn


% --- Executes on button press in C2_RadioBtn.
function C2_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to C2_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(9,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of C2_RadioBtn


% --- Executes on button press in C3_RadioBtn.
function C3_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to C3_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(10,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of C3_RadioBtn


% --- Executes on button press in C4_RadioBtn.
function C4_RadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to C4_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(11,:) = (get(hObject,'Value'));
guidata(hObject, handles);
PlotRequestedData(handles);
% Hint: get(hObject,'Value') returns toggle state of C4_RadioBtn


% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to Semi_Auto_Flight_RadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Time = handles.MainData.DataSet.CLOCK(5,:);
Data = handles.MainData.DataSet.SYSTEM_MODE(handles.RequestDataFlags,:);
cla(handles.SystemModeDataAxes, 'reset')
LegendString = {'ARM', 'Pre Arm', 'Config', 'Man', 'Semi-Auto', 'Auto', 'Record', 'C_1', 'C_2', 'C_3', 'C_4'};
if(any(handles.RequestDataFlags))
    plot(Time, Data, '.')
    legend(LegendString{handles.RequestDataFlags},'location', 'best')
end
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('System Modes')


% --- Executes when SystemModeData is resized.
function SystemModeData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to SystemModeData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.SystemModeDataAxes.Units    = 'normalized';
handles.System_Mode_Panel.Units     = 'normalized';

% OuterPosition = [X Y Width Height]
handles.SystemModeDataAxes.OuterPosition    = [0.00 0.00 0.80 1.00];
handles.System_Mode_Panel.OuterPosition     = [0.80 0.00 0.20 1.00];


% --- Executes when System_Mode_Panel is resized.
% function SystemModeData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to System_Mode_Panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes when System_Mode_Panel is resized.
function System_Mode_Panel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to System_Mode_Panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Arm_RadioBtn.Units              = 'normalized';
handles.Pre_Arm_RadioBtn.Units          = 'normalized';
handles.Config_RadioBtn.Units           = 'normalized';
handles.Man_Flight_RadioBtn.Units       = 'normalized';
handles.Semi_Auto_Flight_RadioBtn.Units = 'normalized';
handles.Auto_Flight_RadioBtn.Units      = 'normalized';
handles.Recording_RadioBtn.Units        = 'normalized';
handles.C1_RadioBtn.Units               = 'normalized';
handles.C2_RadioBtn.Units               = 'normalized';
handles.C3_RadioBtn.Units               = 'normalized';
handles.C4_RadioBtn.Units               = 'normalized';

% OuterPosition = [X Y Width Height]
handles.Arm_RadioBtn.OuterPosition              = [0.00 10/11 1.00 0.10];
handles.Pre_Arm_RadioBtn.OuterPosition          = [0.00 9/11  1.00 0.10];
handles.Config_RadioBtn.OuterPosition           = [0.00 8/11  1.00 0.10];
handles.Man_Flight_RadioBtn.OuterPosition       = [0.00 7/11  1.00 0.10];
handles.Semi_Auto_Flight_RadioBtn.OuterPosition = [0.00 6/11  1.00 0.10];
handles.Auto_Flight_RadioBtn.OuterPosition      = [0.00 5/11  1.00 0.10];
handles.Recording_RadioBtn.OuterPosition        = [0.00 4/11  1.00 0.10];
handles.C1_RadioBtn.OuterPosition               = [0.00 3/11  1.00 0.10];
handles.C2_RadioBtn.OuterPosition               = [0.00 2/11  1.00 0.10];
handles.C3_RadioBtn.OuterPosition               = [0.00 1/11  1.00 0.10];
handles.C4_RadioBtn.OuterPosition               = [0.00 0/11  1.00 0.10];
