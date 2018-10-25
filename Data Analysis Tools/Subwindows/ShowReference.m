function varargout = ShowRAwSensorData(varargin)
% SHOWREFERENCE MATLAB code for showreference.fig
%      SHOWREFERENCE, by itself, creates a new SHOWREFERENCE or raises the existing
%      singleton*.
%
%      H = SHOWREFERENCE returns the handle to a new SHOWREFERENCE or the handle to
%      the existing singleton*.
%
%      SHOWREFERENCE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWREFERENCE.M with the given input arguments.
%
%      SHOWREFERENCE('Property','Value',...) creates a new SHOWREFERENCE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before showreference_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to showreference_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help showreference

% Last Modified by GUIDE v2.5 30-May-2018 14:34:25

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ShowReference_OpeningFcn, ...
                   'gui_OutputFcn',  @ShowReference_OutputFcn, ...
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


% --- Executes just before showreference is made visible.
function ShowReference_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to showreference (see VARARGIN)

% Choose default command line output for showreference
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(14,1);
set(handles.PosRadioBtn, 'value', 1);
set(handles.RateRadioBtn, 'value', 1);

set(handles.ThrottleRadioBtn, 'value', 0);
set(handles.RollRadioBtn, 'value', 0);
set(handles.PitchRadioBtn, 'value', 0);
set(handles.YawRadioBtn, 'value', 0);
set(handles.XRadioBtn, 'value', 0);
set(handles.YRadioBtn, 'value', 0);
set(handles.ZRadioBtn, 'value', 0);

cla(handles.ReferenceAxes, 'reset')
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('Reference')
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes showreference wait for user response (see UIRESUME)
% uiwait(handles.ShowReference);


% --- Outputs from this function are returned to the command line.
function varargout = ShowReference_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in PosRadioBtn.
function PosRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PosRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ThrottleRadioBtn, 'value') get(handles.RollRadioBtn, 'value') get(handles.PitchRadioBtn, 'value') get(handles.YawRadioBtn, 'value') get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(1:7) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(1:7) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PosRadioBtn


% --- Executes on button press in RateRadioBtn.
function RateRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RateRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ThrottleRadioBtn, 'value') get(handles.RollRadioBtn, 'value') get(handles.PitchRadioBtn, 'value') get(handles.YawRadioBtn, 'value') get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.RequestDataFlags(8:14) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(8:14) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of RateRadioBtn

% --- Executes on button press in ThrottleRadioBtn.
function ThrottleRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ThrottleRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.RateRadioBtn, 'value')];
handles.RequestDataFlags([1 8]) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags([1 8]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ThrottleRadioBtn


% --- Executes on button press in RollRadioBtn.
function RollRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RollRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.RateRadioBtn, 'value')];
handles.RequestDataFlags([2 9]) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags([2 9]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of RollRadioBtn


% --- Executes on button press in PitchRadioBtn.
function PitchRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PitchRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.RateRadioBtn, 'value')];
handles.RequestDataFlags([3 10]) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags([3 10]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PitchRadioBtn


% --- Executes on button press in YawRadioBtn.
function YawRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YawRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.RateRadioBtn, 'value')];
handles.RequestDataFlags([4 11]) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags([4 11]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of YawRadioBtn


% --- Executes on button press in XRadioBtn.
function XRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to XRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.RateRadioBtn, 'value')];
handles.RequestDataFlags([5 12]) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags([5 12]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of XRadioBtn


% --- Executes on button press in YRadioBtn.
function YRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.RateRadioBtn, 'value')];
handles.RequestDataFlags([6 13]) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags([6 13]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of YRadioBtn

% --- Executes on button press in ZRadioBtn.
function ZRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ZRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.RateRadioBtn, 'value')];
handles.RequestDataFlags([7 14]) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags([7 14]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ZRadioBtn


% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to YRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Time = handles.MainData.DataSet.CLOCK(5,:);
Data = handles.MainData.DataSet.TARGET(handles.RequestDataFlags, :);
cla(handles.ReferenceAxes, 'reset')
LegendString = {'Throttle', 'Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z', 'Throttle Rate', 'Roll Rate', 'Pitch Rate', 'Yaw Rate', 'X Rate', 'Y Rate', 'Z Rate'};
if(any(handles.RequestDataFlags))
    plot(Time, Data)
    legend(LegendString{handles.RequestDataFlags},'location', 'best')
end
    grid on
    axis tight
    xlabel('Time')
    ylabel('Data')
    title('Reference')


% --- Executes when ShowRawSensorData is resized.
function ShowRawSensorData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowRawSensorData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ReferenceAxes.Units     = 'normalized';
handles.ReferencePanel.Units    = 'normalized';
handles.Order_Panel.Units       = 'normalized';

% OuterPosition = [X Y Width Height]
handles.ReferenceAxes.OuterPosition     = [0.00 0.00 0.80 1.00];
handles.ReferencePanel.OuterPosition    = [0.80 0.50 0.20 0.50];
handles.Order_Panel.OuterPosition       = [0.80 0.00 0.20 0.50];


% --- Executes when ReferencePanel is resized.
function ReferencePanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ReferencePanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ThrottleRadioBtn.Units  = 'normalized';
handles.RollRadioBtn.Units      = 'normalized';
handles.PitchRadioBtn.Units     = 'normalized';
handles.YawRadioBtn.Units       = 'normalized';
handles.XRadioBtn.Units         = 'normalized';
handles.YRadioBtn.Units         = 'normalized';
handles.ZRadioBtn.Units         = 'normalized';

% OuterPosition = [X Y Width Height]
handles.ThrottleRadioBtn.OuterPosition  = [0.00 0.90 1.00 0.10];
handles.RollRadioBtn.OuterPosition      = [0.00 0.80 1.00 0.10];
handles.PitchRadioBtn.OuterPosition     = [0.00 0.70 1.00 0.10];
handles.YawRadioBtn.OuterPosition       = [0.00 0.60 1.00 0.10];
handles.XRadioBtn.OuterPosition         = [0.00 0.50 1.00 0.10];
handles.YRadioBtn.OuterPosition         = [0.00 0.40 1.00 0.10];
handles.ZRadioBtn.OuterPosition         = [0.00 0.30 1.00 0.10];


% --- Executes when Order_Panel is resized.
function Order_Panel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to Order_Panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PosRadioBtn.Units   = 'normalized';
handles.RateRadioBtn.Units  = 'normalized';

% OuterPosition = [X Y Width Height]
handles.PosRadioBtn.OuterPosition   = [0.00 0.90 1.00 0.10];
handles.RateRadioBtn.OuterPosition  = [0.00 0.80 1.00 0.10];
