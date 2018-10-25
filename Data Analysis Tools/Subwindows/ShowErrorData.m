function varargout = ShowErrorData(varargin)
% SHOWERRORDATA MATLAB code for showerrordata.fig
%      SHOWERRORDATA, by itself, creates a new SHOWERRORDATA or raises the existing
%      singleton*.
%
%      H = SHOWERRORDATA returns the handle to a new SHOWERRORDATA or the handle to
%      the existing singleton*.
%
%      SHOWERRORDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWERRORDATA.M with the given input arguments.
%
%      SHOWERRORDATA('Property','Value',...) creates a new SHOWERRORDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before showerrordata_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to showerrordata_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help showerrordata

% Last Modified by GUIDE v2.5 24-May-2018 11:15:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @ShowErrorData_OpeningFcn, ...
    'gui_OutputFcn',  @ShowErrorData_OutputFcn, ...
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


% --- Executes just before showerrordata is made visible.
function ShowErrorData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to showerrordata (see VARARGIN)

% Choose default command line output for showerrordata
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(30,1);
set(handles.ErrorBtn, 'value', 1);
set(handles.IErrorBtn, 'value', 1);
set(handles.DErrorBtn, 'value', 1);
set(handles.FFBtn, 'value', 1);
set(handles.OutputBtn, 'value', 1);

set(handles.XRadioBtn, 'value', 0);
set(handles.YRadioBtn, 'value', 0);
set(handles.ZRadioBtn, 'value', 0);
set(handles.PhiRadioBtn, 'value', 0);
set(handles.ThetaRadioBtn, 'value', 0);
set(handles.PsiRadioBtn, 'value', 0);

cla(handles.ErrorDataAxes, 'reset')
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('Error Data')
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes showerrordata wait for user response (see UIRESUME)
% uiwait(handles.ShowErrorData);


% --- Outputs from this function are returned to the command line.
function varargout = ShowErrorData_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in ErrorBtn.
function ErrorBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ErrorBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value') get(handles.PhiRadioBtn, 'value') get(handles.ThetaRadioBtn, 'value') get(handles.PsiRadioBtn, 'value')];
handles.RequestDataFlags([1 6 11 16 21 26]) = false(6,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([1 6 11 16 21 26]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ErrorBtn

% --- Executes on button press in IErrorBtn.
function IErrorBtn_Callback(hObject, eventdata, handles)
% hObject    handle to IErrorBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value') get(handles.PhiRadioBtn, 'value') get(handles.ThetaRadioBtn, 'value') get(handles.PsiRadioBtn, 'value')];
handles.RequestDataFlags([2 7 12 17 22 27]) = false(6,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([2 7 12 17 22 27]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of IErrorBtn


% --- Executes on button press in FFBtn.
function DErrorBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FFBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value') get(handles.PhiRadioBtn, 'value') get(handles.ThetaRadioBtn, 'value') get(handles.PsiRadioBtn, 'value')];
handles.RequestDataFlags([3 8 13 18 23 28]) = false(6,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([3 8 13 18 23 28]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of FFBtn


% --- Executes on button press in FFBtn.
function FFBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FFBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value') get(handles.PhiRadioBtn, 'value') get(handles.ThetaRadioBtn, 'value') get(handles.PsiRadioBtn, 'value')];
handles.RequestDataFlags([4 9 14 19 24 29]) = false(6,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([4 9 14 19 24 29]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of FFBtn

% --- Executes on button press in OutputBtn.
function OutputBtn_Callback(hObject, eventdata, handles)
% hObject    handle to OutputBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value') get(handles.PhiRadioBtn, 'value') get(handles.ThetaRadioBtn, 'value') get(handles.PsiRadioBtn, 'value')];
handles.RequestDataFlags([5 10 15 20 25 30]) = false(6,1);
if(get(hObject,'Value'))
    handles.RequestDataFlags([5 10 15 20 25 30]) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of OutputBtn

% --- Executes on button press in XRadioBtn.
function XRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to XRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ErrorBtn, 'value') get(handles.IErrorBtn, 'value') get(handles.DErrorBtn, 'value') get(handles.FFBtn, 'value') get(handles.OutputBtn, 'value')];
handles.RequestDataFlags(1:5) = false(1,5);
if(get(hObject,'Value'))
    handles.RequestDataFlags(1:5) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of XRadioBtn


% --- Executes on button press in YRadioBtn.
function YRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ErrorBtn, 'value') get(handles.IErrorBtn, 'value') get(handles.DErrorBtn, 'value') get(handles.FFBtn, 'value') get(handles.OutputBtn, 'value')];
handles.RequestDataFlags(6:10) = false(1,5);
if(get(hObject,'Value'))
    handles.RequestDataFlags(6:10) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of YRadioBtn


% --- Executes on button press in ZRadioBtn.
function ZRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ZRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ErrorBtn, 'value') get(handles.IErrorBtn, 'value') get(handles.DErrorBtn, 'value') get(handles.FFBtn, 'value') get(handles.OutputBtn, 'value')];
handles.RequestDataFlags(11:15) = false(1,5);
if(get(hObject,'Value'))
    handles.RequestDataFlags(11:15) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ZRadioBtn


% --- Executes on button press in PhiRadioBtn.
function PhiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PhiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ErrorBtn, 'value') get(handles.IErrorBtn, 'value') get(handles.DErrorBtn, 'value') get(handles.FFBtn, 'value') get(handles.OutputBtn, 'value')];
handles.RequestDataFlags(16:20) = false(1,5);
if(get(hObject,'Value'))
    handles.RequestDataFlags(16:20) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PhiRadioBtn

% --- Executes on button press in ThetaRadioBtn.
function ThetaRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ThetaRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ErrorBtn, 'value') get(handles.IErrorBtn, 'value') get(handles.DErrorBtn, 'value') get(handles.FFBtn, 'value') get(handles.OutputBtn, 'value')];
handles.RequestDataFlags(21:25) = false(1,5);
if(get(hObject,'Value'))
    handles.RequestDataFlags(21:25) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ThetaRadioBtn

% --- Executes on button press in PsiRadioBtn.
function PsiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PsiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.ErrorBtn, 'value') get(handles.IErrorBtn, 'value') get(handles.DErrorBtn, 'value') get(handles.FFBtn, 'value') get(handles.OutputBtn, 'value')];
handles.RequestDataFlags(26:30) = false(1,5);
if(get(hObject,'Value'))
    handles.RequestDataFlags(26:30) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PsiRadioBtn

% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to ThetaRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Time = handles.MainData.DataSet.CLOCK(5,:);
Data = zeros(24, size(Time,2));
% Data = handles.MainData.DataSet.RAW_SENSOR_DATA(handles.RequestDataFlags, :);
Data(1:15,:) = handles.MainData.DataSet.ERROR([16:30],:);
Data(16:30,:) = handles.MainData.DataSet.ERROR([1:15],:);
cla(handles.ErrorDataAxes, 'reset')
LegendString = {'X_{Pe}', 'X_{Ie}', 'X_{De}', 'X_{FF}', 'X_{Out}', 'Y_{Pe}', 'Y_{Ie}', 'Y_{De}', 'Y_{FF}', 'Y_{Out}', 'Z_{Pe}', 'Z_{Ie}', 'Z_{De}', 'Z_{FF}', 'Z_{Out}', '\phi_{Pe}', '\phi_{Ie}', '\phi_{De}', '\phi_{FF}', '\phi_{Out}', '\theta_{Pe}', '\theta_{Ie}', '\theta_{De}', '\theta_{FF}', '\theta_{Out}', '\psi_{Pe}', '\psi_{Ie}', '\psi_{De}', '\psi_{FF}', '\psi_{Out}'};
if(any(handles.RequestDataFlags))
    plot(Time, Data(handles.RequestDataFlags,:))
    legend(LegendString{handles.RequestDataFlags},'location', 'best')
end
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('Error Data')


% --- Executes when ShowRawSensorData is resized.
function ShowRawSensorData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowRawSensorData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ErrorDataAxes.Units = 'normalized';
handles.StatePanel.Units    = 'normalized';
handles.ErrorPanel.Units    = 'normalized';

% OuterPosition = [X Y Width Height]
handles.ErrorDataAxes.OuterPosition = [0.00 0.00 0.80 1.00];
handles.StatePanel.OuterPosition    = [0.80 0.50 0.20 0.50];
handles.ErrorPanel.OuterPosition    = [0.80 0.00 0.20 0.50];


% --- Executes when StatePanel is resized.
function StatePanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to StatePanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.XRadioBtn.Units     = 'normalized';
handles.YRadioBtn.Units     = 'normalized';
handles.ZRadioBtn.Units     = 'normalized';
handles.PhiRadioBtn.Units   = 'normalized';
handles.ThetaRadioBtn.Units = 'normalized';
handles.PsiRadioBtn.Units   = 'normalized';

% OuterPosition = [X Y Width Height]
handles.XRadioBtn.OuterPosition     = [0.00 0.90 1.00 0.10];
handles.YRadioBtn.OuterPosition     = [0.00 0.80 1.00 0.10];
handles.ZRadioBtn.OuterPosition     = [0.00 0.70 1.00 0.10];
handles.PhiRadioBtn.OuterPosition   = [0.00 0.60 1.00 0.10];
handles.ThetaRadioBtn.OuterPosition = [0.00 0.50 1.00 0.10];
handles.PsiRadioBtn.OuterPosition   = [0.00 0.40 1.00 0.10];


% --- Executes when ErrorPanel is resized.
function ErrorPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ErrorPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ErrorBtn.Units  = 'normalized';
handles.IErrorBtn.Units = 'normalized';
handles.DErrorBtn.Units = 'normalized';
handles.FFBtn.Units     = 'normalized';
handles.OutputBtn.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.ErrorBtn.OuterPosition  = [0.00 0.90 1.00 0.10];
handles.IErrorBtn.OuterPosition = [0.00 0.80 1.00 0.10];
handles.DErrorBtn.OuterPosition = [0.00 0.70 1.00 0.10];
handles.FFBtn.OuterPosition     = [0.00 0.60 1.00 0.10];
handles.OutputBtn.OuterPosition = [0.00 0.50 1.00 0.10];

