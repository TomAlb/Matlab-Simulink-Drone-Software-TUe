function varargout = ShowErrorVSRef(varargin)
% SHOWERRORVSREF MATLAB code for showerrorvsref.fig
%      SHOWERRORVSREF, by itself, creates a new SHOWERRORVSREF or raises the existing
%      singleton*.
%
%      H = SHOWERRORVSREF returns the handle to a new SHOWERRORVSREF or the handle to
%      the existing singleton*.
%
%      SHOWERRORVSREF('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWERRORVSREF.M with the given input arguments.
%
%      SHOWERRORVSREF('Property','Value',...) creates a new SHOWERRORVSREF or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before showerrorvsref_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to showerrorvsref_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help showerrorvsref

% Last Modified by GUIDE v2.5 17-Sep-2018 15:45:50

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @ShowErrorVSRef_OpeningFcn, ...
    'gui_OutputFcn',  @ShowErrorVSRef_OutputFcn, ...
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


% --- Executes just before showerrorvsref is made visible.
function ShowErrorVSRef_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to showerrorvsref (see VARARGIN)

% Choose default command line output for showerrorvsref
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(6,1);

set(handles.XRadioBtn, 'value', 0);
set(handles.YRadioBtn, 'value', 0);
set(handles.ZRadioBtn, 'value', 0);
set(handles.PhiRadioBtn, 'value', 0);
set(handles.ThetaRadioBtn, 'value', 0);
set(handles.PsiRadioBtn, 'value', 0);

cla(handles.ErrorAxes, 'reset');
grid(handles.ErrorAxes, 'on');
axis(handles.ErrorAxes, 'tight'); 
xlabel(handles.ErrorAxes, 'Time');
ylabel(handles.ErrorAxes, 'Data'); 
title(handles.ErrorAxes, 'Error Data');

cla(handles.RefStateAxes, 'reset');
grid(handles.RefStateAxes, 'on');
axis(handles.RefStateAxes, 'tight'); 
xlabel(handles.RefStateAxes, 'Time');
ylabel(handles.RefStateAxes, 'Data'); 
title(handles.RefStateAxes, 'Reference vs State Data');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes showerrorvsref wait for user response (see UIRESUME)
% uiwait(handles.ShowErrorVSRef);


% --- Outputs from this function are returned to the command line.
function varargout = ShowErrorVSRef_OutputFcn(hObject, eventdata, handles)
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
handles.RequestDataFlags(4) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(4) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of XRadioBtn


% --- Executes on button press in YRadioBtn.
function YRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(5) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(5) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of YRadioBtn


% --- Executes on button press in ZRadioBtn.
function ZRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ZRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(6) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(6) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ZRadioBtn


% --- Executes on button press in PhiRadioBtn.
function PhiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PhiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(1) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(1) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PhiRadioBtn

% --- Executes on button press in ThetaRadioBtn.
function ThetaRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ThetaRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(2) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(2) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ThetaRadioBtn

% --- Executes on button press in PsiRadioBtn.
function PsiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PsiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(3) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(3) = true;
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

State = handles.MainData.DataSet.STATE([4 5 6 1 2 3],:);
Ref   = handles.MainData.DataSet.TARGET(2:7,:);

cla(handles.RefStateAxes, 'reset');
hold(handles.RefStateAxes, 'on');
cla(handles.ErrorAxes, 'reset');
hold(handles.ErrorAxes, 'on');

RefLegendString = {'R_\phi', 'R_\theta', 'R_\psi', 'R_X', 'R_Y', 'R_Z'};
StateLegendString = {'\phi', '\theta', '\psi', 'X', 'Y', 'Z'};
ErrorLegendString = {'E_\phi', 'E_\theta', 'E_\psi', 'E_X', 'E_Y', 'E_Z'};

if(any(handles.RequestDataFlags))
    for i = 1:length(handles.RequestDataFlags)
        if(handles.RequestDataFlags(i))
            plot(handles.RefStateAxes, Time, Ref(i,:), 'DisplayName', RefLegendString{i})
            plot(handles.RefStateAxes, Time, State(i,:), 'DisplayName', StateLegendString{i})
            plot(handles.ErrorAxes, Time, Ref(i,:)-State(i,:), 'DisplayName', ErrorLegendString{i})
        end
    end
end

grid(handles.ErrorAxes, 'on');
axis(handles.ErrorAxes, 'tight'); 
xlabel(handles.ErrorAxes, 'Time');
ylabel(handles.ErrorAxes, 'Data'); 
legend(handles.ErrorAxes, 'show'); 
title(handles.ErrorAxes, 'Error Data');

grid(handles.RefStateAxes, 'on');
axis(handles.RefStateAxes, 'tight'); 
xlabel(handles.RefStateAxes, 'Time');
ylabel(handles.RefStateAxes, 'Data'); 
legend(handles.RefStateAxes, 'show'); 
title(handles.RefStateAxes, 'Reference vs State Data');


% --- Executes when ShowRawSensorData is resized.
function ShowRawSensorData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowRawSensorData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RefStateAxes.Units = 'normalized';
handles.ErrorAxes.Units    = 'normalized';
handles.StatePanel.Units   = 'normalized';

% OuterPosition = [X Y Width Height]
handles.RefStateAxes.OuterPosition = [0.00 1/3 0.80 2/3];
handles.ErrorAxes.OuterPosition    = [0.00 0.00 0.80 1/3];
handles.StatePanel.OuterPosition   = [0.80 0.50 0.20 0.50];


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
handles.PhiRadioBtn.OuterPosition   = [0.00 0.90 1.00 0.10];
handles.ThetaRadioBtn.OuterPosition = [0.00 0.80 1.00 0.10];
handles.PsiRadioBtn.OuterPosition   = [0.00 0.70 1.00 0.10];
handles.XRadioBtn.OuterPosition     = [0.00 0.60 1.00 0.10];
handles.YRadioBtn.OuterPosition     = [0.00 0.50 1.00 0.10];
handles.ZRadioBtn.OuterPosition     = [0.00 0.40 1.00 0.10];
