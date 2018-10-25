function varargout = Controller(varargin)
% CONTROLLER MATLAB code for Controller.fig
%      CONTROLLER, by itself, creates a new CONTROLLER or raises the existing
%      singleton*.
%
%      H = CONTROLLER returns the handle to a new CONTROLLER or the handle to
%      the existing singleton*.
%
%      CONTROLLER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONTROLLER.M with the given input arguments.
%
%      CONTROLLER('Property','Value',...) creates a new CONTROLLER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Controller_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Controller_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Controller

% Last Modified by GUIDE v2.5 15-Mar-2018 09:52:19

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Controller_OpeningFcn, ...
                   'gui_OutputFcn',  @Controller_OutputFcn, ...
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


% --- Executes just before Controller is made visible.
function Controller_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Controller (see VARARGIN)

% Choose default command line output for Controller
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end
handles.XRadioBtn.Value = 0;
handles.YRadioBtn.Value = 0;
handles.ZRadioBtn.Value = 0;
handles.PhiRadioBtn.Value = 0;
handles.ThetaRadioBtn.Value = 0;
handles.PsiRadioBtn.Value = 0;
handles.PlotFlags = false(6,1);
handles.PlotConFlag = false;
handles.ControlFlags = false(3,1);
% Update handles structure
guidata(hObject, handles);
PlotAngles(handles)
% PlotAngles(handles)

% UIWAIT makes Controller wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Controller_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function CalculateNewAngles(handles)
% Controller settings
tic
handles.ControlFlags = false(3,1);
PGain = str2num(handles.PGain.String);
IGain = str2num(handles.IGain.String);
DGain = str2num(handles.DGain.String);
Ts = str2num(handles.Ts.String);
Fc = str2num(handles.Fc.String);

handles.ControlFlags(1,1) = PGain ~= 0;
handles.ControlFlags(2,1) = IGain ~= 0;
handles.ControlFlags(3,1) = DGain ~= 0;

% Get Data
Time                = handles.MainData.DataSet.CLOCK(5,:);
State               = handles.MainData.DataSet.STATE(1:6,:);
Target              = [zeros(size(handles.MainData.DataSet.TARGET(2:4,:))); (handles.MainData.DataSet.TARGET(2:4,:))];
ErrorEst            = Target - State;
ErrorMeas           = zeros(6, size(Time,2));
ErrorMeas(4:6,:)    = handles.MainData.DataSet.ERROR([2 6 10],:);
OutputMeas          = zeros(6, size(Time,2));
OutputMeas(4:6,:)   = handles.MainData.DataSet.CONTROLLER(2:4,:);


% Setup Complementary Filter
% Controller  = tf([1],[1]);
PController = tf(PGain*1, 1);
IController = tf(IGain,[1 0]);
DController = tf(DGain*[Fc 0],[1 Fc]);
Controller = PController + IController + DController;

PController = c2d(PController, Ts);
IController = c2d(IController, Ts);
DController = c2d(DController, Ts);
Controller = c2d(Controller, Ts);


% Put error through controller
OutputEst = zeros(6, size(Time,2));
for i = 1:6
    if(handles.PlotFlags(i))
        OutputEst(i,:) = lsim(Controller, ErrorEst(i,:));
    end
end

% Set data to plot
handles.PlotData.X = Time;

handles.PlotData.Y1 = [Target;  State];
handles.PlotData.Legend1 = {'Target X', 'Target Y', 'Target Z', 'Target \phi', 'Target \theta', 'Target \psi', 'State X', 'State Y', 'State Z', 'State \phi', 'State \theta', 'State \psi'};
handles.PlotData.Flags1 = [handles.PlotFlags; handles.PlotFlags];

handles.PlotData.Y2 = [ErrorEst; ErrorMeas];
handles.PlotData.Legend2 = {'Est X', 'Est Y', 'Est Z', 'Est \phi', 'Est \theta', 'Est \psi', 'Meas X', 'Meas Y', 'Meas Z', 'Meas \phi', 'Meas \theta', 'Meas \psi'};
handles.PlotData.Flags2 = [handles.PlotFlags; handles.PlotFlags];

handles.PlotData.Y3 = [OutputEst; OutputMeas];
handles.PlotData.Legend3 = {'Est X', 'Est Y', 'Est Z', 'Est \phi', 'Est \theta', 'Est \psi', 'Meas X', 'Meas Y', 'Meas Z', 'Meas \phi', 'Meas \theta', 'Meas \psi'};
handles.PlotData.Flags3 = [handles.PlotFlags; handles.PlotFlags];

handles.PlotData.Controller = Controller;
handles.PlotData.PController = PController;
handles.PlotData.IController = IController;
handles.PlotData.DController = DController;

% Plot Data
PlotAngles(handles)




function PlotAngles(handles)
% Set strings for easier for loop processing
AxesList = {'RefTarAxes', 'ErrorAxes', 'ConOutAxes', 'ControllerAxes'};
TitleList = {'Reference VS State', 'Error', 'Controller Output', 'Controller'};
YDataList = {'Y1', 'Y2', 'Y3'};
LegendList = {'Legend1', 'Legend2', 'Legend3'};
FlagList = {'Flags1', 'Flags2', 'Flags3'};
Time = handles.MainData.DataSet.CLOCK(5,:);

% Clear axes
for i = 1:4
    cla(handles.(AxesList{i}), 'reset')    
end

% Plot all requested data
if(any(handles.PlotFlags))
    for i = 1:3
        for j = 1:size(handles.PlotData.(YDataList{i}), 1)
            if((handles.PlotData.(FlagList{i})(j)))
                plot(handles.(AxesList{i}), handles.PlotData.X, handles.PlotData.(YDataList{i})(j,:)) % , 'color', Color(j, :)
                hold(handles.(AxesList{i}), 'on')
            end
        end
        legend(handles.(AxesList{i}), handles.PlotData.(LegendList{i})(handles.PlotData.(FlagList{i})), 'location', 'best')
    end
end
% Set all additional plot info
for i = 1:3
    grid(handles.(AxesList{i}), 'on')
    xlabel(handles.(AxesList{i}), 'Time')
    ylabel(handles.(AxesList{i}), 'Data')
    xlim(handles.(AxesList{i}), [Time(1) Time(end)])
    title(handles.(AxesList{i}), TitleList{i})
end

% Plot controleller bode plots
if(handles.PlotConFlag)
    hold(handles.ControllerAxes, 'on')
    % Only plot P-controller if gain is non-zero
    if(handles.ControlFlags(1))
        bode(handles.ControllerAxes, handles.PlotData.PController, 'r')
    end

    % Only plot I-controller if gain is non-zero
    if(handles.ControlFlags(2))
        bode(handles.ControllerAxes, handles.PlotData.IController, 'g')
    end

    % Only plot D-controller if gain is non-zero
    if(handles.ControlFlags(3))
        bode(handles.ControllerAxes, handles.PlotData.DController, 'b')
    end
    bode(handles.ControllerAxes, handles.PlotData.Controller, '--k')
    LegendStr = {'P-Gain', 'I-Gain', 'D-gain', 'Controller'};
    legend(handles.ControllerAxes, LegendStr([handles.ControlFlags; true]), 'location', 'best')
end
grid(handles.ControllerAxes, 'on')
title(handles.ControllerAxes, 'Controller')


% --- Executes on button press in SetControllerBtn.
function SetControllerBtn_Callback(hObject, eventdata, handles)
% hObject    handle to SetControllerBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotConFlag = true;
CalculateNewAngles(handles)
guidata(hObject, handles);


% --- Executes on button press in XRadioBtn.
function XRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to XRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(1) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of XRadioBtn


% --- Executes on button press in YRadioBtn.
function YRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(2) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of YRadioBtn


% --- Executes on button press in ZRadioBtn.
function ZRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ZRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(3) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of ZRadioBtn

% --- Executes on button press in PhiRadioBtn.
function PhiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PhiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(4) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of PhiRadioBtn


% --- Executes on button press in ThetaRadioBtn.
function ThetaRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ThetaRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(5) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of ThetaRadioBtn


% --- Executes on button press in PsiRadioBtn.
function PsiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PsiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(6) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of PsiRadioBtn


function PGain_Callback(hObject, eventdata, handles)
% hObject    handle to PGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PGain as text
%        str2double(get(hObject,'String')) returns contents of PGain as a double


% --- Executes during object creation, after setting all properties.
function PGain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function IGain_Callback(hObject, eventdata, handles)
% hObject    handle to IGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of IGain as text
%        str2double(get(hObject,'String')) returns contents of IGain as a double


% --- Executes during object creation, after setting all properties.
function IGain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function DGain_Callback(hObject, eventdata, handles)
% hObject    handle to DGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of DGain as text
%        str2double(get(hObject,'String')) returns contents of DGain as a double


% --- Executes during object creation, after setting all properties.
function DGain_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DGain (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Fc_Callback(hObject, eventdata, handles)
% hObject    handle to Fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fc as text
%        str2double(get(hObject,'String')) returns contents of Fc as a double


% --- Executes during object creation, after setting all properties.
function Fc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ts_Callback(hObject, eventdata, handles)
% hObject    handle to Ts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ts as text
%        str2double(get(hObject,'String')) returns contents of Ts as a double


% --- Executes during object creation, after setting all properties.
function Ts_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ts (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
