function varargout = Torques2PWM(varargin)
% TORQUES2PWM MATLAB code for Torques2PWM.fig
%      TORQUES2PWM, by itself, creates a new TORQUES2PWM or raises the existing
%      singleton*.
%
%      H = TORQUES2PWM returns the handle to a new TORQUES2PWM or the handle to
%      the existing singleton*.
%
%      TORQUES2PWM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TORQUES2PWM.M with the given input arguments.
%
%      TORQUES2PWM('Property','Value',...) creates a new TORQUES2PWM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Torques2PWM_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Torques2PWM_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Torques2PWM

% Last Modified by GUIDE v2.5 30-May-2018 14:50:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Torques2PWM_OpeningFcn, ...
                   'gui_OutputFcn',  @Torques2PWM_OutputFcn, ...
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


% --- Executes just before Torques2PWM is made visible.
function Torques2PWM_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Torques2PWM (see VARARGIN)

% Choose default command line output for Torques2PWM
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h(1))
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h(1));
end
handles.M1Btn.Value = 1;
handles.M2Btn.Value = 1;
handles.M3Btn.Value = 1;
handles.M4Btn.Value = 1;

handles.FBtn.Value = 1;
handles.TXBtn.Value = 1;
handles.TYBtn.Value = 1;
handles.TZBtn.Value = 1;

handles.PlotMotorFlags = true(4,1);
handles.PlotOutputFlags = true(4,1);
% Update handles structure
guidata(hObject, handles);
CalculateNewAngles(handles);

% UIWAIT makes Torques2PWM wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Torques2PWM_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function CalculateNewAngles(handles)
% Get Data
Time                = handles.MainData.DataSet.CLOCK(5,:);
ControlOutput       = handles.MainData.DataSet.CONTROLLER;
PWM                 = handles.MainData.DataSet.MOTOR(1:4,:);
PWM_MODE            = handles.MainData.DataSet.MOTOR(5:7,:);

% Get Type 1 Solutions
Sol1(1,:) = fix(PWM_MODE(1,:)/1000);
Sol1(2,:) = fix((PWM_MODE(1,:)-Sol1(1,:)*1000)/100);
Sol1(3,:) = fix((PWM_MODE(1,:)-Sol1(1,:)*1000-Sol1(2,:)*100)/10);
Sol1(4,:) = fix((PWM_MODE(1,:)-Sol1(1,:)*1000-Sol1(2,:)*100-Sol1(3,:)*10)/1);

% Get Type 2 Solutions
Sol2 = zeros(4, size(PWM_MODE(2,:), 2));
Sol2(1,:) = fix(PWM_MODE(2,:)/1000);
Sol2(2,:) = fix((PWM_MODE(2,:)-Sol2(1,:)*1000)/100);
Sol2(3,:) = fix((PWM_MODE(2,:)-Sol2(1,:)*1000-Sol2(2,:)*100)/10);
Sol2(4,:) = fix((PWM_MODE(2,:)-Sol2(1,:)*1000-Sol2(2,:)*100-Sol2(3,:)*10)/1);

% Motor Mode
Mode = PWM_MODE(3,:);


% Set data to plot
handles.PlotData.X = Time;

handles.PlotData.Y1 = [ControlOutput];
handles.PlotData.Legend1 = {'\tau_\phi', '\tau_\theta', '\tau_\psi', 'F'};
handles.PlotData.Flags1 = [handles.PlotOutputFlags];


handles.PlotData.Y2 = [PWM];
handles.PlotData.Legend2 = {'PWM_1', 'PWM_2', 'PWM_3', 'PWM_4'};
handles.PlotData.Flags2 = [handles.PlotMotorFlags];

handles.PlotData.Y3 = [Sol1; Sol2; Mode];
handles.PlotData.Legend3 = {'M_1 Sol_1', 'M_2 Sol_1', 'M_3 Sol_1', 'M_4 Sol_1','M_1 Sol_2', 'M_2 Sol_2', 'M_3 Sol_2', 'M_4 Sol_2', 'Mode'};
handles.PlotData.Flags3 = [fliplr(handles.PlotMotorFlags); fliplr(handles.PlotMotorFlags); true];

% Plot Data
PlotAngles(handles)




function PlotAngles(handles)
% Set strings for easier for loop processing
AxesList = {'ControllerAxes', 'PWMAxes', 'SolutionAxes'};
TitleList = {'Controller Output', 'PWM', 'Solutions'};
YDataList = {'Y1', 'Y2', 'Y3'};
LegendList = {'Legend1', 'Legend2', 'Legend3'};
FlagList = {'Flags1', 'Flags2', 'Flags3'};
Color = [1 0 0; 0 1 0; 0 0 1; 1 0 1; 0 1 1;  1 1 0; 0.75 0 0; 0 0.75 0; 0 0 0.75];
Time = handles.MainData.DataSet.CLOCK(5,:);

% Clear axes
for i = 1:3
    cla(handles.(AxesList{i}), 'reset')    
end

% Plot all requested data
if(any(handles.PlotOutputFlags) || any(handles.PlotMotorFlags))
    for i = 1:3
        for j = 1:size(handles.PlotData.(YDataList{i}), 1)
            if((handles.PlotData.(FlagList{i})(j)))
                plot(handles.(AxesList{i}), handles.PlotData.X, handles.PlotData.(YDataList{i})(j,:), 'Color', Color(j,:))
                hold(handles.(AxesList{i}), 'on')
            end
        end
        legend(handles.(AxesList{i}), handles.PlotData.(LegendList{i})(handles.PlotData.(FlagList{i})), 'location', 'southoutside', 'orientation', 'horizontal')
    end
end
linkaxes([handles.ControllerAxes handles.PWMAxes handles.SolutionAxes], 'x')
% Set all additional plot info
for i = 1:3
    grid(handles.(AxesList{i}), 'on')
    xlabel(handles.(AxesList{i}), 'Time')
    ylabel(handles.(AxesList{i}), 'Data')
    xlim(handles.(AxesList{i}), [Time(1) Time(end)])
    title(handles.(AxesList{i}), TitleList{i})
end


% --- Executes on button press in M1Btn.
function M1Btn_Callback(hObject, eventdata, handles)
% hObject    handle to M1Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotMotorFlags(1) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of M1Btn


% --- Executes on button press in M2Btn.
function M2Btn_Callback(hObject, eventdata, handles)
% hObject    handle to M2Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotMotorFlags(2) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of M2Btn


% --- Executes on button press in M3Btn.
function M3Btn_Callback(hObject, eventdata, handles)
% hObject    handle to M3Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotMotorFlags(3) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of M3Btn

% --- Executes on button press in M4Btn.
function M4Btn_Callback(hObject, eventdata, handles)
% hObject    handle to M4Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotMotorFlags(4) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of M4Btn

% --- Executes on button press in FBtn.
function FBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotOutputFlags(4) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of FBtn


% --- Executes on button press in TXBtn.
function TXBtn_Callback(hObject, eventdata, handles)
% hObject    handle to TXBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotOutputFlags(1) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of TXBtn


% --- Executes on button press in TYBtn.
function TYBtn_Callback(hObject, eventdata, handles)
% hObject    handle to TYBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotOutputFlags(2) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of TYBtn

% --- Executes on button press in TZBtn.
function TZBtn_Callback(hObject, eventdata, handles)
% hObject    handle to TZBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotOutputFlags(3) = get(hObject, 'Value');
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of TZBtn


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.ControllerAxes.Units    = 'normalized';
handles.PWMAxes.Units           = 'normalized';
handles.SolutionAxes.Units      = 'normalized';
handles.ControlPanel.Units      = 'normalized';

% OuterPosition = [X Y Width Height]
handles.ControllerAxes.OuterPosition    = [0.00 0.66 0.80 0.33];
handles.PWMAxes.OuterPosition           = [0.00 0.33 0.80 0.33];
handles.SolutionAxes.OuterPosition      = [0.00 0.00 0.80 0.33];
handles.ControlPanel.OuterPosition      = [0.80 0.50 0.20 0.50];


% --- Executes when ControlPanel is resized.
function ControlPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ControlPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.M1Btn.Units = 'normalized';
handles.M2Btn.Units = 'normalized';
handles.M3Btn.Units = 'normalized';
handles.M4Btn.Units = 'normalized';
handles.FBtn.Units  = 'normalized';
handles.TXBtn.Units = 'normalized';
handles.TYBtn.Units = 'normalized';
handles.TZBtn.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.M1Btn.OuterPosition = [0.00 0.90 1.00 0.10];
handles.M2Btn.OuterPosition = [0.00 0.80 1.00 0.10];
handles.M3Btn.OuterPosition = [0.00 0.70 1.00 0.10];
handles.M4Btn.OuterPosition = [0.00 0.60 1.00 0.10];
handles.FBtn.OuterPosition  = [0.00 0.50 1.00 0.10];
handles.TXBtn.OuterPosition = [0.00 0.40 1.00 0.10];
handles.TYBtn.OuterPosition = [0.00 0.30 1.00 0.10];
handles.TZBtn.OuterPosition = [0.00 0.20 1.00 0.10];
