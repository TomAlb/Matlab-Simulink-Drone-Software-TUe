function varargout = Timing(varargin)
% TIMING MATLAB code for timing.fig
%      TIMING, by itself, creates a new TIMING or raises the existing
%      singleton*.
%
%      H = TIMING returns the handle to a new TIMING or the handle to
%      the existing singleton*.
%
%      TIMING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TIMING.M with the given input arguments.
%
%      TIMING('Property','Value',...) creates a new TIMING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before timing_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to timing_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help timing

% Last Modified by GUIDE v2.5 15-Aug-2018 18:37:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Timing_OpeningFcn, ...
                   'gui_OutputFcn',  @Timing_OutputFcn, ...
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


% --- Executes just before timing is made visible.
function Timing_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to timing (see VARARGIN)

% Choose default command line output for timing
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(5,1);
set(handles.SensorsTSRadioBtn, 'value', 0);
set(handles.PositionTSRadioBtn, 'value', 0);
set(handles.GPSTSRadioBtn, 'value', 0);
set(handles.BatteryTSRadioBtn, 'value', 0);
set(handles.CounterTSRadioBtn, 'value', 0);

cla(handles.TimeAxes, 'reset'); 
grid(handles.TimeAxes, 'on'); 
axis(handles.TimeAxes, 'tight'); 
xlabel(handles.TimeAxes, 'Measurement'); 
ylabel(handles.TimeAxes, 'Time'); 
title(handles.TimeAxes, 'Time');

cla(handles.DiffAxes, 'reset'); 
grid(handles.DiffAxes, 'on'); 
axis(handles.DiffAxes, 'tight'); 
xlabel(handles.DiffAxes, 'Measurement'); 
ylabel(handles.DiffAxes, 'Period'); 
title(handles.DiffAxes, 'Time Period');

cla(handles.PDFAxes, 'reset'); 
grid(handles.PDFAxes, 'on'); 
axis(handles.PDFAxes, 'tight'); 
xlabel(handles.PDFAxes, 'Sampling Period'); 
ylabel(handles.PDFAxes, 'Occurances'); 
title(handles.PDFAxes, 'Sampling time PDF');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes timing wait for user response (see UIRESUME)
% uiwait(handles.Timing);


% --- Outputs from this function are returned to the command line.
function varargout = Timing_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in SensorsTSRadioBtn.
function SensorsTSRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to SensorsTSRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(1) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(1) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of SensorsTSRadioBtn


% --- Executes on button press in PositionTSRadioBtn.
function PositionTSRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PositionTSRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(2) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(2) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PositionTSRadioBtn


% --- Executes on button press in GPSTSRadioBtn.
function GPSTSRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to GPSTSRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(3) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(3) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of GPSTSRadioBtn


% --- Executes on button press in BatteryTSRadioBtn.
function BatteryTSRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to BatteryTSRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(4) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(4) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of BatteryTSRadioBtn


% --- Executes on button press in CounterTSRadioBtn.
function CounterTSRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to CounterTSRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.RequestDataFlags(5) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(5) = true;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of CounterTSRadioBtn

% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to CounterTSRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Time_Source = handles.MainData.DataSet.CLOCK(handles.RequestDataFlags,:);

%% Plot Time
cla(handles.TimeAxes, 'reset'); hold(handles.TimeAxes, 'on');
LegendString = {'Sensors', 'Position', 'GPS', 'Battery', 'Counter'};

if(any(handles.RequestDataFlags))
    for i = 1:size(Time_Source,1)
        plot(handles.TimeAxes, Time_Source(i,:))
    end
    legend(handles.TimeAxes, LegendString{handles.RequestDataFlags},'location', 'eastoutside')
end
grid(handles.TimeAxes, 'on'); 
axis(handles.TimeAxes, 'tight'); 
xlabel(handles.TimeAxes, 'Measurement'); 
ylabel(handles.TimeAxes, 'Time'); 
title(handles.TimeAxes, 'Time');

%% Plot Differential
cla(handles.DiffAxes, 'reset'); hold(handles.DiffAxes, 'on');
LegendString = {'Sensors', 'Position', 'GPS', 'Battery', 'Counter'};

if(any(handles.RequestDataFlags))
    for i = 1:size(Time_Source,1)
        plot(handles.DiffAxes, diff(Time_Source(i,:)), '.')
    end
    legend(handles.DiffAxes, LegendString{handles.RequestDataFlags},'location', 'eastoutside')
end
grid(handles.DiffAxes, 'on'); 
axis(handles.DiffAxes, 'tight'); 
xlabel(handles.DiffAxes, 'Measurement'); 
ylabel(handles.DiffAxes, 'Period'); 
title(handles.DiffAxes, 'Time Period');

%% Plot PDF
cla(handles.PDFAxes, 'reset'); hold(handles.PDFAxes, 'on');
LegendString = {'Sensors', 'Position', 'GPS', 'Battery', 'Counter'};

if(any(handles.RequestDataFlags))
    for i = 1:size(Time_Source,1)
        pd = fitdist(diff(Time_Source(i,:))','Normal');
        x = linspace(pd.mu - 4*pd.sigma, pd.mu + 4*pd.sigma, 100);
        pdf_normal = pdf(pd, x);
        % normalize pdf
        if(max(pdf_normal) > 0)
            pdf_normal = pdf_normal/max(pdf_normal);
        end
        plot(handles.PDFAxes,x, pdf_normal, '--', 'Linewidth', 2)
    end
    legend(handles.PDFAxes, LegendString{handles.RequestDataFlags},'location', 'eastoutside')
end 
grid(handles.PDFAxes, 'on'); 
axis(handles.PDFAxes, 'tight'); 
xlabel(handles.PDFAxes, 'Sampling Period'); 
ylabel(handles.PDFAxes, 'Occurances'); 
title(handles.PDFAxes, 'Sampling time PDF');

%% Plot PDF
% cla(handles.PDFAxes, 'reset'); 
% if(any(handles.RequestDataFlags))
%     plot(Time_Source(handles.RequestDataFlags,:))
% end
% grid on; axis tight; xlabel('Sampling Period'); ylabel('Occurances'); title('Sampling time PDF');


% --- Executes when Timing is resized.
function Timming_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to Timing (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.TimeAxes.Units          = 'normalized';
handles.DiffAxes.Units          = 'normalized';
handles.PDFAxes.Units           = 'normalized';
handles.Time_Source_Panel.Units = 'normalized';
handles.Direction_Panel.Units   = 'normalized';

% OuterPosition = [X Y Width Height]
handles.TimeAxes.OuterPosition          = [0 2/3 0.8 1/3];
handles.DiffAxes.OuterPosition          = [0 1/3 0.8 1/3];
handles.PDFAxes.OuterPosition           = [0 0.0 0.8 1/3];
handles.Time_Source_Panel.OuterPosition = [0.8 0.0 0.2 1.0];


% --- Executes when Time_Source_Panel is resized.
function Time_Source_Panel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to Time_Source_Panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.SensorsTSRadioBtn.Units     = 'normalized';
handles.PositionTSRadioBtn.Units    = 'normalized';
handles.GPSTSRadioBtn.Units         = 'normalized';
handles.BatteryTSRadioBtn.Units     = 'normalized';
handles.CounterTSRadioBtn.Units     = 'normalized';

% OuterPosition = [X Y Width Height]
handles.SensorsTSRadioBtn.OuterPosition     = [0.00 0.90 1.00 0.10];
handles.PositionTSRadioBtn.OuterPosition    = [0.00 0.80 1.00 0.10];
handles.GPSTSRadioBtn.OuterPosition         = [0.00 0.70 1.00 0.10];
handles.BatteryTSRadioBtn.OuterPosition     = [0.00 0.60 1.00 0.10];
handles.CounterTSRadioBtn.OuterPosition     = [0.00 0.50 1.00 0.10];
