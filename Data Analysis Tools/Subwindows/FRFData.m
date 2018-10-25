function varargout = FRFData(varargin)
% FRFDATA MATLAB code for frfdata.fig
%      FRFDATA, by itself, creates a new FRFDATA or raises the existing
%      singleton*.
%
%      H = FRFDATA returns the handle to a new FRFDATA or the handle to
%      the existing singleton*.
%
%      FRFDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FRFDATA.M with the given input arguments.
%
%      FRFDATA('Property','Value',...) creates a new FRFDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before frfdata_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to frfdata_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help frfdata

% Last Modified by GUIDE v2.5 27-Jun-2018 11:47:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FRFData_OpeningFcn, ...
                   'gui_OutputFcn',  @FRFData_OutputFcn, ...
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


% --- Executes just before frfdata is made visible.
function FRFData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to frfdata (see VARARGIN)

% Choose default command line output for frfdata
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end


set(handles.MainDataListBox, 'Max', 1);
set(handles.SubDataListBox, 'Max', 1);
set(handles.SubSubDataListBox, 'Max', 1);
handles.MainDataListBox.String = {'None', 'Raw Sensor', 'Filtered Sensor', 'RC', 'Reference', 'Error', 'States'};
handles.MainDataListBox.Value = 1;
handles.SubDataListBox.Value = 1;
handles.SubSubDataListBox.Value = 1;
handles.SubDataListBox.String = {};
handles.SubSubDataListBox.String = {};

handles.RequestDataFlags.Raw_Sensor = false(18,1);
handles.RequestDataFlags.Filtered_Sensor = false(18,1);
handles.RequestDataFlags.RC = false(4,1);
handles.RequestDataFlags.Reference = false(7,1);
handles.RequestDataFlags.Error = false(30,1);
handles.RequestDataFlags.State = false(18,1);

cla(handles.FRFDataAxes, 'reset')
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('FRF Data')
set(handles.FRFDataAxes, 'XScale', 'Log');
set(handles.FRFDataAxes, 'YScale', 'Log');
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes frfdata wait for user response (see UIRESUME)
% uiwait(handles.FRFData);


% --- Outputs from this function are returned to the command line.
function varargout = FRFData_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to BatteryRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.FRFDataAxes, 'reset'); hold on
Time = handles.MainData.DataSet.CLOCK(end,:);


T = mean(diff(Time));   % Sampling Period
Fs = 1/T;               % Sampling Frequency
L = length(Time);       % Length of signal

% Load Raw Sensor Data
Data.Raw_Sensor = handles.MainData.DataSet.RAW_SENSOR_DATA(handles.RequestDataFlags.Raw_Sensor,:);
FullLegendString.Raw_Sensor = {'m_x_{Raw}', 'm_y_{Raw}', 'm_z_{Raw}', 'a_x_{Raw}', 'a_y_{Raw}', 'a_z_{Raw}', '\omega_x_{Raw}', '\omega_y_{Raw}', '\omega_z_{Raw}', 'Pres_{Raw}', 'Latitude_{Raw}' , 'Longitude_{Raw}', 'Altitude_{Raw}', 'N Sats_{Raw}', 'Voltage_{Raw}', 'Current_{Raw}', 'mAh_{Raw}', 'Sonar_{Raw}' };
LegendString.Raw_Sensor = FullLegendString.Raw_Sensor(handles.RequestDataFlags.Raw_Sensor);
% Plot Raw Sensor Data
if(any(handles.RequestDataFlags.Raw_Sensor))
    for i = 1:size(Data.Raw_Sensor,1)
        Y = fft(Data.Raw_Sensor(i,:));
        P2 = abs(Y/L);
        P1 = P2(1:floor(L/2)+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;
        loglog(f, P1, 'DisplayName', LegendString.Raw_Sensor{i})
    end 
end

% Load Filtered Sensor Data
Data.Fil_Sensor = handles.MainData.DataSet.FILTERED_SENSOR_DATA(handles.RequestDataFlags.Filtered_Sensor,:);
FullLegendString.Fil_Sensor = {'m_x_{Fil}', 'm_y_{Fil}', 'm_z_{Fil}', 'a_x_{Fil}', 'a_y_{Fil}', 'a_z_{Fil}', '\omega_x_{Fil}', '\omega_y_{Fil}', '\omega_z_{Fil}', 'Pres_{Fil}', 'Latitude_{Fil}' , 'Longitude_{Fil}', 'Altitude_{Fil}', 'N Sats_{Fil}', 'Voltage_{Fil}', 'Current_{Fil}', 'mAh_{Fil}', 'Sonar_{Fil}' };
LegendString.Fil_Sensor = FullLegendString.Fil_Sensor(handles.RequestDataFlags.Filtered_Sensor);
% Plot Filtered Sensor Data
if(any(handles.RequestDataFlags.Filtered_Sensor))
    for i = 1:size(Data.Fil_Sensor,1)
        Y = fft(Data.Fil_Sensor(i,:));
        P2 = abs(Y/L);
        P1 = P2(1:floor(L/2)+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;
        loglog(f, P1, 'DisplayName', LegendString.Fil_Sensor{i})
    end 
end

% Load RC Data
Data.RC = handles.MainData.DataSet.RC(handles.RequestDataFlags.RC,:);
FullLegendString.RC = {'Roll_{RC}', 'Pitch_{RC}', 'Yaw_{RC}', 'Thrust_{RC}'};
LegendString.RC = FullLegendString.RC(handles.RequestDataFlags.RC);
% Plot RC Data
if(any(handles.RequestDataFlags.RC))
    for i = 1:size(Data.RC,1)
        Y = fft(Data.RC(i,:));
        P2 = abs(Y/L);
        P1 = P2(1:floor(L/2)+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;
        loglog(f, P1, 'DisplayName', LegendString.RC{i})
    end 
end

% Load Reference Data
Data.Reference = handles.MainData.DataSet.TARGET(handles.RequestDataFlags.Reference,:);
FullLegendString.Reference = {'Throttle_{ref}', 'Roll_{ref}', 'Pitch_{ref}', 'Yaw_{ref}', 'X_{ref}', 'Y_{ref}', 'Z_{ref}'};
LegendString.Reference = FullLegendString.Reference(handles.RequestDataFlags.Reference);
% Plot Reference Data
if(any(handles.RequestDataFlags.Reference))
    for i = 1:size(Data.Reference,1)
        Y = fft(Data.Reference(i,:));
        P2 = abs(Y/L);
        P1 = P2(1:floor(L/2)+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;
        loglog(f, P1, 'DisplayName', LegendString.Reference{i})
    end 
end

% Load Error Data
Data.Error = handles.MainData.DataSet.ERROR(handles.RequestDataFlags.Error,:);
FullLegendString.Error = {'P_{Roll}', 'I_{Roll}', 'D_{Roll}', 'FF_{Roll}', 'Out_{Roll}', 'P_{Pitch}', 'I_{Pitch}', 'D_{Pitch}', 'FF_{Pitch}', 'Out_{Pitch}', 'P_{Yaw}', 'I_{Yaw}', 'D_{Yaw}', 'FF_{Yaw}', 'Out_{Yaw}', 'P_{X}', 'I_{X}', 'D_{X}', 'FF_{X}', 'Out_{X}', 'P_{Y}', 'I_{Y}', 'D_{Y}', 'FF_{Y}', 'Out_{Y}', 'P_{Z}', 'I_{Z}', 'D_{Z}', 'FF_{Z}', 'Out_{Z}'};
LegendString.Error = FullLegendString.Error(handles.RequestDataFlags.Error);
% Plot Error Data
if(any(handles.RequestDataFlags.Error))
    for i = 1:size(Data.Error,1)
        Y = fft(Data.Error(i,:));
        P2 = abs(Y/L);
        P1 = P2(1:floor(L/2)+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;
        loglog(f, P1, 'DisplayName', LegendString.Error{i})
    end 
end

% Load State Data
Data.State = handles.MainData.DataSet.STATE(handles.RequestDataFlags.State,:);
FullLegendString.State = {'x', 'y', 'z', '\phi', '\theta', '\psi', 'v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z', 'a_x', 'a_y', 'a_z', '\alpha_x', '\alpha_y', '\alpha_z'};
LegendString.State = FullLegendString.State(handles.RequestDataFlags.State);
% Plot State Data
if(any(handles.RequestDataFlags.State))
    for i = 1:size(Data.State,1)
        Y = fft(Data.State(i,:));
        P2 = abs(Y/L);
        P1 = P2(1:floor(L/2)+1);
        P1(2:end-1) = 2*P1(2:end-1);
        f = Fs*(0:(L/2))/L;
        loglog(f, P1, 'DisplayName', LegendString.State{i})
    end 
end

legend('show','location', 'eastoutside')
grid on
axis tight
xlabel('f (Hz)')
ylabel('|P1(f)|')
title('Single-Sided Amplitude Spectrum')
set(handles.FRFDataAxes, 'XScale', 'Log');
set(handles.FRFDataAxes, 'YScale', 'Log');


% --- Executes on selection change in MainDataListBox.
function MainDataListBox_Callback(hObject, eventdata, handles)
% hObject    handle to MainDataListBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch handles.MainDataListBox.String{get(handles.MainDataListBox,'Value')}
    case 'Raw Sensor' 
        handles.SubDataListBox.String = {'None', 'Magnetometer', 'Accelerometer', 'Gyroscope', 'Barometer', 'GPS', 'Battery', 'Sonar'};
        set(handles.SubDataListBox, 'Max', 1);
        set(handles.SubSubDataListBox, 'Max', 3);
    case 'Filtered Sensor'
        handles.SubDataListBox.String = {'None', 'Magnetometer', 'Accelerometer', 'Gyroscope', 'Barometer', 'GPS', 'Battery', 'Sonar'};
        set(handles.SubDataListBox, 'Max', 1);
        set(handles.SubSubDataListBox, 'Max', 3);
    case 'RC'
        handles.SubDataListBox.String = {'None', 'Roll', 'Pitch', 'Yaw', 'Thrust'};
        set(handles.SubDataListBox, 'Max', 4);
        set(handles.SubSubDataListBox, 'Max', 1);
    case 'Reference'
        handles.SubDataListBox.String = {'None', 'Throttle', 'Roll', 'Pitch', 'Yaw', 'X', 'Y', 'Z'};
        set(handles.SubDataListBox, 'Max', 7);
        set(handles.SubSubDataListBox, 'Max', 1);
    case 'Error'
        handles.SubDataListBox.String = {'None', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
        set(handles.SubDataListBox, 'Max', 1);
        set(handles.SubSubDataListBox, 'Max', 5);
    case 'States'
        handles.SubDataListBox.String = {'None', 'Position Level', 'Velocity Level', 'Acceleration Level'};
        set(handles.SubDataListBox, 'Max', 1);
        set(handles.SubSubDataListBox, 'Max', 6);
    otherwise
        handles.SubDataListBox.String = {};                            
end
handles.SubSubDataListBox.String = {};
handles.SubDataListBox.Value = 1;
guidata(hObject, handles);
% Hints: contents = cellstr(get(hObject,'String')) returns MainDataListBox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from MainDataListBox


% --- Executes during object creation, after setting all properties.
function MainDataListBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MainDataListBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in SubDataListBox.
function SubDataListBox_Callback(hObject, eventdata, handles)
% hObject    handle to SubDataListBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch handles.MainDataListBox.String{get(handles.MainDataListBox,'Value')}
    case 'Raw Sensor' 
        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'Magnetometer'
                handles.SubSubDataListBox.String = {'None', 'mx', 'my', 'mz'};
            case 'Accelerometer'
                handles.SubSubDataListBox.String = {'None', 'ax', 'ay', 'az'};
            case 'Gyroscope'
                handles.SubSubDataListBox.String = {'None', 'gx', 'gy', 'gz'};
            case 'Barometer'
                handles.SubSubDataListBox.String = {'None', 'Pressure'};
            case 'GPS'
                handles.SubSubDataListBox.String = {'None', 'Latitude', 'Longitude', 'Altitude'};
            case 'Battery'
                handles.SubSubDataListBox.String = {'None', 'Voltage', 'Current', 'mAh'};
            case 'Sonar'
                handles.SubSubDataListBox.String = {'None', 'Sonar'};
            otherwise
                handles.SubSubDataListBox.String = {};
        end
    case 'Filtered Sensor'
        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'Magnetometer'
                handles.SubSubDataListBox.String = {'None', 'mx', 'my', 'mz'};
            case 'Accelerometer'
                handles.SubSubDataListBox.String = {'None', 'ax', 'ay', 'az'};
            case 'Gyroscope'
                handles.SubSubDataListBox.String = {'None', 'wx', 'wy', 'wz'};
            case 'Barometer'
                handles.SubSubDataListBox.String = {'None', 'Pressure'};
            case 'GPS'
                handles.SubSubDataListBox.String = {'None', 'Latitude', 'Longitude', 'Altitude'};
            case 'Battery'
                handles.SubSubDataListBox.String = {'None', 'Voltage', 'Current', 'mAh'};
            case 'Sonar'
                handles.SubSubDataListBox.String = {'None', 'Sonar'};
            otherwise
                handles.SubSubDataListBox.String = {};
        end
    case 'RC'
        handles.SubSubDataListBox.String = {};
        Selection = get(handles.SubDataListBox,'Value');
        handles.RequestDataFlags.RC = false(4,1);
        if(Selection(1) ~= 1)
            handles.RequestDataFlags.RC(Selection-1) = true;
        end
        PlotRequestedData(handles)
        
    case 'Reference'
        handles.SubSubDataListBox.String = {};
        Selection = get(handles.SubDataListBox,'Value');
        handles.RequestDataFlags.Reference = false(7,1);
        if(Selection(1) ~= 1)
            handles.RequestDataFlags.Reference(Selection-1) = true;
        end
        PlotRequestedData(handles)
        
    case 'Error'
        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'X'
                handles.SubSubDataListBox.String = {'None', 'P', 'I', 'D', 'FF', 'Output'};
            case 'Y'
                handles.SubSubDataListBox.String = {'None', 'P', 'I', 'D', 'FF', 'Output'};
            case 'Z'
                handles.SubSubDataListBox.String = {'None', 'P', 'I', 'D', 'FF', 'Output'};
            case 'Roll'
                handles.SubSubDataListBox.String = {'None', 'P', 'I', 'D', 'FF', 'Output'};
            case 'Pitch'
                handles.SubSubDataListBox.String = {'None', 'P', 'I', 'D', 'FF', 'Output'};
            case 'Yaw'
                handles.SubSubDataListBox.String = {'None', 'P', 'I', 'D', 'FF', 'Output'};
            otherwise
                handles.SubSubDataListBox.String = {};
        end
    case 'States'
        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'Position Level'
                handles.SubSubDataListBox.String = {'None', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
            case 'Velocity Level'
                handles.SubSubDataListBox.String = {'None', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
            case 'Acceleration Level'
                handles.SubSubDataListBox.String = {'None', 'X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'};
            otherwise
                handles.SubSubDataListBox.String = {};
        end
    otherwise
        handles.SubSubDataListBox.String = {};                            
end
handles.SubSubDataListBox.Value = 1;
guidata(hObject, handles);
% Hints: contents = cellstr(get(hObject,'String')) returns SubDataListBox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SubDataListBox


% --- Executes during object creation, after setting all properties.
function SubDataListBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SubDataListBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in SubSubDataListBox.
function SubSubDataListBox_Callback(hObject, eventdata, handles)
% hObject    handle to SubSubDataListBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% get(handles.SubDataListBox,'Value')
% get(handles.SubSubDataListBox,'Value')

switch handles.MainDataListBox.String{get(handles.MainDataListBox,'Value')}
    case 'Raw Sensor'
        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'Magnetometer'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Raw_Sensor(1:3,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Raw_Sensor(Selection-1) = true;
                end

            case 'Accelerometer'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Raw_Sensor(4:6,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Raw_Sensor(Selection+2) = true;
                end
                
            case 'Gyroscope'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Raw_Sensor(7:9,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Raw_Sensor(Selection+5) = true;
                end
                
            case 'Barometer'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Raw_Sensor(10,:) = false;
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Raw_Sensor(Selection+8) = true;
                end    
                
            case 'GPS'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Raw_Sensor(11:14,:) = false(4,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Raw_Sensor(Selection+9) = true;
                end
                
            case 'Battery'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Raw_Sensor(15:17,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Raw_Sensor(Selection+13) = true;
                end
                
            case 'Sonar'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Raw_Sensor(18,:) = false;
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Raw_Sensor(Selection+16) = true;
                end
                
        end

    case 'Filtered Sensor'
        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'Magnetometer'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Filtered_Sensor(1:3,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Filtered_Sensor(Selection-1) = true;
                end

            case 'Accelerometer'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Filtered_Sensor(4:6,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Filtered_Sensor(Selection+2) = true;
                end
                
            case 'Gyroscope'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Filtered_Sensor(7:9,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Filtered_Sensor(Selection+5) = true;
                end
                
            case 'Barometer'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Filtered_Sensor(10,:) = false;
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Filtered_Sensor(Selection+8) = true;
                end    
                
            case 'GPS'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Filtered_Sensor(11:14,:) = false(4,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Filtered_Sensor(Selection+9) = true;
                end
                
            case 'Battery'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Filtered_Sensor(15:17,:) = false(3,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Filtered_Sensor(Selection+13) = true;
                end
                
            case 'Sonar'
                Selection = get(handles.SubSubDataListBox,'Value');                
                handles.RequestDataFlags.Filtered_Sensor(18,:) = false;
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Filtered_Sensor(Selection+16) = true;
                end
        end
        
    case 'Error'
        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'X'
                Selection = get(handles.SubSubDataListBox,'Value');     
                handles.RequestDataFlags.Error(16:20,:) = false(5,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Error(Selection+14) = true;
                end
                
            case 'Y'
                Selection = get(handles.SubSubDataListBox,'Value');     
                handles.RequestDataFlags.Error(21:25,:) = false(5,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Error(Selection+19) = true;
                end
                
            case 'Z'
                Selection = get(handles.SubSubDataListBox,'Value');     
                handles.RequestDataFlags.Error(26:30,:) = false(5,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Error(Selection+24) = true;
                end
                
            case 'Roll'
                Selection = get(handles.SubSubDataListBox,'Value');     
                handles.RequestDataFlags.Error(1:5,:) = false(5,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Error(Selection-1) = true;
                end
                
            case 'Pitch'
                Selection = get(handles.SubSubDataListBox,'Value');     
                handles.RequestDataFlags.Error(6:10,:) = false(5,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Error(Selection+4) = true;
                end
                
            case 'Yaw'
                Selection = get(handles.SubSubDataListBox,'Value');     
                handles.RequestDataFlags.Error(11:15,:) = false(5,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.Error(Selection+9) = true;
                end
                
        end
        
    case 'States'
%         handles.RequestDataFlags.State

        switch handles.SubDataListBox.String{get(handles.SubDataListBox,'Value')}
            case 'Position Level'
                Selection = get(handles.SubSubDataListBox,'Value'); 
                handles.RequestDataFlags.State(1:6,:) = false(6,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.State(Selection-1) = true;
                end
                
            case 'Velocity Level'
                Selection = get(handles.SubSubDataListBox,'Value'); 
                handles.RequestDataFlags.State(7:12,:) = false(6,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.State(Selection+5) = true;
                end
                
            case 'Acceleration Level'
                Selection = get(handles.SubSubDataListBox,'Value'); 
                handles.RequestDataFlags.State(13:18,:) = false(6,1);
                if(Selection(1) ~= 1)
                    handles.RequestDataFlags.State(Selection+11) = true;
                end
        end
        
end
PlotRequestedData(handles)
guidata(hObject, handles);
% Hints: contents = cellstr(get(hObject,'String')) returns SubSubDataListBox contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SubSubDataListBox


% --- Executes during object creation, after setting all properties.
function SubSubDataListBox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to SubSubDataListBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when ShowRawSensorData is resized.
function ShowRawSensorData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowRawSensorData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% OuterPosition = [X Y Width Height]
handles.FRFDataAxes.Units = 'normalized';
handles.FRFDataAxes.OuterPosition = [0 0 0.8 1];

handles.MainDataListBox.Units = 'normalized';
handles.MainDataListBox.OuterPosition = [0.8 2/3 0.2 1/3];

handles.SubDataListBox.Units = 'normalized';
handles.SubDataListBox.OuterPosition = [0.8 1/3 0.2 1/3];

handles.SubSubDataListBox.Units = 'normalized';
handles.SubSubDataListBox.OuterPosition = [0.8 0 0.2 1/3];
