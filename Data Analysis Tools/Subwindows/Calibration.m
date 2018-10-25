function varargout = Calibration(varargin)
% CALIBRATION MATLAB code for calibration.fig
%      CALIBRATION, by itself, creates a new CALIBRATION or raises the existing
%      singleton*.
%
%      H = CALIBRATION returns the handle to a new CALIBRATION or the handle to
%      the existing singleton*.
%
%      CALIBRATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CALIBRATION.M with the given input arguments.
%
%      CALIBRATION('Property','Value',...) creates a new CALIBRATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ShowRAwSensorData_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ShowRAwSensorData_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help calibration

% Last Modified by GUIDE v2.5 20-Aug-2018 11:49:38

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Calibration_OpeningFcn, ...
    'gui_OutputFcn',  @Calibration_OutputFcn, ...
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


% --- Executes just before calibration is made visible.
function Calibration_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to calibration (see VARARGIN)

% Choose default command line output for calibration
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end
handles.PlotRequest = 'empty';
handles.SensorPopupMenu.String = {'None', 'Magnetometer', 'Accelerometer', 'Gyroscope', 'Barometer', 'GPS', 'Battery', 'Sonar'};
handles.SensorPopupMenu.Value = 1;

handles.CalTypePopupMenu.String = {'None'};
handles.CalTypePopupMenu.Value = 1;

handles.CalPar.TXT = cell(1,10);
handles.CalPar.VAL = cell(1,10);
handles.CalSet.TXT = cell(1,9);
handles.CalSet.VAL = cell(1,9);
% Update handles structure
guidata(hObject, handles);
ShowCalibrationData(handles);
PlotRequestedData(handles);
% UIWAIT makes calibration wait for user response (see UIRESUME)
% uiwait(handles.Calibration);


% --- Outputs from this function are returned to the command line.
function varargout = Calibration_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes during object creation, after setting all properties.
function SensorPopupMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CalTypePopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on selection change in SensorPopupMenu.
function SensorPopupMenu_Callback(hObject, eventdata, handles)
% hObject    handle to SensorPopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch handles.SensorPopupMenu.String{handles.SensorPopupMenu.Value}
    case 'Magnetometer'
        handles.PlotRequest = 'Mag';
        handles.CalTypePopupMenu.String = {'None', 'Hard Iron Corrections', 'Soft Iron Corrections', 'Current Corrections'};
    case 'Accelerometer'
        handles.PlotRequest = 'Acc';
        handles.CalTypePopupMenu.String = {'None', 'Bias', 'Scale', 'A Peak2Peak'};
    case 'Gyroscope'
        handles.PlotRequest = 'Gyro';
        handles.CalTypePopupMenu.String = {'None', 'Bias', 'X Scale', 'Y Scale', 'Z Scale'};
    case 'Barometer'
        handles.PlotRequest = 'Baro';
        handles.CalTypePopupMenu.String = {'None', 'Calibrate'};
    case 'GPS'
        handles.PlotRequest = 'GPS';
        handles.CalTypePopupMenu.String = {'None', 'Latitude', 'Longitude', 'Altitude'};
    case 'Battery'
        handles.PlotRequest = 'Bat';
        handles.CalTypePopupMenu.String = {'None'};
    case 'Sonar'
        handles.PlotRequest = 'Sonar';
        handles.CalTypePopupMenu.String = {'None', 'Calibrate'};
    otherwise
        handles.PlotRequest = 'None';
        handles.CalTypePopupMenu.String = {'None'};
end
handles.CalTypePopupMenu.Value = 1;
guidata(hObject, handles);
PlotRequestedData(handles)
% Hints: contents = cellstr(get(hObject,'String')) returns SensorPopupMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from SensorPopupMenu


% --- Executes during object creation, after setting all properties.
function CalTypePopupMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to CalTypePopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in CalTypePopupMenu.
function CalTypePopupMenu_Callback(hObject, eventdata, handles)
% hObject    handle to CalTypePopupMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Empty Axes
handles.CalPar.TXT = cell(1,10);
handles.CalPar.VAL = cell(1,10);
handles.CalSet.TXT = cell(1,9);
PlotAxes = {'axes2', 'axes3', 'axes4', 'axes5'};
Time = handles.MainData.DataSet.CLOCK(end,:);
Color = [1 0 0; 0 1 0; 0 0 1; 0 0 0];
for i = 1:4
    cla(handles.(PlotAxes{i}), 'reset')
end

switch handles.CalTypePopupMenu.String{handles.CalTypePopupMenu.Value}
    case 'Calibrate'
        % Calibrate Sonar and/or Barometer
        handles.CalSet.TXT(1:9) = {'Start 1','End 1','Heigth 1', 'Start 2','End 2','Heigth 2','Start 3','End 3','Heigth 3'};
        handles.SubCalibrate = 'Calibrate';
    case 'Hard Iron Corrections'
        % Calculate Hard Iron Corrections
        handles.MainData.Calibration.EllFit.X = handles.MainData.DataSet.RAW_SENSOR_DATA(1:3,:)';
        handles.MainData.Calibration.EllFit.equals = '';
        handles.MainData.Calibration.EllFit.Type = 'Bias';
        ellipsoid_fit(hObject, eventdata, handles);
        handles = guidata(hObject);
        
        V = handles.MainData.Calibration.EllFit.V;
        handles.MainData.Calibration.Mag.V = V;
        handles.CalPar.TXT(1:3) = {'Vx', 'Vy', 'Vz'};
        handles.CalPar.VAL(1:3) = num2cell([V']);
    case 'Soft Iron Corrections'
        % Calculate Soft Iron Corrections
        handles.MainData.Calibration.EllFit.X = handles.MainData.DataSet.RAW_SENSOR_DATA(1:3,:)';
        handles.MainData.Calibration.EllFit.equals = '';
        handles.MainData.Calibration.EllFit.Type = 'Scale';
        ellipsoid_fit(hObject, eventdata, handles);
        handles = guidata(hObject);
        
        evecs = handles.MainData.Calibration.EllFit.evecs;
        radii = handles.MainData.Calibration.EllFit.radii;
        
        W = mean(radii)*evecs*diag(1./radii)*evecs';
        handles.MainData.Calibration.Mag.W = W;
        handles.CalPar.TXT(1:10) = {'Wxx', 'Wxy', 'Wxz', 'Wyx', 'Wyy', 'Wyz', 'Wzx', 'Wzy', 'Wzz', 'B'};
        handles.CalPar.VAL(1:10) = num2cell([reshape(W,1,9), mean(radii)]);
    case 'Current Corrections'
        % Calculate Current Corrections
        X = [handles.MainData.DataSet.RAW_SENSOR_DATA(16,:)' ones(size(handles.MainData.DataSet.RAW_SENSOR_DATA(16,:)'))];
        Y = handles.MainData.DataSet.RAW_SENSOR_DATA(1:3,:)';
        C = (inv(X'*X)*X'*Y)';
        
        handles.CalPar.TXT(1:6) = {'ax', 'ay', 'az', 'bx', 'by', 'bz'};
        handles.CalPar.VAL(1:6) = num2cell(reshape(C,1,6));
        handles.MainData.Calibration.Mag.C = C(:,1);
    case 'Bias'
        switch handles.SensorPopupMenu.String{handles.SensorPopupMenu.Value}
            case 'Accelerometer'
                % Calculate Bias
                handles.MainData.Calibration.EllFit.X = handles.MainData.DataSet.RAW_SENSOR_DATA(4:6,:)';
                handles.MainData.Calibration.EllFit.equals = '';
                handles.MainData.Calibration.EllFit.Type = 'Bias';
                ellipsoid_fit(hObject, eventdata, handles);
                handles = guidata(hObject);
                
                V = handles.MainData.Calibration.EllFit.V;
                handles.MainData.Calibration.Acc.V = V;
                handles.CalPar.TXT(1:3) = {'Vx', 'Vy', 'Vz'};
                handles.CalPar.VAL(1:3) = num2cell([V']);
            case 'Gyroscope'
                % Calculate Gyro Offset
                handles.CalSet.TXT(1:2) = {'Start','End'};
                handles.SubCalibrate = 'Bias';
            case 'Barometer'
                % Calculate Barometer Offset
                handles.CalSet.TXT(1:2) = {'Start','End'};
                handles.SubCalibrate = 'Bias';
            otherwise
        end
    case 'Scale'
        switch handles.SensorPopupMenu.String{handles.SensorPopupMenu.Value}
            case 'Accelerometer'
                % Calculate Accelerometer Scale
                handles.MainData.Calibration.EllFit.X = handles.MainData.DataSet.RAW_SENSOR_DATA(4:6,:)';
                handles.MainData.Calibration.EllFit.equals = '';
                handles.MainData.Calibration.EllFit.Type = 'Scale';
                ellipsoid_fit(hObject, eventdata, handles);
                handles = guidata(hObject);
                
                evecs = handles.MainData.Calibration.EllFit.evecs;
                radii = handles.MainData.Calibration.EllFit.radii;
                W = mean(radii)*evecs*diag(1./radii)*evecs';
                
                handles.MainData.Calibration.Acc.W = W;
                handles.CalPar.TXT(1:10) = {'Wxx', 'Wxy', 'Wxz', 'Wyx', 'Wyy', 'Wyz', 'Wzx', 'Wzy', 'Wzz', 'A'};
                handles.CalPar.VAL(1:10) = num2cell([reshape(W,1,9), mean(radii)]);
            case 'Barometer'
                % Calculate Gyro Scale
                handles.CalSet.TXT(1:4) = {'Start (t)','End (t)', 'Start (Dist)', 'End (Dist)'};
                handles.SubCalibrate = 'Scale';
            otherwise
        end
    case 'A Peak2Peak'
        % Calculate Accelerometer Peak2Peak
        handles.CalSet.TXT(1:2) = {'Stationary Start','Stationary End'};
    case 'X Scale'
        % Calculate Gyro Scale
        handles.CalSet.TXT(1:5) = {'0 Start','0 End','Rounds Start','Rounds End', 'N Rounds'};
        handles.SubCalibrate = 'X Scale';
    case 'Y Scale'
        % Calculate Gyro Scale
        handles.CalSet.TXT(1:5) = {'0 Start','0 End','Rounds Start','Rounds End', 'N Rounds'};
        handles.SubCalibrate = 'Y Scale';
    case 'Z Scale'
        % Calculate Gyro Scale
        handles.CalSet.TXT(1:5) = {'0 Start','0 End','Rounds Start','Rounds End', 'N Rounds'};
        handles.SubCalibrate = 'Z Scale';
    case 'Latitude'
        % GPS Latitude
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.GPS.V;
        W = handles.MainData.Calibration.GPS.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';

        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(11,:);
        Data_2 = Data_1 - V(1);
        Data_3 = inv(W(1,1))*Data_2;
        
        YData = [Data_1; Data_2; Data_3];
        n = size(Data_1,1);
        
        TitleStr = {'Raw GPS Data','GPS Data Bias Removed','GPS Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'GPS Data';
        LegendStr = {'Latitude'};

        for j = 1:4
            if j < 4
                for i = 1:n
                    plot(handles.(PlotAxes{j}), Time, YData(i+(j-1)*n,:),'color',Color(1,:))
                    hold(handles.(PlotAxes{j}), 'on')
                end
                legend(handles.(PlotAxes{j}),LegendStr)
            end
            xlabel(handles.(PlotAxes{j}),XlabelStr)
            ylabel(handles.(PlotAxes{j}),YlabelStr)
            title(handles.(PlotAxes{j}), TitleStr{j})
            grid(handles.(PlotAxes{j}), 'on')
            axis(handles.(PlotAxes{j}), 'tight')
            
        end

        % Calculate Z offset and scale
        handles.CalSet.TXT(1:9) = {'t start 1', 't end 1', 'Position 1', 't start 2', 't end 2', 'Position 2', 't start 3', 't end 3', 'Position 3'};
        handles.SubCalibrate = 'Latitude';
    case 'Longitude'
        % GPS Longitude
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.GPS.V;
        W = handles.MainData.Calibration.GPS.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';

        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(12,:);
        Data_2 = Data_1 - V(2);
        Data_3 = inv(W(2,2))*Data_2;
        
        YData = [Data_1; Data_2; Data_3];
        n = size(Data_1,1);
        
        TitleStr = {'Raw GPS Data','GPS Data Bias Removed','GPS Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'GPS Data';
        LegendStr = {'Longitude'};

        for j = 1:4
            if j < 4
                for i = 1:n
                    plot(handles.(PlotAxes{j}), Time, YData(i+(j-1)*n,:),'color',Color(1,:))
                    hold(handles.(PlotAxes{j}), 'on')
                end
                legend(handles.(PlotAxes{j}),LegendStr)
            end
            xlabel(handles.(PlotAxes{j}),XlabelStr)
            ylabel(handles.(PlotAxes{j}),YlabelStr)
            title(handles.(PlotAxes{j}), TitleStr{j})
            grid(handles.(PlotAxes{j}), 'on')
            axis(handles.(PlotAxes{j}), 'tight')
            
        end
        
        % Calculate Z offset and scale
        handles.CalSet.TXT(1:9) = {'t start 1', 't end 1', 'Position 1', 't start 2', 't end 2', 'Position 2', 't start 3', 't end 3', 'Position 3'};
        handles.SubCalibrate = 'Longitude';
    case 'Altitude'
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.GPS.V;
        W = handles.MainData.Calibration.GPS.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';

        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(13,:);
        Data_2 = Data_1 - V(3);
        Data_3 = inv(W(3,3))*Data_2;
        
        YData = [Data_1; Data_2; Data_3];
        n = size(Data_1,1);
        
        TitleStr = {'Raw GPS Data','GPS Data Bias Removed','GPS Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'GPS Data';
        LegendStr = {'Altitude'};

        for j = 1:4
            if j < 4
                for i = 1:n
                    plot(handles.(PlotAxes{j}), Time, YData(i+(j-1)*n,:),'color',Color(3,:))
                    hold(handles.(PlotAxes{j}), 'on')
                end
                legend(handles.(PlotAxes{j}),LegendStr)
            end
            xlabel(handles.(PlotAxes{j}),XlabelStr)
            ylabel(handles.(PlotAxes{j}),YlabelStr)
            title(handles.(PlotAxes{j}), TitleStr{j})
            grid(handles.(PlotAxes{j}), 'on')
            axis(handles.(PlotAxes{j}), 'tight')
            
        end
        % Calculate Z offset and scale
        handles.CalSet.TXT(1:9) = {'t start 1', 't end 1', 'Position 1', 't start 2', 't end 2', 'Position 2', 't start 3', 't end 3', 'Position 3'};
        handles.SubCalibrate = 'Altitude';
    otherwise
end

% Show results
guidata(hObject, handles);
h = findobj('Tag', 'MainGUI');
guidata(h,handles.MainData);
PlotRequestedData(handles)
GetCalibrationSettings(handles)
ShowCalibrationData(handles)
% Hints: contents = cellstr(get(hObject,'String')) returns CalTypePopupMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from CalTypePopupMenu


function PlotRequestedData(handles)
Time = handles.MainData.DataSet.CLOCK(end,:);
PlotFlag = false;

switch handles.PlotRequest
    case 'Mag'
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.Mag.V;
        B = handles.MainData.Calibration.Mag.B;
        W = handles.MainData.Calibration.Mag.W;
        C = handles.MainData.Calibration.Mag.C;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';
        
        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(1:3,:);
        Data_1(4,:) = sqrt(sum(Data_1.^2));
        % Hard iron corrections
        Data_2 = Data_1(1:3,:) - V;
        Data_2(4,:) = sqrt(sum(Data_2.^2));
        % Soft Iron Corrections
        Data_3 = W*Data_2(1:3,:);
        Data_3(4,:) = sqrt(sum(Data_3.^2));
        % Current Corrections
        Cur = handles.MainData.DataSet.RAW_SENSOR_DATA(16,:);
        Data_4 = Data_3(1:3,:) - C(:,1)*Cur;
        Data_4(4,:) = sqrt(sum(Data_4.^2));
        YData = [Data_1; Data_2; Data_3; Data_4];
        
        TitleStr = {'Raw Magnetometer Data', 'Hard Iron Corrections', 'Soft Iron Corrections', 'Current Corrections'};
        XlabelStr = 'Time';
        YlabelStr = 'Mag Data';
        LegendStr = {'M_x', 'M_y', 'M_z', 'M_m'};
        PlotFlag = true;
    case 'Acc'
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.Acc.V;
        W = handles.MainData.Calibration.Acc.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';
        
        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(4:6,:);
        Data_1(4,:) = sqrt(sum(Data_1(1:3,:).^2));
        Data_2 = Data_1(1:3,:) - V;
        Data_2(4,:) = sqrt(sum(Data_2(1:3,:).^2));
        Data_3 = W*Data_2(1:3,:);
        Data_3(4,:) = sqrt(sum(Data_3(1:3,:).^2));
        YData = [Data_1; Data_2; Data_3];
        
        TitleStr = {'Raw Accelerometer Data','Accelerometer Data Bias Removed','Accelerometer Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'Acc Data';
        LegendStr = {'a_x', 'a_y', 'a_z', 'a_m'};
        PlotFlag = true;
    case 'Gyro'
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.Gyro.V;
        W = handles.MainData.Calibration.Gyro.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';
        
        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(7:9,:);
        Data_2 = Data_1 - V;
        Data_3 = W*Data_2;
        for i = 1:3
            Data_4(i,:) = cumtrapz(Time, Data_3(i,:));
        end
        YData = [Data_1; Data_2; Data_3; Data_4];
        
        TitleStr = {'Raw Gyroscope Data','Gyroscope Data Bias Removed','Gyroscope Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'Gyro Data';
        LegendStr = {'\phi', '\theta', '\psi'};
        PlotFlag = true;
    case 'Baro'
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.Baro.V;
        W = handles.MainData.Calibration.Baro.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';
        
        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(10,:);
        Data_2 = Data_1 - V;
        Data_3 = inv(W)*Data_2;
        YData = [Data_1; Data_2; Data_3];
        
        TitleStr = {'Raw Barometer Data','Barometer Data Bias Removed','Barometer Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'Pressure Data';
        LegendStr = {'p'};
        PlotFlag = true;
    case 'GPS'
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.GPS.V;
        W = handles.MainData.Calibration.GPS.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';
        
        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(11:13,:);
        Data_2 = Data_1 - V;
        Data_3 = inv(W)*Data_2;
        
        YData = [Data_1; Data_2; Data_3];
        
        TitleStr = {'Raw GPS Data','GPS Data Bias Removed','GPS Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'GPS Data';
        LegendStr = {'Latitude', 'Longitude', 'Altitude'};
        PlotFlag = true;
    case 'Bat'
        % Set initial Calibration parameters
        V = handles.MainData.Calibration.Bat.V;
        W = handles.MainData.Calibration.Bat.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'off';
        
        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA(15:17,:);
        Data_2 = Data_1 - V;
        Data_3 = inv(W)*Data_2;
        
        YData = [Data_1; Data_2; Data_3];
        
        TitleStr = {'Raw Battery Data','Battery Data Bias Removed','Battery Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'Battery Data';
        LegendStr = {'Voltage', 'Current', 'mAh'};
        PlotFlag = true;
    case 'Sonar'
         % Set initial Calibration parameters
        V = handles.MainData.Calibration.Sonar.V;
        W = handles.MainData.Calibration.Sonar.W;
        % Calibration Button Settings
        handles.CalTypeBtnGroup.Visible = 'on';
        
        % Raw Sensor Data
        Data_1 = handles.MainData.DataSet.RAW_SENSOR_DATA([18],:);
        Data_2 = Data_1 - V;
        Data_3 = inv([W])*Data_2;        
        YData = [Data_1; Data_2; Data_3];
        
        TitleStr = {'Raw Altitude Data','Altitude Data Bias Removed','Altitude Data Scaled',''};
        XlabelStr = 'Time';
        YlabelStr = 'Altitude Data';
        LegendStr = {'Sonar'};
        PlotFlag = true;
    otherwise
        PlotFlag = false;
end

%% Plot Data
PlotAxes = {'axes2', 'axes3', 'axes4', 'axes5'};
for i = 1:4
    cla(handles.(PlotAxes{i}), 'reset')
end
handles.CalPar.TXT = cell(1,10);
handles.CalPar.VAL = cell(1,10);
handles.CalSet.TXT = cell(1,9);
Color = [1 0 0; 0 1 0; 0 0 1; 0 0 0];
AxesList = [];
linkaxes([handles.axes2 handles.axes3 handles.axes4 handles.axes5], 'off');
if(PlotFlag)
    for j = 1:4
        if j < (size(YData, 1)/ size(Data_1, 1) + 1)
            for i = 1:size(Data_1, 1)
                plot(handles.(PlotAxes{j}), Time, YData(i+(j-1)*size(Data_1, 1),:),'color',Color(i,:))
                hold(handles.(PlotAxes{j}), 'on')
            end
            legend(handles.(PlotAxes{j}),LegendStr)
            AxesList = [AxesList handles.(PlotAxes{j})];
        end
        xlabel(handles.(PlotAxes{j}),XlabelStr)
        ylabel(handles.(PlotAxes{j}),YlabelStr)
        title(handles.(PlotAxes{j}), TitleStr{j})
        grid(handles.(PlotAxes{j}), 'on')
        axis(handles.(PlotAxes{j}), 'tight')
    end
else
    TitleStr = {'', '', '', ''};
    XlabelStr = 'Time';
    YlabelStr = 'Data';
    for j = 1:4
        xlabel(handles.(PlotAxes{j}),XlabelStr)
        ylabel(handles.(PlotAxes{j}),YlabelStr)
        legend(handles.(PlotAxes{j}),'off')
        title(handles.(PlotAxes{j}), TitleStr{j})
        grid(handles.(PlotAxes{j}), 'on')
        axis(handles.(PlotAxes{j}), 'tight')
        AxesList = [AxesList handles.(PlotAxes{j})];
    end
    handles.CalTypeBtnGroup.Visible = 'off';
    handles.CalSettingsPanel.Visible = 'off';
end
linkaxes(AxesList, 'x');
handles.CalPar.TXT = cell(1,10);
handles.CalPar.VAL = cell(1,10);
GetCalibrationSettings(handles)
ShowCalibrationData(handles)


% --- Executes on button press in CalSetCalibrationBtn.
function CalSetCalibrationBtn_Callback(hObject, eventdata, handles)
% hObject    handle to CalSetCalibrationBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SettingsList = {'Cal_Set_Val_1', 'Cal_Set_Val_2', 'Cal_Set_Val_3', 'Cal_Set_Val_4', 'Cal_Set_Val_5', 'Cal_Set_Val_6', 'Cal_Set_Val_7', 'Cal_Set_Val_8', 'Cal_Set_Val_9'};
switch handles.PlotRequest
    case 'Gyro'
        switch handles.SubCalibrate
            case 'Bias'
                for i = 1:sum(~cellfun(@isempty, handles.CalSet.TXT))
                    Frames(i) = str2num(handles.(SettingsList{i}).String);
                end
                Clock = handles.MainData.DataSet.CLOCK(end,:);
                Data = handles.MainData.DataSet.RAW_SENSOR_DATA(7:9,:);
                ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));

                V = mean(Data(:,ClockIndex(1,:)), 2);
                handles.MainData.Calibration.Gyro.V = V;
                W = handles.MainData.Calibration.Gyro.W;
                for i = 1:length(Clock)
                    Clean_Data(:,i) = W*(Data(:,i) - V);
                end
                thg = max(abs(Clean_Data(:,ClockIndex)),[],2);
                handles.CalPar.TXT(1:6) = {'Vx', 'Vy', 'Vz', 'thgx' , 'thgy', 'thgz'};
                handles.CalPar.VAL(1:6) = num2cell([V, thg]);
                
            case 'X Scale'
                V = handles.MainData.Calibration.Gyro.V;
                for i = 1:sum(~cellfun(@isempty, handles.CalSet.TXT))
                    Frames(i) = str2num(handles.(SettingsList{i}).String);
                end
                Clock = handles.MainData.DataSet.CLOCK(end,:);
                Data = cumtrapz(Clock, handles.MainData.DataSet.RAW_SENSOR_DATA(7,:) - V(1));
                ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));
                ClockIndex(2,:) = ((Clock > Frames(3)) & (Clock < Frames(4)));
                
                Scale = (Frames(5)*2*pi)/(mean(Data(:, ClockIndex(2,:))) - mean(Data(:, ClockIndex(1,:))));
                handles.MainData.Calibration.Gyro.W(1,1) = Scale;
                handles.CalPar.TXT(1) = {'Wxx'};
                handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.Gyro.W(1,1));
                
            case 'Y Scale'
                V = handles.MainData.Calibration.Gyro.V;
                for i = 1:sum(~cellfun(@isempty, handles.CalSet.TXT))
                    Frames(i) = str2num(handles.(SettingsList{i}).String);
                end
                Clock = handles.MainData.DataSet.CLOCK(end,:);
                Data = cumtrapz(Clock, handles.MainData.DataSet.RAW_SENSOR_DATA(8,:) - V(2));
                ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));
                ClockIndex(2,:) = ((Clock > Frames(3)) & (Clock < Frames(4)));
                
                Scale = (Frames(5)*2*pi)/(mean(Data(:, ClockIndex(2,:))) - mean(Data(:, ClockIndex(1,:))));
                handles.MainData.Calibration.Gyro.W(2,2) = Scale;
                handles.CalPar.TXT(1) = {'Wyy'};
                handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.Gyro.W(2,2));
                
            case 'Z Scale'
                V = handles.MainData.Calibration.Gyro.V;
                for i = 1:sum(~cellfun(@isempty, handles.CalSet.TXT))
                    Frames(i) = str2num(handles.(SettingsList{i}).String);
                end
                Clock = handles.MainData.DataSet.CLOCK(end,:);
                Data = cumtrapz(Clock, handles.MainData.DataSet.RAW_SENSOR_DATA(9,:) - V(3));
                ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));
                ClockIndex(2,:) = ((Clock > Frames(3)) & (Clock < Frames(4)));
                
                Scale = (Frames(5)*2*pi)/(mean(Data(:, ClockIndex(2,:))) - mean(Data(:, ClockIndex(1,:))));
                handles.MainData.Calibration.Gyro.W(3,3) = Scale;
                handles.CalPar.TXT(1) = {'Wzz'};
                handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.Gyro.W(3,3));
        end
    case 'Acc'
        for i = 1:sum(~cellfun(@isempty, handles.CalSet.TXT))
            Frames(i) = str2num(handles.(SettingsList{i}).String);
        end
                
        V = handles.MainData.Calibration.Acc.V;
        W = handles.MainData.Calibration.Acc.W;
        Clock = handles.MainData.DataSet.CLOCK(end,:);
        for i = 1:length(Clock)
            Data(i,:) = W*(handles.MainData.DataSet.RAW_SENSOR_DATA(4:6,i) - V);
        end
        ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));
        tha = max(abs(diff(Data(ClockIndex,:))));
        
        handles.CalPar.TXT(1:3) = {'app x', 'app y', 'app z'};
        handles.CalPar.VAL(1:3) = num2cell(tha);
        
    case 'Baro'
        for i  = 1:length(SettingsList)
            Frames(i) = str2num(handles.(SettingsList{i}).String);
        end
        Start   = Frames([1 4 7]);
        End     = Frames([2 5 8]);
        Heigth  = Frames([3 6 9]);
        
        Clock = handles.MainData.DataSet.CLOCK(end,:);
        Sonar = handles.MainData.DataSet.RAW_SENSOR_DATA(10,:);
        M = [];
        for i = 1:3
            if((Start(i) ~= 0) | (End(i) ~= 0) | (Heigth(i) ~= 0))
                M = [M; mean(Sonar(1,[Clock >= Start(i) & Clock <= End(i)]))];
            end
        end
        
        %% Least squares fit
        X = [M ones(size(M))];
        Y = Heigth';
        B  = inv(X'*X) * X' * Y
        
        handles.CalPar.TXT(1) = {'Offset'};
        handles.CalPar.TXT(2) = {'Scale'};
        
        handles.MainData.Calibration.Baro.W = 1/B(1);
        handles.MainData.Calibration.Baro.V = -B(2)/B(1);
        
        handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.Baro.V);
        handles.CalPar.VAL(2) = num2cell(handles.MainData.Calibration.Baro.W);
        
        
    case 'GPS'
        switch handles.SubCalibrate
            case 'Latitude'
                for i  = 1:length(SettingsList)
                    Frames(i) = str2num(handles.(SettingsList{i}).String);
                end
                
                Clock = handles.MainData.DataSet.CLOCK(end,:);
                Data = handles.MainData.DataSet.RAW_SENSOR_DATA(10,:);
                ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));
                ClockIndex(2,:) = ((Clock > Frames(4)) & (Clock < Frames(5)));
                ClockIndex(3,:) = ((Clock > Frames(7)) & (Clock < Frames(8)));
                
                M = [mean(Data(:,ClockIndex(1,:))); mean(Data(:,ClockIndex(2,:))); mean(Data(:,ClockIndex(3,:)))];
                Y = [Frames([3 6 9])]';
                X = [M M.^0];
                B = inv(X' * X) * X' * Y;
                handles.MainData.Calibration.GPS.V(1) = 1/B(2);
                handles.MainData.Calibration.GPS.W(1,1) = 1/B(1);
                handles.CalPar.TXT(1) = {'Offset'};
                handles.CalPar.TXT(2) = {'Scale'};
                handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.GPS.V(1));
                handles.CalPar.VAL(2) = num2cell(handles.MainData.Calibration.GPS.W(1,1));
                
            case 'Longitude'
                for i  = 1:length(SettingsList)
                    Frames(i) = str2num(handles.(SettingsList{i}).String);
                end
                
                Clock = handles.MainData.DataSet.CLOCK(end,:);
                Data = handles.MainData.DataSet.RAW_SENSOR_DATA(11,:);
                ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));
                ClockIndex(2,:) = ((Clock > Frames(4)) & (Clock < Frames(5)));
                ClockIndex(3,:) = ((Clock > Frames(7)) & (Clock < Frames(8)));
                
                M = [mean(Data(:,ClockIndex(1,:))); mean(Data(:,ClockIndex(2,:))); mean(Data(:,ClockIndex(3,:)))];
                Y = [Frames([3 6 9])]';
                X = [M M.^0];
                B = inv(X' * X) * X' * Y;
                handles.MainData.Calibration.GPS.V(2) = 1/B(2);
                handles.MainData.Calibration.GPS.W(2,2) = 1/B(1);
                handles.CalPar.TXT(1) = {'Offset'};
                handles.CalPar.TXT(2) = {'Scale'};
                handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.GPS.V(2));
                handles.CalPar.VAL(2) = num2cell(handles.MainData.Calibration.GPS.W(2,2));
                
            case 'Altitude'
                for i  = 1:length(SettingsList)
                    Frames(i) = str2num(handles.(SettingsList{i}).String);
                end

                Clock = handles.MainData.DataSet.CLOCK(end,:);
                Data = handles.MainData.DataSet.RAW_SENSOR_DATA(12,:);
                ClockIndex(1,:) = ((Clock > Frames(1)) & (Clock < Frames(2)));
                ClockIndex(2,:) = ((Clock > Frames(4)) & (Clock < Frames(5)));
                ClockIndex(3,:) = ((Clock > Frames(7)) & (Clock < Frames(8)));
                
                M = [mean(Data(:,ClockIndex(1,:))); mean(Data(:,ClockIndex(2,:))); mean(Data(:,ClockIndex(3,:)))];
                Y = [Frames([3 6 9])]';
                X = [M M.^0];
                B = inv(X' * X) * X' * Y;
                handles.MainData.Calibration.GPS.V(3) = 1/B(2);
                handles.MainData.Calibration.GPS.W(3,3) = 1/B(1);
                handles.CalPar.TXT(1) = {'Offset'};
                handles.CalPar.TXT(2) = {'Scale'};
                handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.GPS.V(3));
                handles.CalPar.VAL(2) = num2cell(handles.MainData.Calibration.GPS.W(3,3));
                
        end    
    case 'Sonar'
        for i  = 1:length(SettingsList)
            Frames(i) = str2num(handles.(SettingsList{i}).String);
        end
        Start   = Frames([1 4 7]);
        End     = Frames([2 5 8]);
        Heigth  = Frames([3 6 9]);
        
        Clock = handles.MainData.DataSet.CLOCK(end,:);
        Sonar = handles.MainData.DataSet.RAW_SENSOR_DATA(18,:);
        M = [];
        for i = 1:3
            if((Start(i) ~= 0) | (End(i) ~= 0) | (Heigth(i) ~= 0))
                M = [M; mean(Sonar(1,[Clock >= Start(i) & Clock <= End(i)]))];
            end
        end
        
        %% Least squares fit
        X = [M ones(size(M))];
        Y = Heigth';
        B  = inv(X'*X) * X' * Y;
        
        handles.CalPar.TXT(1) = {'Offset'};
        handles.CalPar.TXT(2) = {'Scale'};
        
        handles.MainData.Calibration.Sonar.W = 1/B(1);
        handles.MainData.Calibration.Sonar.V = -B(2)/B(1);
        
        handles.CalPar.VAL(1) = num2cell(handles.MainData.Calibration.Sonar.V);
        handles.CalPar.VAL(2) = num2cell(handles.MainData.Calibration.Sonar.W);
        
end
guidata(hObject, handles);
h = findobj('Tag', 'MainGUI');
guidata(h,handles.MainData);
PlotRequestedData(handles)
GetCalibrationSettings(handles)
ShowCalibrationData(handles)
% Hint: get(hObject,'Value') returns toggle state of CalSetCalibrationBtn

function ShowCalibrationData(handles)
FieldNames1 = {'Cal_Par_1','Cal_Par_2','Cal_Par_3','Cal_Par_4','Cal_Par_5','Cal_Par_6','Cal_Par_7','Cal_Par_8','Cal_Par_9','Cal_Par_10'};
FieldNames2 = {'Cal_Par_Val_1','Cal_Par_Val_2','Cal_Par_Val_3','Cal_Par_Val_4','Cal_Par_Val_5','Cal_Par_Val_6','Cal_Par_Val_7','Cal_Par_Val_8','Cal_Par_Val_9','Cal_Par_Val_10'};

if(any(~cellfun(@isempty, handles.CalPar.TXT)))
    % Something needs to be shown
    handles.CalResultsPanel.Visible = 'on';

    for i = 1:length(FieldNames1)
        handles.(FieldNames1{i}).Visible = 'off';
        handles.(FieldNames2{i}).Visible = 'off';

        if(~isempty(handles.CalPar.TXT{i}) | ~isempty(handles.CalPar.VAL{i}))
            handles.(FieldNames1{i}).Visible = 'on';
            handles.(FieldNames2{i}).Visible = 'on';
            
            handles.(FieldNames1{i}).String = handles.CalPar.TXT{i};
            handles.(FieldNames2{i}).String = handles.CalPar.VAL{i};
        end
    end
else
    % Nothing needs to be shown
    handles.CalResultsPanel.Visible = 'off';
end

function GetCalibrationSettings(handles)
FieldNames1 = {'Cal_Set_1','Cal_Set_2','Cal_Set_3','Cal_Set_4','Cal_Set_5','Cal_Set_6','Cal_Set_7','Cal_Set_8','Cal_Set_9'};
FieldNames2 = {'Cal_Set_Val_1','Cal_Set_Val_2','Cal_Set_Val_3','Cal_Set_Val_4','Cal_Set_Val_5','Cal_Set_Val_6','Cal_Set_Val_7','Cal_Set_Val_8','Cal_Set_Val_9'};

if(any(~cellfun(@isempty, handles.CalSet.TXT)))
    % Something needs to be shown
    handles.CalSettingsPanel.Visible = 'on';
    
    for i = 1:length(FieldNames1)
        handles.(FieldNames1{i}).Visible = 'off';
        handles.(FieldNames2{i}).Visible = 'off';
        
        if(~isempty(handles.CalSet.TXT{i}))
            handles.(FieldNames1{i}).Visible = 'on';
            handles.(FieldNames2{i}).Visible = 'on';
            handles.(FieldNames1{i}).String = handles.CalSet.TXT{i};
        end
    end
else
    % Nothing needs to be shown
    handles.CalSettingsPanel.Visible = 'off';
end

function Cal_Par_Val_10_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_10 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_10 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Cal_Par_Val_9_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_9 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_9 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_8_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_8 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_8 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_7_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_7 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_7 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_6_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_6 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_6 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_5_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_5 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_5 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_4_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_4 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_4 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_3_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_3 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_3 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_2_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_2 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_2 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Par_Val_1_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Par_Val_1 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Par_Val_1 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Par_Val_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Par_Val_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function Cal_Set_Val_9_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_9 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_9 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_8_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_8 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_8 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_7_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_7 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_7 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_6_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_6 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_6 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_5_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_5 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_5 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_4_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_4 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_4 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_3_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_3 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_3 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_2_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_2 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_2 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Cal_Set_Val_1_Callback(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Cal_Set_Val_1 as text
%        str2double(get(hObject,'String')) returns contents of Cal_Set_Val_1 as a double


% --- Executes during object creation, after setting all properties.
function Cal_Set_Val_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Cal_Set_Val_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on request
function ellipsoid_fit(hObject, eventdata, handles)
X = handles.MainData.Calibration.EllFit.X;
equals = handles.MainData.Calibration.EllFit.equals;

%
% Fit an ellispoid/sphere/paraboloid/hyperboloid to a set of xyz data points:
%
%   [center, radii, evecs, pars ] = ellipsoid_fit( X )
%   [center, radii, evecs, pars ] = ellipsoid_fit( [x y z] );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 1 );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 2, 'xz' );
%   [center, radii, evecs, pars ] = ellipsoid_fit( X, 3 );
%
% Parameters:
% * X, [x y z]   - Cartesian data, n x 3 matrix or three n x 1 vectors
% * flag         - '' or empty fits an arbitrary ellipsoid (default),
%                - 'xy' fits a spheroid with x- and y- radii equal
%                - 'xz' fits a spheroid with x- and z- radii equal
%                - 'xyz' fits a sphere
%                - '0' fits an ellipsoid with its axes aligned along [x y z] axes
%                - '0xy' the same with x- and y- radii equal
%                - '0xz' the same with x- and z- radii equal
%
% Output:
% * center    -  ellispoid or other conic center coordinates [xc; yc; zc]
% * radii     -  ellipsoid or other conic radii [a; b; c]
% * evecs     -  the radii directions as columns of the 3x3 matrix
% * v         -  the 10 parameters describing the ellipsoid / conic algebraically: 
%                Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz + J = 0
% * chi2      -  residual sum of squared errors (chi^2), this chi2 is in the 
%                coordinate frame in which the ellipsoid is a unit sphere.
%
% Author:
% Yury Petrov, Oculus VR
% Date:
% September, 2015
%

narginchk( 1, 3 ) ;  % check input arguments
if nargin == 1
    equals = ''; % no constraints by default
end
    
if size( X, 2 ) ~= 3
    error( 'Input data must have three columns!' );
else
    x = X( :, 1 );
    y = X( :, 2 );
    z = X( :, 3 );
end

% need nine or more data points
if length( x ) < 9 && strcmp( equals, '' ) 
   error( 'Must have at least 9 points to fit a unique ellipsoid' );
end
if length( x ) < 8 && ( strcmp( equals, 'xy' ) || strcmp( equals, 'xz' ) )
   error( 'Must have at least 8 points to fit a unique ellipsoid with two equal radii' );
end
if length( x ) < 6 && strcmp( equals, '0' )
   error( 'Must have at least 6 points to fit a unique oriented ellipsoid' );
end
if length( x ) < 5 && ( strcmp( equals, '0xy' ) || strcmp( equals, '0xz' ) )
   error( 'Must have at least 5 points to fit a unique oriented ellipsoid with two equal radii' );
end
if length( x ) < 4 && strcmp( equals, 'xyz' );
   error( 'Must have at least 4 points to fit a unique sphere' );
end

% fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx +
% 2Hy + 2Iz + J = 0 and A + B + C = 3 constraint removing one extra
% parameter
if strcmp( equals, '' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
        x .* x + z .* z - 2 * y .* y, ...
        2 * x .* y, ...
        2 * x .* z, ...
        2 * y .* z, ...
        2 * x, ...
        2 * y, ...
        2 * z, ...
        1 + 0 * x ];  % ndatapoints x 9 ellipsoid parameters
elseif strcmp( equals, 'xy' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
        2 * x .* y, ...
        2 * x .* z, ...
        2 * y .* z, ...
        2 * x, ...
        2 * y, ...
        2 * z, ...
        1 + 0 * x ];  % ndatapoints x 8 ellipsoid parameters
elseif strcmp( equals, 'xz' )
    D = [ x .* x + z .* z - 2 * y .* y, ...
        2 * x .* y, ...
        2 * x .* z, ...
        2 * y .* z, ...
        2 * x, ...
        2 * y, ...
        2 * z, ...
        1 + 0 * x ];  % ndatapoints x 8 ellipsoid parameters
    % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1
elseif strcmp( equals, '0' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
          x .* x + z .* z - 2 * y .* y, ...
          2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 6 ellipsoid parameters
    % fit ellipsoid in the form Ax^2 + By^2 + Cz^2 + 2Gx + 2Hy + 2Iz = 1,
    % where A = B or B = C or A = C
elseif strcmp( equals, '0xy' )
    D = [ x .* x + y .* y - 2 * z .* z, ...
          2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 5 ellipsoid parameters
elseif strcmp( equals, '0xz' )
    D = [ x .* x + z .* z - 2 * y .* y, ...
          2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 5 ellipsoid parameters
     % fit sphere in the form A(x^2 + y^2 + z^2) + 2Gx + 2Hy + 2Iz = 1
elseif strcmp( equals, 'xyz' )
    D = [ 2 * x, ...
          2 * y, ... 
          2 * z, ... 
          1 + 0 * x ];  % ndatapoints x 4 ellipsoid parameters
else
    error( [ 'Unknown parameter value ' equals '!' ] );
end

% solve the normal system of equations
d2 = x .* x + y .* y + z .* z; % the RHS of the llsq problem (y's)
u = ( D' * D ) \ ( D' * d2 );  % solution to the normal equations

% find the residual sum of errors
% chi2 = sum( ( 1 - ( D * u ) ./ d2 ).^2 ); % this chi2 is in the coordinate frame in which the ellipsoid is a unit sphere.

% find the ellipsoid parameters
% convert back to the conventional algebraic form
if strcmp( equals, '' )
    v(1) = u(1) +     u(2) - 1;
    v(2) = u(1) - 2 * u(2) - 1;
    v(3) = u(2) - 2 * u(1) - 1;
    v( 4 : 10 ) = u( 3 : 9 );
elseif strcmp( equals, 'xy' )
    v(1) = u(1) - 1;
    v(2) = u(1) - 1;
    v(3) = -2 * u(1) - 1;
    v( 4 : 10 ) = u( 2 : 8 );
elseif strcmp( equals, 'xz' )
    v(1) = u(1) - 1;
    v(2) = -2 * u(1) - 1;
    v(3) = u(1) - 1;
    v( 4 : 10 ) = u( 2 : 8 );
elseif strcmp( equals, '0' )
    v(1) = u(1) +     u(2) - 1;
    v(2) = u(1) - 2 * u(2) - 1;
    v(3) = u(2) - 2 * u(1) - 1;
    v = [ v(1) v(2) v(3) 0 0 0 u( 3 : 6 )' ];
elseif strcmp( equals, '0xy' )
    v(1) = u(1) - 1;
    v(2) = u(1) - 1;
    v(3) = -2 * u(1) - 1;
    v = [ v(1) v(2) v(3) 0 0 0 u( 2 : 5 )' ];
elseif strcmp( equals, '0xz' )
    v(1) = u(1) - 1;
    v(2) = -2 * u(1) - 1;
    v(3) = u(1) - 1;
    v = [ v(1) v(2) v(3) 0 0 0 u( 2 : 5 )' ];
elseif strcmp( equals, 'xyz' )
    v = [ -1 -1 -1 0 0 0 u( 1 : 4 )' ];
end
v = v';

% form the algebraic form of the ellipsoid
A = [ v(1) v(4) v(5) v(7); ...
      v(4) v(2) v(6) v(8); ...
      v(5) v(6) v(3) v(9); ...
      v(7) v(8) v(9) v(10) ];
% find the center of the ellipsoid
center = -A( 1:3, 1:3 ) \ v( 7:9 );
% form the corresponding translation matrix
T = eye( 4 );
T( 4, 1:3 ) = center';
% translate to the center
R = T * A * T';
% solve the eigenproblem
[ evecs, evals ] = eig( R( 1:3, 1:3 ) / -R( 4, 4 ) );
radii = sqrt( 1 ./ diag( abs( evals ) ) );
sgns = sign( diag( evals ) );
radii = radii .* sgns;

% calculate difference of the fitted points from the actual data normalized by the conic radii
d = [ x - center(1), y - center(2), z - center(3) ]; % shift data to origin
d = d * evecs; % rotate to cardinal axes of the conic;
d = [ d(:,1) / radii(1), d(:,2) / radii(2), d(:,3) / radii(3) ]; % normalize to the conic radii
chi2 = sum( abs( 1 - sum( d.^2 .* repmat( sgns', size( d, 1 ), 1 ), 2 ) ) );

if abs( v(end) ) > 1e-6
    v = -v / v(end); % normalize to the more conventional form with constant term = -1
else
    v = -sign( v(end) ) * v;
end

switch handles.MainData.Calibration.EllFit.Type
    case 'Bias'
        handles.MainData.Calibration.EllFit.V = center;
    case 'Scale'
        handles.MainData.Calibration.EllFit.evecs = evecs;
        handles.MainData.Calibration.EllFit.radii = radii;
end
guidata(hObject,handles)


% --- Executes when Calibration is resized.
function Calibration_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to Calibration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.axes2.Units             = 'normalized';
handles.axes3.Units             = 'normalized';
handles.axes4.Units             = 'normalized';
handles.axes5.Units             = 'normalized';
handles.Sensor_Panel.Units      = 'normalized';
handles.CalTypeBtnGroup.Units   = 'normalized';
handles.CalSettingsPanel.Units  = 'normalized';
handles.CalResultsPanel.Units   = 'normalized';

% OuterPosition = [X Y Width Height]
handles.axes2.OuterPosition             = [0 3/4 0.8 1/4];
handles.axes3.OuterPosition             = [0 2/4 0.8 1/4];
handles.axes4.OuterPosition             = [0 1/4 0.8 1/4];
handles.axes5.OuterPosition             = [0 0 0.8 1/4];
handles.Sensor_Panel.OuterPosition      = [0.8 7/8 0.2 1/8];
handles.CalTypeBtnGroup.OuterPosition   = [0.8 6/8 0.2 1/8];
handles.CalSettingsPanel.OuterPosition  = [0.8 3/8 0.2 3/8];
handles.CalResultsPanel.OuterPosition   = [0.8 0/8 0.2 3/8];


% --- Executes when Sensor_Panel is resized.
function Sensor_Panel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to Sensor_Panel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.SensorPopupMenu.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.SensorPopupMenu.OuterPosition    = [0.00 0.85 1.00 0.15];


% --- Executes when CalTypeBtnGroup is resized.
function CalTypeBtnGroup_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to CalTypeBtnGroup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.CalTypePopupMenu.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.CalTypePopupMenu.OuterPosition = [0.00 0.85 1.00 0.15];


% --- Executes when CalSettingsPanel is resized.
function CalSettingsPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to CalSettingsPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.CalSetCalibrationBtn.Units  = 'normalized';
handles.Cal_Set_1.Units             = 'normalized';
handles.Cal_Set_2.Units             = 'normalized';
handles.Cal_Set_3.Units             = 'normalized';
handles.Cal_Set_4.Units             = 'normalized';
handles.Cal_Set_5.Units             = 'normalized';
handles.Cal_Set_6.Units             = 'normalized';
handles.Cal_Set_7.Units             = 'normalized';
handles.Cal_Set_8.Units             = 'normalized';
handles.Cal_Set_9.Units             = 'normalized';
handles.Cal_Set_Val_1.Units         = 'normalized';
handles.Cal_Set_Val_2.Units         = 'normalized';
handles.Cal_Set_Val_3.Units         = 'normalized';
handles.Cal_Set_Val_4.Units         = 'normalized';
handles.Cal_Set_Val_5.Units         = 'normalized';
handles.Cal_Set_Val_6.Units         = 'normalized';
handles.Cal_Set_Val_7.Units         = 'normalized';
handles.Cal_Set_Val_8.Units         = 'normalized';
handles.Cal_Set_Val_9.Units         = 'normalized';

% OuterPosition = [X Y Width Height]
handles.CalSetCalibrationBtn.OuterPosition  = [0.00 0.90 1.00 0.08];
handles.Cal_Set_1.OuterPosition             = [0.00 0.80 0.50 0.08];
handles.Cal_Set_2.OuterPosition             = [0.00 0.70 0.50 0.08];
handles.Cal_Set_3.OuterPosition             = [0.00 0.60 0.50 0.08];
handles.Cal_Set_4.OuterPosition             = [0.00 0.50 0.50 0.08];
handles.Cal_Set_5.OuterPosition             = [0.00 0.40 0.50 0.08];
handles.Cal_Set_6.OuterPosition             = [0.00 0.30 0.50 0.08];
handles.Cal_Set_7.OuterPosition             = [0.00 0.20 0.50 0.08];
handles.Cal_Set_8.OuterPosition             = [0.00 0.10 0.50 0.08];
handles.Cal_Set_9.OuterPosition             = [0.00 0.00 0.50 0.08];
handles.Cal_Set_Val_1.OuterPosition         = [0.50 0.80 0.50 0.08];
handles.Cal_Set_Val_2.OuterPosition         = [0.50 0.70 0.50 0.08];
handles.Cal_Set_Val_3.OuterPosition         = [0.50 0.60 0.50 0.08];
handles.Cal_Set_Val_4.OuterPosition         = [0.50 0.50 0.50 0.08];
handles.Cal_Set_Val_5.OuterPosition         = [0.50 0.40 0.50 0.08];
handles.Cal_Set_Val_6.OuterPosition         = [0.50 0.30 0.50 0.08];
handles.Cal_Set_Val_7.OuterPosition         = [0.50 0.20 0.50 0.08];
handles.Cal_Set_Val_8.OuterPosition         = [0.50 0.10 0.50 0.08];
handles.Cal_Set_Val_9.OuterPosition         = [0.50 0.00 0.50 0.08];


% --- Executes when CalResultsPanel is resized.
function CalResultsPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to CalResultsPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.Cal_Par_1.Units         = 'normalized';
handles.Cal_Par_2.Units         = 'normalized';
handles.Cal_Par_3.Units         = 'normalized';
handles.Cal_Par_4.Units         = 'normalized';
handles.Cal_Par_5.Units         = 'normalized';
handles.Cal_Par_6.Units         = 'normalized';
handles.Cal_Par_7.Units         = 'normalized';
handles.Cal_Par_8.Units         = 'normalized';
handles.Cal_Par_9.Units         = 'normalized';
handles.Cal_Par_10.Units        = 'normalized';
handles.Cal_Par_Val_1.Units     = 'normalized';
handles.Cal_Par_Val_2.Units     = 'normalized';
handles.Cal_Par_Val_3.Units     = 'normalized';
handles.Cal_Par_Val_4.Units     = 'normalized';
handles.Cal_Par_Val_5.Units     = 'normalized';
handles.Cal_Par_Val_6.Units     = 'normalized';
handles.Cal_Par_Val_7.Units     = 'normalized';
handles.Cal_Par_Val_8.Units     = 'normalized';
handles.Cal_Par_Val_9.Units     = 'normalized';
handles.Cal_Par_Val_10.Units    = 'normalized';

% OuterPosition = [X Y Width Height]
handles.Cal_Par_1.OuterPosition         = [0.00 0.90 0.50 0.08];
handles.Cal_Par_2.OuterPosition         = [0.00 0.80 0.50 0.08];
handles.Cal_Par_3.OuterPosition         = [0.00 0.70 0.50 0.08];
handles.Cal_Par_4.OuterPosition         = [0.00 0.60 0.50 0.08];
handles.Cal_Par_5.OuterPosition         = [0.00 0.50 0.50 0.08];
handles.Cal_Par_6.OuterPosition         = [0.00 0.40 0.50 0.08];
handles.Cal_Par_7.OuterPosition         = [0.00 0.30 0.50 0.08];
handles.Cal_Par_8.OuterPosition         = [0.00 0.20 0.50 0.08];
handles.Cal_Par_9.OuterPosition         = [0.00 0.10 0.50 0.08];
handles.Cal_Par_10.OuterPosition        = [0.00 0.00 0.50 0.08];
handles.Cal_Par_Val_1.OuterPosition     = [0.50 0.90 0.50 0.08];
handles.Cal_Par_Val_2.OuterPosition     = [0.50 0.80 0.50 0.08];
handles.Cal_Par_Val_3.OuterPosition     = [0.50 0.70 0.50 0.08];
handles.Cal_Par_Val_4.OuterPosition     = [0.50 0.60 0.50 0.08];
handles.Cal_Par_Val_5.OuterPosition     = [0.50 0.50 0.50 0.08];
handles.Cal_Par_Val_6.OuterPosition     = [0.50 0.40 0.50 0.08];
handles.Cal_Par_Val_7.OuterPosition     = [0.50 0.30 0.50 0.08];
handles.Cal_Par_Val_8.OuterPosition     = [0.50 0.20 0.50 0.08];
handles.Cal_Par_Val_9.OuterPosition     = [0.50 0.10 0.50 0.08];
handles.Cal_Par_Val_10.OuterPosition    = [0.50 0.00 0.50 0.08];
