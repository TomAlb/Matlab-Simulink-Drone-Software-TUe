function varargout = AnalyseDroneData(varargin)
% ANALYSEDRONEDATA MATLAB code for AnalyseDroneData.fig
%      ANALYSEDRONEDATA, by itself, creates a new ANALYSEDRONEDATA or raises the existing
%      singleton*.
%
%      H = ANALYSEDRONEDATA returns the handle to a new ANALYSEDRONEDATA or the handle to
%      the existing singleton*.
%
%      ANALYSEDRONEDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ANALYSEDRONEDATA.M with the given input arguments.
%
%      ANALYSEDRONEDATA('Property','Value',...) creates a new ANALYSEDRONEDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AnalyseDroneData_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AnalyseDroneData_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AnalyseDroneData

% Last Modified by GUIDE v2.5 17-Sep-2018 15:46:00

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AnalyseDroneData_OpeningFcn, ...
                   'gui_OutputFcn',  @AnalyseDroneData_OutputFcn, ...
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


% --- Executes just before AnalyseDroneData is made visible.
function AnalyseDroneData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AnalyseDroneData (see VARARGIN)

% Choose default command line output for AnalyseDroneData
handles.output = hObject;
DataFolder = 'Measurement_Data';
% List all folders currently in thi path
handles.Load_Data_Location.Path = cd;
if(exist(DataFolder) ~= 7)
    % Folder does not exist
    disp('Folder Measurement_Data does not exist!')
    mkdir(DataFolder);
    disp('Folder Measurement_Data created!')
    disp('Load flight data in Measurement_Data!')
end
handles.Load_Data_Location.Path = [cd '\' DataFolder];
addpath('Subwindows')

files = dir(handles.Load_Data_Location.Path);
dirFlags = [files.isdir];
dirFlags(1:2) = 0;
subFolders = files(dirFlags);
s = {'',char(subFolders(1:end).name)};
set(handles.FolderSelectionList, 'string', s);
% Set initial Calibration Parameters
handles.Calibration.Mag.V = [0.0168008;... 
                             0.171846;...
                            -0.0625626];                    % initial Hard iron corrections
handles.Calibration.Mag.B = 0.3578;                                             % initial Magnitude
handles.Calibration.Mag.W = [1.05062 -0.0686782 0.00743534;...
                             -0.0686782 1.0053 -0.0151594;...
                             0.00743534 -0.0151594 0.957706];                     % initial Soft iron corrections
handles.Calibration.Mag.C = [0.0017 -0.1056; -0.0075 0.1617; -0.008 0.4284];    % initial Current corrections
handles.Calibration.Mag.X = zeros(3,1);                                         % Calibration Data

handles.Calibration.Acc.V = [0.0899951; -0.0644168; -0.161696];                           % Initial Acc offset
handles.Calibration.Acc.W = [1.00976   -0.0686782 -0.00487985;...
                                -0.00272847  1.00145 -0.00249778;...
                                 -0.004879854 -0.00249778 0.989088];                                             % Initial Acc scaling

handles.Calibration.Baro.V = -52.4747;                                                % Initial Barometer offset
handles.Calibration.Baro.W = -1.19068;                                                % Initial Barometer scaling                            

handles.Calibration.Gyro.V = [0; 0; 0];                         % Initial Gyro offset
handles.Calibration.Gyro.W = diag([0.992736 1.01609 1.03003]);                      % Initial Gyro scaling

handles.Calibration.GPS.V = [0; 0; 0];                                    % Initial GPS offset
handles.Calibration.GPS.W = diag([1e-2; 1e-2; 1e-3]);                        % Initial GPS scaling

handles.Calibration.Bat.V = [0; 0; 0];                                          % Initial Battery offset
handles.Calibration.Bat.W = eye(3);                                             % Initial Battery scaling

handles.Calibration.Sonar.V = -6.87278;                                                % Initial Sonar offset
handles.Calibration.Sonar.W = 438.185;                                                % Initial Sonar scaling

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AnalyseDroneData wait for user response (see UIRESUME)
% uiwait(handles.MainGUI);


% --- Outputs from this function are returned to the command line.
function varargout = AnalyseDroneData_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in FolderSelectionList.
function FolderSelectionList_Callback(hObject, eventdata, handles)
% hObject    handle to FolderSelectionList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str = get(hObject, 'String');
val = get(hObject, 'Value');

if(val ~= 1)
    handles.Load_Data_Location.Folder = strtrim(str{val,:});
    % Get a list  of all files and folders in the selected folder.
    path = [handles.Load_Data_Location.Path '\' handles.Load_Data_Location.Folder];
    DataFiles = dir([path '\RawSensorData_*.bin']);
    SettingP1Files = dir([path '\SettingsP1_*.bin']);
    SettingP2Files = dir([path '\SettingsP2_*.bin']);
    % Take smallest list and gather unique name part
    switch length(DataFiles) >= length(SettingP1Files)
        case true
            for i = 1:length(SettingP1Files)
                ListIdent{i} = regexprep(SettingP1Files(i).name, {'SettingsP1_','.bin'},'','ignorecase');
            end
        case false
            for i = 1:length(DataFiles)
                ListIdent{i} = regexprep(DataFiles(i).name, {'RawSensorData_','.bin'},'','ignorecase');
            end
    end
    % Test if both Settings_*.bin and RawSensorData_*.bin do exist
    for i = 1:length(ListIdent)
        Flags(i,1) = exist([path '\RawSensorData_' ListIdent{i} '.bin'],'file') == 2;
        Flags(i,2) = exist([path '\SettingsP1_' ListIdent{i} '.bin'],'file') == 2;
        Flags(i,3) = exist([path '\SettingsP2_' ListIdent{i} '.bin'],'file') == 2;
    end
    % Load list of files in dropdown menu
    s = {' ',ListIdent{Flags(:,1) & Flags(:,2)}};
    set(handles.FileSelectionList, 'String', s)
    set(handles.FileSelectionList, 'Value', 1)
    guidata(hObject, handles);
    % get(handles.FolderSelectionList
    % Hints: contents = cellstr(get(hObject,'String')) returns FolderSelectionList contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from FolderSelectionList
end

% --- Executes during object creation, after setting all properties.
function FolderSelectionList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FolderSelectionList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in FileSelectionList.
function FileSelectionList_Callback(hObject, eventdata, handles)
% hObject    handle to FileSelectionList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str = get(hObject, 'String');
val = get(hObject, 'Value');

if(val ~= 1)
    handles.Load_Data_Location.Files = strtrim(str{val,:});
    guidata(hObject, handles);
    % handles.Load_Data_Location.Folder = str{val,:};
    % Hints: contents = cellstr(get(hObject,'String')) returns FileSelectionList contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from FileSelectionList
end

% --- Executes during object creation, after setting all properties.
function FileSelectionList_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FileSelectionList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in LoadDataBtn.
function LoadDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to LoadDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
LoadFile{1} = [handles.Load_Data_Location.Path '\' handles.Load_Data_Location.Folder '\RawSensorData_' handles.Load_Data_Location.Files '.bin'];
LoadFile{2} = [handles.Load_Data_Location.Path '\' handles.Load_Data_Location.Folder '\SettingsP1_' handles.Load_Data_Location.Files '.bin'];
LoadFile{3} = [handles.Load_Data_Location.Path '\' handles.Load_Data_Location.Folder '\SettingsP2_' handles.Load_Data_Location.Files '.bin'];

for i = 1:length(LoadFile)
    dataFile = LoadFile{i};
    datapts = 0;
    fid = fopen(dataFile, 'r');
    % load the header information
    hdrToken = fread(fid, 8, 'char');
    if strncmp(char(hdrToken),'MWLOGV',6) == true
        logTime = uint32(fread(fid, 1, 'uint32'));
        numflds = double(fread(fid, 1, 'uint8'));
        typefld = uint8(fread(fid, 1, 'uint8'));
        recSize = uint16(fread(fid, 1, 'uint16'));
        switch(typefld)
            case 1
                dtypeStr = 'double';
            case 2
                dtypeStr = 'single';
            case 3
                dtypeStr = 'int32';
            case 4
                dtypeStr = 'uint32';
            case 5
                dtypeStr = 'int16';
            case 6
                dtypeStr = 'uint16';
            case 7
                dtypeStr = 'int8';
            case 8
                dtypeStr = 'uint8';
            case 9
                dtypeStr = 'logical';
            case 10
                dtypeStr = 'embedded.fi';
        end
        fieldTypeStr = dtypeStr;
        datapts = fread(fid, double([numflds, Inf]), fieldTypeStr);
        fclose(fid);
    end
    if(i == 1)
        handles.Data = datapts;
    elseif(i == 2)
        handles.Params = datapts;
    elseif(i == 3)
        handles.Params = [handles.Params; datapts];
    end
end
guidata(hObject, handles);
Set_Structure_Btn_Callback(hObject, eventdata, handles)

function RC_Size_Callback(hObject, eventdata, handles)
% hObject    handle to RC_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
get(hObject, 'string')
% Hints: get(hObject,'String') returns contents of RC_Size as text
%        str2double(get(hObject,'String')) returns contents of RC_Size as a double


% --- Executes during object creation, after setting all properties.
function RC_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to RC_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Target_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Target_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Target_Size as text
%        str2double(get(hObject,'String')) returns contents of Target_Size as a double


% --- Executes during object creation, after setting all properties.
function Target_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Target_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Raw_Sensor_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Raw_Sensor_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Raw_Sensor_Size as text
%        str2double(get(hObject,'String')) returns contents of Raw_Sensor_Size as a double


% --- Executes during object creation, after setting all properties.
function Raw_Sensor_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Raw_Sensor_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Filtered_Sensor_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Filtered_Sensor_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Filtered_Sensor_Size as text
%        str2double(get(hObject,'String')) returns contents of Filtered_Sensor_Size as a double


% --- Executes during object creation, after setting all properties.
function Filtered_Sensor_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Filtered_Sensor_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function State_Size_Callback(hObject, eventdata, handles)
% hObject    handle to State_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of State_Size as text
%        str2double(get(hObject,'String')) returns contents of State_Size as a double


% --- Executes during object creation, after setting all properties.
function State_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to State_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Error_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Error_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Error_Size as text
%        str2double(get(hObject,'String')) returns contents of Error_Size as a double


% --- Executes during object creation, after setting all properties.
function Error_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Error_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Motor_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Motor_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Motor_Size as text
%        str2double(get(hObject,'String')) returns contents of Motor_Size as a double


% --- Executes during object creation, after setting all properties.
function Motor_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Motor_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Clock_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Clock_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Clock_Size as text
%        str2double(get(hObject,'String')) returns contents of Clock_Size as a double


% --- Executes during object creation, after setting all properties.
function Clock_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Clock_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Controller_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Controller_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Controller_Size as text
%        str2double(get(hObject,'String')) returns contents of Controller_Size as a double


% --- Executes during object creation, after setting all properties.
function Controller_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Controller_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function System_Mode_Size_Callback(hObject, eventdata, handles)
% hObject    handle to System_Mode_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of System_Mode_Size as text
%        str2double(get(hObject,'String')) returns contents of System_Mode_Size as a double


% --- Executes during object creation, after setting all properties.
function System_Mode_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to System_Mode_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Complementary_Size_Callback(hObject, eventdata, handles)
% hObject    handle to Complementary_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Complementary_Size as text
%        str2double(get(hObject,'String')) returns contents of Complementary_Size as a double


% --- Executes during object creation, after setting all properties.
function Complementary_Size_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Complementary_Size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Set_Structure_Btn.
function Set_Structure_Btn_Callback(hObject, eventdata, handles)
% hObject    handle to Set_Structure_Btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Obtain data sizes
Bs = [str2double(get(handles.RC_Size, 'String')) ...
str2double(get(handles.Target_Size, 'String')) ...
str2double(get(handles.Raw_Sensor_Size, 'String')) ...
str2double(get(handles.Filtered_Sensor_Size, 'String')) ...
str2double(get(handles.State_Size, 'String')) ...
str2double(get(handles.Error_Size, 'String')) ...
str2double(get(handles.Controller_Size, 'String')) ...
str2double(get(handles.Motor_Size, 'String')) ...
str2double(get(handles.System_Mode_Size, 'String')) ...
str2double(get(handles.Clock_Size, 'String')) ...
str2double(get(handles.Complementary_Size, 'String'))];
Bl = cumsum(Bs);
l = 1;

% Orden Measured data in substructures
if(Bl(end) == size(handles.Data,1))
    handles.DataSet.RC                      = handles.Data(l:Bl(l),:);
    handles.DataSet.TARGET                  = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.RAW_SENSOR_DATA         = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.FILTERED_SENSOR_DATA    = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.STATE                   = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.ERROR                   = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.CONTROLLER              = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.MOTOR                   = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.SYSTEM_MODE             = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.CLOCK                   = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    handles.DataSet.COMPLEMENTARY           = handles.Data(Bl(l)+1:Bl(l+1),:); l = l + 1;
    
    % Adjustments
    % handles.DataSet.RAW_SENSOR_DATA(16,:) = handles.DataSet.RAW_SENSOR_DATA(16,:)/1000; % Scale mAh to Ah
    handles.DataSet.CLOCK = handles.DataSet.CLOCK - handles.DataSet.CLOCK(:,1);     % Set start time to 0
    handles.DataSet.CLOCK(1,:) = handles.DataSet.CLOCK(1,:)*10^-6;                  % Convert microseconds to seconds
    handles.DataSet.CLOCK(2,:) = handles.DataSet.CLOCK(2,:)*10^-6;                  % Convert microseconds to seconds
    handles.DataSet.CLOCK(4,:) = handles.DataSet.CLOCK(4,:)*10^-6;                  % Convert microseconds to seconds
    disp('Data Loaded');
else
    disp('Set Data structure is not the same as the loaded data');
end

guidata(hObject, handles);
% set(handles.Rc_Size, 'string', num2str(DataSizes(1)));
% guidata(hObject, handles);


% --- Executes on button press in RawSensorSubWindowBtn.
function RawSensorSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RawSensorSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowRawSensorData


% --- Executes on button press in FilSensorSubWindowBtn.
function FilSensorSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FilSensorSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowFilteredSensorData


% --- Executes on button press in AccSubWindowBtn.
function AccSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to AccSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowAccelerometerData


% --- Executes on button press in MagSubWindowBtn.
function MagSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to MagSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowMagnetometerData


% --- Executes on button press in CalibrationBtn.
function CalibrationBtn_Callback(hObject, eventdata, handles)
% hObject    handle to CalibrationBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Calibration


% --- Executes on button press in StateSubWindowBtn.
function StateSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to StateSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowStateData

% --- Executes on button press in RCSubWindowBtn.
function RCSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RCSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowRCData

% --- Executes on button press in ErrorSubWindowBtn.
function ErrorSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ErrorSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowErrorData


% --- Executes on button press in PWMBtn.
function PWMBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PWMBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Torques2PWM


% --- Executes on button press in ParamSubWindowBtn.
function ParamSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ParamSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Parameters


% --- Executes on button press in ReferenceBtn.
function ReferenceBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ReferenceBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowReference


% --- Executes on button press in AHRS_btn.
function AHRS_btn_Callback(hObject, eventdata, handles)
% hObject    handle to AHRS_btn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SetAHRS

% --- Executes on button press in pushbutton33.
function pushbutton33_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton33 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SetKalman


% --- Executes on button press in SystemModeSubWindowBtn.
function SystemModeSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to SystemModeSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
SystemModeData


% --- Executes on button press in FRFDataWindowBtn.
function FRFDataWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to FRFDataWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
FRFData


% --- Executes on button press in TimingSubWindowBtn.
function TimingSubWindowBtn_Callback(hObject, eventdata, handles)
% hObject    handle to TimingSubWindowBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Timing


% --- Executes on button press in RefVSStateBtn.
function RefVSStateBtn_Callback(hObject, eventdata, handles)
% hObject    handle to RefVSStateBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
ShowErrorVSRef
