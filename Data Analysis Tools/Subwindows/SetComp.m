function varargout = SetComp(varargin)
% SETCOMP MATLAB code for SetComp.fig
%      SETCOMP, by itself, creates a new SETCOMP or raises the existing
%      singleton*.
%
%      H = SETCOMP returns the handle to a new SETCOMP or the handle to
%      the existing singleton*.
%
%      SETCOMP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SETCOMP.M with the given input arguments.
%
%      SETCOMP('Property','Value',...) creates a new SETCOMP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SetComp_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SetComp_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SetComp

% Last Modified by GUIDE v2.5 13-Mar-2018 12:48:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SetComp_OpeningFcn, ...
                   'gui_OutputFcn',  @SetComp_OutputFcn, ...
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


% --- Executes just before SetComp is made visible.
function SetComp_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SetComp (see VARARGIN)

% Choose default command line output for SetComp
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end
handles.PhiRadioBtn.Value = 0;
handles.ThetaRadioBtn.Value = 0;
handles.PsiRadioBtn.Value = 0;
handles.PlotFlags = false(3,1);
% Update handles structure
guidata(hObject, handles);
PlotAngles(handles)
% PlotAngles(handles)

% UIWAIT makes SetComp wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SetComp_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

function CalculateNewAngles(handles)
% Get configuration parameters
VMag = handles.MainData.Calibration.Mag.V;
WMag = handles.MainData.Calibration.Mag.W;
CMag = handles.MainData.Calibration.Mag.C(:,1);
VAcc = handles.MainData.Calibration.Acc.V;
WAcc = handles.MainData.Calibration.Acc.W;
VGyro = handles.MainData.Calibration.Gyro.V;
WGyro = handles.MainData.Calibration.Gyro.W;

Fc = str2num(handles.Fc.String);
Ts = str2num(handles.Ts.String);

% Get Data
RawMag = handles.MainData.DataSet.RAW_SENSOR_DATA(1:3,:);
RawAcc = handles.MainData.DataSet.RAW_SENSOR_DATA(4:6,:);
RawGyro = handles.MainData.DataSet.RAW_SENSOR_DATA(7:9,:);
RawCur = handles.MainData.DataSet.RAW_SENSOR_DATA(15,:);
Time = handles.MainData.DataSet.CLOCK(5,:);

% Correct data
for i = 1:length(Time)
    CorMag(:,i) = WMag*(RawMag(:,i) - VMag) - CMag*RawCur(:,i);
    CorAcc(:,i) = WAcc*(RawAcc(:,i) - VAcc);
    CorGyro(:,i) = WGyro*(RawGyro(:,i) - VGyro);
end
for i = 1:3
    CorGyro(i,:) = cumtrapz(Time, CorGyro(i,:));
end

% Calculate angles
Phi_AM = atan2(CorAcc(2,:),CorAcc(3,:));
Theta_AM = -sign(CorAcc(3,:)).*atan2(CorAcc(1,:), sqrt(CorAcc(2,:).^2 + CorAcc(3,:).^2));

% Unwrapping angles
for i = 2:length(Time)
    if(Phi_AM(i) - Phi_AM(i-1) > pi)
        Phi_AM(i) = Phi_AM(i) - round((Phi_AM(i) - Phi_AM(i-1))/pi)*pi;
    elseif(Phi_AM(i) - Phi_AM(i-1) < -pi)
        Phi_AM(i) = Phi_AM(i) + round((Phi_AM(i) - Phi_AM(i-1))/-pi)*pi;
    end
end


for i = 1:length(Time)
    Rx = [1 0 0; 0 cos(Phi_AM(i)) -sin(Phi_AM(i)); 0 sin(Phi_AM(i)) cos(Phi_AM(i))];
    Ry = [cos(Theta_AM(i)) 0 sin(Theta_AM(i)); 0 1 0; -sin(Theta_AM(i)) 0 cos(Theta_AM(i))];
    Mt(:,i) = Ry*Rx*CorMag(:,i);
    Psi_AM(:,i) = -atan2(Mt(2,i), Mt(1,i));
end
Psi_AM = Psi_AM - mean(Psi_AM(Time < 1));

% Unwrap angles
for i = 2:size(Time,2)
    if(Psi_AM(i) - Psi_AM(i-1) > 0.9*pi)
        Psi_AM(i) = Psi_AM(i) - round((Psi_AM(i) - Psi_AM(i-1))/pi)*pi;
    elseif(Psi_AM(i) - Psi_AM(i-1) < -0.9*pi)
        Psi_AM(i) = Psi_AM(i) + round((Psi_AM(i) - Psi_AM(i-1))/-pi)*pi;
    end
end

% Setup Complementary Filter
HpFC = tf([1/Fc 0], [1/Fc 1]);
LpFC = tf([0 1], [1/Fc 1]);

HpFD = c2d(HpFC, Ts);
LpFD = c2d(LpFC, Ts);

Phi_Lp = lsim(LpFD, Phi_AM)';
Theta_Lp = lsim(LpFD, Theta_AM)';
Psi_Lp = lsim(LpFD, Psi_AM)';

Phi_Hp = lsim(HpFD, CorGyro(1,:))';
Theta_Hp = lsim(HpFD, CorGyro(2,:))';
Psi_Hp = lsim(HpFD, CorGyro(3,:))';

Phi = Phi_Lp + Phi_Hp;
Theta = Theta_Lp + Theta_Hp;
Psi = Psi_Lp + Psi_Hp;

% Set data to plot
handles.PlotData.X = handles.MainData.DataSet.CLOCK(5,:);
handles.PlotData.Y1 = [Phi_AM; Theta_AM; Psi_AM; Phi_Lp; Theta_Lp; Psi_Lp];
handles.PlotData.Legend1 = {'Raw \phi', 'Raw \theta', 'Raw \psi', 'Lp \phi', 'Lp \theta', 'Lp \psi', };
handles.PlotData.Flags1 = [handles.PlotFlags; handles.PlotFlags];

handles.PlotData.Y2 = [CorGyro; Phi_Hp; Theta_Hp; Psi_Hp];
handles.PlotData.Legend2 = {'Raw \phi', 'Raw \theta', 'Raw \psi', 'Hp \phi', 'Hp \theta', 'Hp \psi', };
handles.PlotData.Flags2 = [handles.PlotFlags; handles.PlotFlags];

handles.PlotData.Y3 = [Phi; Theta; Psi];
handles.PlotData.Legend3 = {'\phi', '\theta', '\psi'};
handles.PlotData.Flags3 = [handles.PlotFlags];

handles.PlotData.LpFD = LpFD;
handles.PlotData.HpFD = HpFD;

handles.LpNum1.String = num2str(LpFD.Num{1:end}(1));
handles.LpNum2.String = num2str(LpFD.Num{1:end}(2));
handles.LpDen1.String = num2str(LpFD.Den{1:end}(1));
handles.LpDen2.String = num2str(LpFD.Den{1:end}(2));
handles.HpNum1.String = num2str(HpFD.Num{1:end}(1));
handles.HpNum2.String = num2str(HpFD.Num{1:end}(2));
handles.HpDen1.String = num2str(HpFD.Den{1:end}(1));
handles.HpDen2.String = num2str(HpFD.Den{1:end}(2));
PlotAngles(handles)



function PlotAngles(handles)
AxesList = {'AccMagAngAxes', 'GyroAngAxes', 'CombAngAxes', 'CompAxes'};
TitleList = {'Acc & Mag Angles', 'Gyro Angles', 'Complementary Filter Angles', 'Complementary'};
YDataList = {'Y1', 'Y2', 'Y3'};
LegendList = {'Legend1', 'Legend2', 'Legend3'};
FlagList = {'Flags1', 'Flags2', 'Flags3'};
Color = [1 0 0; 0 1 0; 0 0 1; 0.50 0 0; 0 0.50 0; 0 0 0.50; 0.25 0.25 0.25];
for i = 1:4
    cla(handles.(AxesList{i}), 'reset')    
end

if(any(handles.PlotFlags))
    if(any([handles.PlotData.Flags1; handles.PlotData.Flags2; handles.PlotData.Flags3]))
        Ylimits = 0;
        for i = 1:3
            Ylimits = max([Ylimits max(max(abs(handles.PlotData.(YDataList{i})(handles.PlotData.(FlagList{i}),:) )))]);
        end
        
        for i = 1:3
            for j = 1:size(handles.PlotData.(YDataList{i}), 1)
                if((handles.PlotData.(FlagList{i})(j)))
                    plot(handles.(AxesList{i}), handles.PlotData.X, handles.PlotData.(YDataList{i})(j,:), 'color', Color(j, :))
                    hold(handles.(AxesList{i}), 'on')
                end
            end
            grid(handles.(AxesList{i}), 'on')
            xlabel(handles.(AxesList{i}), 'Time')
            ylabel(handles.(AxesList{i}), 'Angles')
            title(handles.(AxesList{i}), TitleList{i})
            axis(handles.(AxesList{i}),  'tight')
%             ylim(handles.(AxesList{i}), ([-Ylimits Ylimits]))
%             xlim(handles.(AxesList{i}), ([handles.PlotData.X(1) handles.PlotData.X(end)]))
            legend(handles.(AxesList{i}), handles.PlotData.(LegendList{i})(handles.PlotData.(FlagList{i})), 'location', 'best')
        end
        
        bode(handles.CompAxes, handles.PlotData.LpFD, 'b')
        hold(handles.CompAxes, 'on')
        bode(handles.CompAxes, handles.PlotData.HpFD, 'r')
        legend(handles.CompAxes, 'Low Pass', 'High Pass', 'location', 'best')
    end
end

for i = 1:3
    grid(handles.(AxesList{i}), 'on')
    xlabel(handles.(AxesList{i}), 'Time')
    ylabel(handles.(AxesList{i}), 'Angles')
    title(handles.(AxesList{i}), TitleList{i})
end
grid(handles.CompAxes, 'on')
title(handles.CompAxes, 'Complementary Filter')
    




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


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
CalculateNewAngles(handles)
guidata(hObject, handles);




function LpNum1_Callback(hObject, eventdata, handles)
% hObject    handle to LpNum1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LpNum1 as text
%        str2double(get(hObject,'String')) returns contents of LpNum1 as a double


% --- Executes during object creation, after setting all properties.
function LpNum1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LpNum1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function LpNum2_Callback(hObject, eventdata, handles)
% hObject    handle to LpNum2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LpNum2 as text
%        str2double(get(hObject,'String')) returns contents of LpNum2 as a double


% --- Executes during object creation, after setting all properties.
function LpNum2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LpNum2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function HpNum1_Callback(hObject, eventdata, handles)
% hObject    handle to HpNum1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of HpNum1 as text
%        str2double(get(hObject,'String')) returns contents of HpNum1 as a double


% --- Executes during object creation, after setting all properties.
function HpNum1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to HpNum1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function HpNum2_Callback(hObject, eventdata, handles)
% hObject    handle to HpNum2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of HpNum2 as text
%        str2double(get(hObject,'String')) returns contents of HpNum2 as a double


% --- Executes during object creation, after setting all properties.
function HpNum2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to HpNum2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function LpDen1_Callback(hObject, eventdata, handles)
% hObject    handle to LpDen1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LpDen1 as text
%        str2double(get(hObject,'String')) returns contents of LpDen1 as a double


% --- Executes during object creation, after setting all properties.
function LpDen1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LpDen1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function LpDen2_Callback(hObject, eventdata, handles)
% hObject    handle to LpDen2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of LpDen2 as text
%        str2double(get(hObject,'String')) returns contents of LpDen2 as a double


% --- Executes during object creation, after setting all properties.
function LpDen2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to LpDen2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function HpDen1_Callback(hObject, eventdata, handles)
% hObject    handle to HpDen1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of HpDen1 as text
%        str2double(get(hObject,'String')) returns contents of HpDen1 as a double


% --- Executes during object creation, after setting all properties.
function HpDen1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to HpDen1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function HpDen2_Callback(hObject, eventdata, handles)
% hObject    handle to HpDen2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of HpDen2 as text
%        str2double(get(hObject,'String')) returns contents of HpDen2 as a double


% --- Executes during object creation, after setting all properties.
function HpDen2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to HpDen2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in PhiRadioBtn.
function PhiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PhiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(1) = handles.PhiRadioBtn.Value;
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of PhiRadioBtn


% --- Executes on button press in ThetaRadioBtn.
function ThetaRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ThetaRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(2) = handles.ThetaRadioBtn.Value;
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of ThetaRadioBtn


% --- Executes on button press in PsiRadioBtn.
function PsiRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PsiRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotFlags(3) = handles.PsiRadioBtn.Value;
guidata(hObject, handles);
CalculateNewAngles(handles)
% Hint: get(hObject,'Value') returns toggle state of PsiRadioBtn
