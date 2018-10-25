function varargout = ShowStateData(varargin)
% SHOWSTATEDATA MATLAB code for showstatedata.fig
%      SHOWSTATEDATA, by itself, creates a new SHOWSTATEDATA or raises the existing
%      singleton*.
%
%      H = SHOWSTATEDATA returns the handle to a new SHOWSTATEDATA or the handle to
%      the existing singleton*.
%
%      SHOWSTATEDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWSTATEDATA.M with the given input arguments.
%
%      SHOWSTATEDATA('Property','Value',...) creates a new SHOWSTATEDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before showstatedata_openingfcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to showstatedata_openingfcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help showstatedata

% Last Modified by GUIDE v2.5 24-May-2018 10:12:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ShowStateData_OpeningFcn, ...
                   'gui_OutputFcn',  @ShowStateData_OutputFcn, ...
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


% --- Executes just before showstatedata is made visible.
function ShowStateData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to showstatedata (see VARARGIN)

% Choose default command line output for showstatedata
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end
handles.TypePlot = '';
handles.RequestDataFlags = false(18,1);
handles.ArrowFlags = false(3,1);
set(handles.XRadioBtn, 'value', 1);
set(handles.YRadioBtn, 'value', 1);
set(handles.ZRadioBtn, 'value', 1);

set(handles.PosRadioBtn, 'value', 0);
set(handles.AngRadioBtn, 'value', 0);
set(handles.VelRadioBtn, 'value', 0);
set(handles.AngVelRadioBtn, 'value', 0);
set(handles.AccRadioBtn, 'value', 0);
set(handles.AngAccRadioBtn, 'value', 0);

PlotRequestedData(handles)
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes showstatedata wait for user response (see UIRESUME)
% uiwait(handles.ShowStateData);


% --- Outputs from this function are returned to the command line.
function varargout = ShowStateData_OutputFcn(hObject, eventdata, handles) 
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
Flags = [get(handles.PosRadioBtn, 'value') get(handles.AngRadioBtn, 'value') get(handles.VelRadioBtn, 'value') get(handles.AngVelRadioBtn, 'value') get(handles.AccRadioBtn, 'value') get(handles.AngAccRadioBtn, 'value')];
handles.RequestDataFlags([1 4 7 10 13 16]) = false(6,1);
handles.ArrowFlags(1) = false;

switch handles.TypePlot
    case 'Data'
        if(get(hObject,'Value'))
            handles.RequestDataFlags([1 4 7 10 13 16]) = Flags;
        end
    case 'Orientation'
        if(get(hObject,'Value'))
            handles.ArrowFlags(1) = get(hObject,'Value');
        end        
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of XRadioBtn


% --- Executes on button press in YRadioBtn.
function YRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to YRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.AngRadioBtn, 'value') get(handles.VelRadioBtn, 'value') get(handles.AngVelRadioBtn, 'value') get(handles.AccRadioBtn, 'value') get(handles.AngAccRadioBtn, 'value')];
handles.RequestDataFlags([2 5 8 11 14 17]) = false(6,1);
handles.ArrowFlags(2) = false;

switch handles.TypePlot
    case 'Data'
        if(get(hObject,'Value'))
            handles.RequestDataFlags([2 5 8 11 14 17]) = Flags;
        end
    case 'Orientation'
        if(get(hObject,'Value'))
            handles.ArrowFlags(2) = get(hObject,'Value');
        end
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of YRadioBtn


% --- Executes on button press in ZRadioBtn.
function ZRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to ZRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.PosRadioBtn, 'value') get(handles.AngRadioBtn, 'value') get(handles.VelRadioBtn, 'value') get(handles.AngVelRadioBtn, 'value') get(handles.AccRadioBtn, 'value') get(handles.AngAccRadioBtn, 'value')];
handles.RequestDataFlags([3 6 9 12 15 18]) = false(6,1);
handles.ArrowFlags(3) = false;

switch handles.TypePlot
    case 'Data'
        if(get(hObject,'Value'))
            handles.RequestDataFlags([3 6 9 12 15 18]) = Flags;
        end
    case 'Orientation'
        if(get(hObject,'Value'))
            handles.ArrowFlags(3) = get(hObject,'Value');
        end
end


guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of ZRadioBtn


% --- Executes on button press in OtherRadioBtn.
function OtherRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to OtherRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = get(handles.AngVelRadioBtn, 'value');
handles.TypePlot = 'Data';
handles.RequestDataFlags(13) = false;
if(get(hObject,'Value'))
    handles.RequestDataFlags(13) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of OtherRadioBtn


% --- Executes on button press in PosRadioBtn.
function PosRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PosRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.TypePlot = 'Data';
handles.RequestDataFlags(1:3) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(1:3) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of PosRadioBtn


% --- Executes on button press in AngRadioBtn.
function AngRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to AngRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.TypePlot = 'Data';
handles.RequestDataFlags(4:6) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(4:6) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of AngRadioBtn


% --- Executes on button press in VelRadioBtn.
function VelRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to VelRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.TypePlot = 'Data';
handles.RequestDataFlags(7:9) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(7:9) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of VelRadioBtn


% --- Executes on button press in AngVelRadioBtn.
function AngVelRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to AngVelRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.TypePlot = 'Data';
handles.RequestDataFlags(10:12) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(10:12) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of AngVelRadioBtn


% --- Executes on button press in AccRadioBtn.
function AccRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to AccRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.TypePlot = 'Data';
handles.RequestDataFlags(13:15) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(13:15) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of AccRadioBtn


% --- Executes on button press in AngAccRadioBtn.
function AngAccRadioBtn_Callback(hObject, eventdata, handles)
% hObject    handle to AngAccRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Flags = [get(handles.XRadioBtn, 'value') get(handles.YRadioBtn, 'value') get(handles.ZRadioBtn, 'value')];
handles.TypePlot = 'Data';
handles.RequestDataFlags(16:18) = false(1,3);
if(get(hObject,'Value'))
    handles.RequestDataFlags(16:18) = Flags;
end
guidata(hObject, handles);
PlotRequestedData(handles)
% Hint: get(hObject,'Value') returns toggle state of AngAccRadioBtn

% --- Plots data on figure
function PlotRequestedData(handles)
% hObject    handle to AccRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
cla(handles.StateDataAxes, 'reset')
switch handles.TypePlot
    case 'Data'
        Time = handles.MainData.DataSet.CLOCK(5,:);
        Data = handles.MainData.DataSet.STATE(handles.RequestDataFlags, :);
        LegendString = {'X', 'Y', 'Z', '\phi', '\theta', '\psi', 'V_x', 'V_y', 'V_z', 'V_\phi', 'V_\theta', 'V_\psi', 'A_x', 'A_y', 'A_z', 'A_\phi', 'A_\theta', 'A_\psi'};
        if(any(handles.RequestDataFlags))
            plot(Time, Data)
            legend(LegendString{handles.RequestDataFlags},'location', 'best')
        end
        axis tight
        xlabel('Time')
        ylabel('Data')
        title('State')
    case 'Orientation'
        Time = handles.MainData.DataSet.CLOCK(5,:);
        Position = handles.MainData.DataSet.STATE(1:3,:);
        Rotation = handles.MainData.DataSet.STATE(4:6,:);
%         x = -Position(2,:)'; y = Position(1,:)'; z = Position(3,:)';
        x = Position(1,:)'; y = Position(2,:)'; z = Position(3,:)';
        scatter3(x, y, z, 10*ones(length(Position),1), 'filled', 'o','.k'), hold on, drawnow limitrate
        for i = 1:length(Time)
            phi = Rotation(1,i); theta = Rotation(2,i); psi = Rotation(3,i);
            Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
            Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
            Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
            X(i,:) = (Rx*Ry*Rz*[1; 0; 0])';
            Y(i,:) = (Rx*Ry*Rz*[0; 1; 0])';
            Z(i,:) = (Rx*Ry*Rz*[0; 0; 1])';
        end
        
        % Plot arrow X-direction body frame
        if(handles.ArrowFlags(1))
            Asx = [max(max(x))-min(min(x))];
            u = X(:,1); v = X(:,2); w = X(:,3);
            quiver3(x,y,z,u,v,w,5*Asx,'r'), drawnow limitrate
        end
        
        % Plot arrow Y-direction body frame
        if(handles.ArrowFlags(2))
            Asy = [max(max(y))-min(min(y))];
            u = Y(:,1); v = Y(:,2); w = Y(:,3);
            quiver3(x,y,z,u,v,w,5*Asy,'g'), drawnow limitrate
        end
        
        % Plot arrow Z-direction body frame
        if(handles.ArrowFlags(3))
            Asz = [max(max(z))-min(min(z))];
            u = Z(:,1); v = Z(:,2); w = Z(:,3);
            quiver3(x,y,z,u,v,w,5*Asz,'b'), drawnow limitrate
        end
        
        axis square
        set(handles.StateDataAxes, 'Zdir', 'reverse')
        set(handles.StateDataAxes, 'Ydir', 'reverse')
        xlabel('X')
        ylabel('Y')
        zlabel('Z')
        title('Position and Orientation')

    otherwise
        axis tight
        xlabel('Time')
        ylabel('Data')
        title('State')
end
grid on


% --- Executes on button press in PosRotBtn.
function PosRotBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PosRotBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.TypePlot = 'Orientation';
handles.RequestDataFlags = false(18,1);
set(handles.XRadioBtn, 'value', 0);
set(handles.YRadioBtn, 'value', 0);
set(handles.ZRadioBtn, 'value', 0);

set(handles.PosRadioBtn, 'value', 0);
set(handles.AngRadioBtn, 'value', 0);
set(handles.VelRadioBtn, 'value', 0);
set(handles.AngVelRadioBtn, 'value', 0);
set(handles.AccRadioBtn, 'value', 0);
set(handles.AngAccRadioBtn, 'value', 0);
guidata(hObject, handles);
PlotRequestedData(handles)


% --- Executes when ShowRawSensorData is resized.
function ShowRawSensorData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowRawSensorData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.StateDataAxes.Units         = 'normalized';
handles.StateGroupPanel.Units       = 'normalized';
handles.DirectionGroupPanel.Units   = 'normalized';
handles.PosRotBtn.Units             = 'normalized';

% OuterPosition = [X Y Width Height]
handles.StateDataAxes.OuterPosition         = [0.00 0.00 0.75 1.00];
handles.StateGroupPanel.OuterPosition       = [0.75 0.55 0.25 0.45];
handles.DirectionGroupPanel.OuterPosition   = [0.75 0.10 0.25 0.45];
handles.PosRotBtn.OuterPosition             = [0.75 0.00 0.25 0.10];


% --- Executes when DirectionGroupPanel is resized.
function DirectionGroupPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to DirectionGroupPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.XRadioBtn.Units = 'normalized';
handles.YRadioBtn.Units = 'normalized';
handles.ZRadioBtn.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.XRadioBtn.OuterPosition = [0.00 0.90 1.00 0.10];
handles.YRadioBtn.OuterPosition = [0.00 0.80 1.00 0.10];
handles.ZRadioBtn.OuterPosition = [0.00 0.70 1.00 0.10];


% --- Executes when StateGroupPanel is resized.
function StateGroupPanel_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to StateGroupPanel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PosRadioBtn.Units       = 'normalized';
handles.AngRadioBtn.Units       = 'normalized';
handles.VelRadioBtn.Units       = 'normalized';
handles.AngVelRadioBtn.Units    = 'normalized';
handles.AccRadioBtn.Units       = 'normalized';
handles.AngAccRadioBtn.Units    = 'normalized';

% OuterPosition = [X Y Width Height]
handles.PosRadioBtn.OuterPosition       = [0.00 0.90 1.00 0.10];
handles.AngRadioBtn.OuterPosition       = [0.00 0.80 1.00 0.10];
handles.VelRadioBtn.OuterPosition       = [0.00 0.70 1.00 0.10];
handles.AngVelRadioBtn.OuterPosition    = [0.00 0.60 1.00 0.10];
handles.AccRadioBtn.OuterPosition       = [0.00 0.50 1.00 0.10];
handles.AngAccRadioBtn.OuterPosition    = [0.00 0.40 1.00 0.10];
