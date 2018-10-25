function varargout = ShowMagnetometerData(varargin)
% SHOWMAGNETOMETERDATA MATLAB code for ShowMagnetometerData.fig
%      SHOWMAGNETOMETERDATA, by itself, creates a new SHOWMAGNETOMETERDATA or raises the existing
%      singleton*.
%
%      H = SHOWMAGNETOMETERDATA returns the handle to a new SHOWMAGNETOMETERDATA or the handle to
%      the existing singleton*.
%
%      SHOWMAGNETOMETERDATA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SHOWMAGNETOMETERDATA.M with the given input arguments.
%
%      SHOWMAGNETOMETERDATA('Property','Value',...) creates a new SHOWMAGNETOMETERDATA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ShowRAwSensorData_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ShowRAwSensorData_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ShowMagnetometerData

% Last Modified by GUIDE v2.5 20-Aug-2018 13:16:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ShowMagnetometerData_OpeningFcn, ...
                   'gui_OutputFcn',  @ShowMagnetometerData_OutputFcn, ...
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


% --- Executes just before ShowMagnetometerData is made visible.
function ShowMagnetometerData_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ShowMagnetometerData (see VARARGIN)

% Choose default command line output for ShowMagnetometerData
handles.output = hObject;
h = findobj('Tag', 'MainGUI');
% if exist (not empty)
if ~isempty(h)
    % get handles and other user-defined data associated to MainGUI
    handles.MainData = guidata(h);
end

handles.RequestDataFlags = false(5,1);
handles.DataTypeFlags = true(3,1);

handles.PlotGraphPopUpMenu.String = {'XY', 'XZ', 'YZ', 'Time', '3D', 'Current', 'Height', 'Dip'};
handles.PlotGraphPopUpMenu.Value = 4;

set(handles.PlotRawDataBtn, 'Value', 1);
set(handles.PlotFilDataBtn, 'Value', 1);
set(handles.PlotCorDataBtn, 'Value', 1);
handles.RequestData = 'Time';
PlotRequestedData(hObject, eventdata, handles);
grid on
axis tight
xlabel('Time')
ylabel('Data')
title('Magnetometer Data')
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ShowMagnetometerData wait for user response (see UIRESUME)
% uiwait(handles.ShowMagnetometerData);


% --- Outputs from this function are returned to the command line.
function varargout = ShowMagnetometerData_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on selection change in PlotGraphPopUpMenu.
function PlotGraphPopUpMenu_Callback(hObject, eventdata, handles)
% hObject    handle to PlotGraphPopUpMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
switch handles.PlotGraphPopUpMenu.String{handles.PlotGraphPopUpMenu.Value}
    case 'XY'
        handles.RequestData = '3D';
        handles.RequestDataSub = 'XY';
    case 'XZ'
        handles.RequestData = '3D';
        handles.RequestDataSub = 'XZ';
    case 'YZ'
        handles.RequestData = '3D';
        handles.RequestDataSub = 'YZ';
    case '3D'
        handles.RequestData = '3D';
        handles.RequestDataSub = 'XYZ';
    case 'Current'
        handles.RequestData = 'Current';
    case 'Height'
        handles.RequestData = 'Height';
    case 'Dip'
        handles.RequestData = 'Dip';
    otherwise 
        handles.RequestData = 'Time';
end
guidata(hObject, handles);
PlotRequestedData(hObject, eventdata, handles)
% Hints: contents = cellstr(get(hObject,'String')) returns PlotGraphPopUpMenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from PlotGraphPopUpMenu


% --- Executes during object creation, after setting all properties.
function PlotGraphPopUpMenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PlotGraphPopUpMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Plots data on figure
function PlotRequestedData(hObject, eventdata, handles)
% hObject    handle to BatteryRadioBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Collect necesary data
V = handles.MainData.Calibration.Mag.V;
B = handles.MainData.Calibration.Mag.B;
W = handles.MainData.Calibration.Mag.W;
C = handles.MainData.Calibration.Mag.C;
Time = handles.MainData.DataSet.CLOCK(end,:);
RawData = handles.MainData.DataSet.RAW_SENSOR_DATA(1:3,:);
FilData = handles.MainData.DataSet.FILTERED_SENSOR_DATA(1:3,:);
HeightData = handles.MainData.DataSet.STATE(3,:);
CurData = handles.MainData.DataSet.RAW_SENSOR_DATA(16,:);
CorData = W*(handles.MainData.DataSet.RAW_SENSOR_DATA(1:3,:)-V) - C(:,1)*CurData;
PlotData = [RawData; FilData; CorData];

cla(handles.MagnetometerAxes, 'reset')
Color = [1.00 0.00 0.00; 0.00 1.00 0.00; 0.00 0.00 1.00; 0.50 0.50 0.50;...
         0.75 0.00 0.00; 0.00 0.75 0.00; 0.00 0.00 0.75; 0.25 0.25 0.25;...
         0.50 0.00 0.00; 0.00 0.50 0.00; 0.00 0.00 0.25; 0.00 0.00 0.00];
switch handles.RequestData
    case 'Time'
        hold on
        marker = {'-', '-', '-'};
        for j = 1:3
            if(handles.DataTypeFlags(j))
                for i = 1:3
                    plot(Time, PlotData((j-1)*3+i,:),marker{j} ,'color',Color((j-1)*4+i,:))
                end
                plot(Time, sqrt(sum(PlotData((j-1)*3+1:(j-1)*3+3,:).^2)),marker{j},'color',Color((j-1)*4+4,:))
            end
        end
        xlabel('Time')
        ylabel('Data')
        axis tight
        LegendBool = false(12,1);
        LegendBool(1:4,:) = handles.DataTypeFlags(1);
        LegendBool(5:8,:) = handles.DataTypeFlags(2);
        LegendBool(9:12,:) = handles.DataTypeFlags(3);
        LegendStr = {'Raw M_x', 'Raw M_y', 'Raw M_z', 'Raw M_m', 'Fil M_x', 'Fil M_y', 'Fil M_z', 'Fil M_m', 'Cor M_x', 'Cor M_y', 'Cor M_z', 'Cor M_m'};
        if(any(LegendBool))
            legend(LegendStr(LegendBool),'Location','bestoutside')
        end
        
    case '3D'
        for i = 1:3
            if(handles.DataTypeFlags(i))
                plot3(PlotData((i-1)*3+1,:), PlotData((i-1)*3+2,:), PlotData((i-1)*3+3,:),'color',Color(i,:)), hold on
            end
        end
        
        for i = 1:3
            if(handles.DataTypeFlags(i))
                handles.PlotMag.X = PlotData((i-1)*3+1:(i-1)*3+3,:)';
                guidata(hObject, handles);
                ellipsoid_fit(hObject, eventdata, handles);
                handles = guidata(hObject);
                V = handles.PlotMag.V;
                R = handles.PlotMag.R;
                Al = handles.PlotMag.Al;
                
                %draw fit
                mind = V - R;
                maxd = V + R;
                nsteps = 50;
                step = ( maxd - mind ) / nsteps;
                [ x, y, z ] = meshgrid( linspace( mind(1) - step(1), maxd(1) + step(1), nsteps ), linspace( mind(2) - step(2), maxd(2) + step(2), nsteps ), linspace( mind(3) - step(3), maxd(3) + step(3), nsteps ) );
                Ellipsoid = Al(1) *x.*x +   Al(2) * y.*y + Al(3) * z.*z + ...
                    2*Al(4) *x.*y + 2*Al(5)*x.*z + 2*Al(6) * y.*z + ...
                    2*Al(7) *x    + 2*Al(8)*y    + 2*Al(9) * z;
                
                p = patch( isosurface( x, y, z, Ellipsoid, -Al(10) ) );
                set( p, 'FaceColor', Color(i,:), 'EdgeColor', 'none' );
                alpha(0.15)
                camlight;
                lighting phong;
                switch handles.RequestDataSub
                    case 'XYZ'
                        az = 35; el = 33;
                    case 'XY'
                        az = 0; el = 90;
                    case 'XZ'
                        az = 180; el = 0;
                    case 'YZ'
                        az = 90; el = 0;
                end
            end
        end
        
        xlabel('X-Data')
        ylabel('Y-Data')
        zlabel('Z-Data')
        axis image
        LegendStr = {'Raw Data','Fil Data','Cor Data', 'Raw Data Limit','Fil Data Limit','Cor Data Limit'};
        legend(LegendStr([handles.DataTypeFlags; handles.DataTypeFlags]),'Location','bestoutside')
        view(az, el);
    case 'Current'
        hold on
        marker = {'-', '--', '.'};
        for j = 1:3
            if(handles.DataTypeFlags(j))
                for i = 1:3
                    plot(CurData, PlotData((j-1)*3+i,:),marker{j} ,'color',Color((j-1)*4+i,:))
                end
                plot(CurData, sqrt(sum(PlotData((j-1)*3+1:(j-1)*3+3,:).^2)),marker{j} ,'color',Color((j-1)*4+4,:))
            end
        end
        xlabel('Current')
        ylabel('Magneto Data')
        axis tight
        LegendBool = false(12,1);
        LegendBool(1:4,:) = handles.DataTypeFlags(1);
        LegendBool(5:8,:) = handles.DataTypeFlags(2);
        LegendBool(9:12,:) = handles.DataTypeFlags(3);
        LegendStr = {'Raw M_x', 'Raw M_y', 'Raw M_z', 'Raw M_m', 'Fil M_x', 'Fil M_y', 'Fil M_z', 'Fil M_m', 'Cor M_x', 'Cor M_y', 'Cor M_z', 'Cor M_m'};
        legend(LegendStr(LegendBool),'Location','bestoutside')
        
    case 'Height'
        hold on
        marker = {'-', '--', '.'};
        for j = 1:3
            if(handles.DataTypeFlags(j))
                for i = 1:3
                    plot(HeightData, PlotData((j-1)*3+i,:),marker{j} ,'color',Color((j-1)*4+i,:))
                end
                plot(HeightData, sqrt(sum(PlotData((j-1)*3+1:(j-1)*3+3,:).^2)),marker{j} ,'color',Color((j-1)*4+4,:))
            end
        end
        xlabel('Height')
        ylabel('Magneto Data')
        axis tight
        LegendBool = false(12,1);
        LegendBool(1:4,:) = handles.DataTypeFlags(1);
        LegendBool(5:8,:) = handles.DataTypeFlags(2);
        LegendBool(9:12,:) = handles.DataTypeFlags(3);
        LegendStr = {'Raw M_x', 'Raw M_y', 'Raw M_z', 'Raw M_m', 'Fil M_x', 'Fil M_y', 'Fil M_z', 'Fil M_m', 'Cor M_x', 'Cor M_y', 'Cor M_z', 'Cor M_m'};
        legend(LegendStr(LegendBool),'Location','bestoutside')
    
    case 'Dip'
        hold on
        marker = {'.', '--', '--'};
        % plot magnitude
        for j = 1:3
            if(handles.DataTypeFlags(j))
                plot(Time, sqrt(sum(PlotData((j-1)*3+1:(j-1)*3+3,:).^2)),marker{1},'color',Color((j),:))
            end
        end
        % Plot Dip
        for j = 1:3
            if(handles.DataTypeFlags(j))
                plot(Time, sqrt(sum(PlotData((j-1)*3+1:(j-1)*3+3,:).^2)), marker{2}, 'color', Color((j),:))
            end
        end
        xlabel('Time')
        ylabel('Data')
        axis tight
        LegendBool = false(6,1);
        LegendBool(1:3,:) = handles.DataTypeFlags(1:3);
        LegendBool(4:6,:) = handles.DataTypeFlags(1:3);
        LegendStr = {'Raw M_m', 'Fil M_m', 'Cor M_m', 'Raw Dip', 'Fil Dip', 'Cor Dip'};
        if(any(LegendBool))
            legend(LegendStr(LegendBool),'Location','bestoutside')
        end
        
    otherwise
        xlabel('Data')
        ylabel('Data')
        axis tight
end
grid on
title('Magnetometer Data')
guidata(hObject, handles);


% --- Executes on button press in PlotRawDataBtn.
function PlotRawDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PlotRawDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.DataTypeFlags(1) = get(hObject, 'Value');
guidata(hObject, handles);
PlotRequestedData(hObject, eventdata, handles)
% Hint: get(hObject,'Value') returns toggle state of PlotRawDataBtn


% --- Executes on button press in PlotFilDataBtn.
function PlotFilDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PlotFilDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.DataTypeFlags(2) = get(hObject, 'Value');
guidata(hObject, handles);
PlotRequestedData(hObject, eventdata, handles)
% Hint: get(hObject,'Value') returns toggle state of PlotFilDataBtn


% --- Executes on button press in PlotCorDataBtn.
function PlotCorDataBtn_Callback(hObject, eventdata, handles)
% hObject    handle to PlotCorDataBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.DataTypeFlags(3) = get(hObject, 'Value');
guidata(hObject, handles);
PlotRequestedData(hObject, eventdata, handles)
% Hint: get(hObject,'Value') returns toggle state of PlotCorDataBtn

function ellipsoid_fit(hObject, eventdata, handles)
X = handles.PlotMag.X;
equals = '';

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

handles.PlotMag.V = center;
handles.PlotMag.R = radii;
handles.PlotMag.W = evecs;
handles.PlotMag.Al = v;
handles.PlotMag.chi2 = chi2;
guidata(hObject,handles)


% --- Executes when ShowMagnetometerData is resized.
function ShowMagnetometerData_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to ShowMagnetometerData (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.MagnetometerAxes.Units = 'normalized';
handles.PlotGraphBtnGroup.Units = 'normalized';
handles.PlotDataType.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.MagnetometerAxes.OuterPosition = [0 0 0.8 1];
handles.PlotGraphBtnGroup.OuterPosition = [0.8 0.5 0.2 0.5];
handles.PlotDataType.OuterPosition = [0.8 0 0.2 0.5];


% --- Executes when PlotDataType is resized.
function PlotDataType_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to PlotDataType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotRawDataBtn.Units = 'normalized';
handles.PlotFilDataBtn.Units = 'normalized';
handles.PlotCorDataBtn.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.PlotRawDataBtn.OuterPosition = [0.00 0.85 1 0.1];
handles.PlotFilDataBtn.OuterPosition = [0.00 0.75 1 0.1];
handles.PlotCorDataBtn.OuterPosition = [0.00 0.65 1 0.1];


% --- Executes when PlotGraphBtnGroup is resized.
function PlotGraphBtnGroup_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to PlotGraphBtnGroup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.PlotGraphPopUpMenu.Units = 'normalized';

% OuterPosition = [X Y Width Height]
handles.PlotGraphPopUpMenu.OuterPosition = [0.0 0.90 1 0.1];

