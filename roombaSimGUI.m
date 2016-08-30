function varargout = roombaSimGUI(varargin)
% roombasimgui M-file for roombasimgui.fig
% Simulator for the iRobot Create
% To run, enter "roombasimgui" into the command window with not input or
%   output arguments.  This will bring up the GUI window for the simulator,
%   and initialize a roombaSim object to hold simulation data and perform
%   the required functions.

% roombasimgui.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com

% Edit the above text to modify the response to help roombasimgui

% Last Modified by GUIDE v2.5 26-Jul-2016 06:44:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @roombaSimGUI_OpeningFcn, ...
    'gui_OutputFcn',  @roombaSimGUI_OutputFcn, ...
    'gui_LayoutFcn',  [], ...
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
end

% --- Executes just before roombasimgui is made visible.
function roombaSimGUI_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)
% UIWAIT makes roombasimgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);

% control position/size of desktop window
disp('Setting window position and layout');
% the following works for full MATLAB, but not -nodesktop option
desktop = com.mathworks.mde.desk.MLDesktop.getInstance;
desktop.restoreLayout('Command Window Only');
desktop.closeGroup('Editor')
ss = get(0,'screensize');
screen_width = ss(3);
screen_height = ss(4);
frame = desktop.getMainFrame;
if ~isempty(frame)
    frame.setBounds(java.awt.Rectangle(0,0,screen_width/2,screen_height-50));
end
obj = roombaSim(hObject);
assignin('base','s',obj);
handles.simObj = obj;
handles.map_items = [];
handles.output = [];

% Get constant properties for use in plotting
[rad, rIR, rSon, rLid, angRLid, numPtsLid]= getConstants(obj);

% Plot robot in default position and store plot handles for updating
showMap(hObject,handles,obj);
axes(handles.axes_map)
hold on;
axis(obj.getWorldLimits());
axis equal;
circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
circ_ang=linspace(0,2*pi,circ_numPts);
circ_rad=ones(1,circ_numPts)*rad;
[circ_x,circ_y]= pol2cart(circ_ang,circ_rad);
handle_circ= fill(circ_x,circ_y,'g-','LineWidth',1.5);
handle_line= plot([0 1.5*rad],[0 0],'k-','LineWidth',1.5);
handles.botPlot = [handle_circ ; handle_line];

% Plot initial sensors visualization and make invisible
% These plot coordinates are rough to reduce computation here, but it is
%   required that something is plotted
handle_bumpR= plot([0 rad],[-rad 0],'m-','LineWidth',2);
handle_bumpF= plot([rad rad],[-rad/2 rad/2],'m-','LineWidth',2);
handle_bumpL= plot([0 rad],[rad 0],'m-','LineWidth',2);
cx = rad.*cos(obj.cliffSensorLocations);
cy = rad.*sin(obj.cliffSensorLocations);

handles.sensors.bumpers = [handle_bumpR, handle_bumpF, handle_bumpL];

for i=1:6 % light bumper text
    handles.sensors.lightBumpers(i) = text(0,0,'0','color','k','fontsize',8,'horizontalalign','left','verticalalign','middle');
end
for i=1:4 % cliff sensor text
    handles.sensors.cliffSensors(i) = text(0,0,'0','color','k','fontsize',8,'horizontalalign','left','verticalalign','middle');
end

set(handles.sensors.bumpers,'Visible','off')
set(handles.sensors.lightBumpers,'Visible','off')
set(handles.sensors.cliffSensors,'Visible','off')


% Update handles structure
guidata(hObject, handles);

% Set up timer to control simulation updates
timerSim= timer;
timerSim.BusyMode= 'drop';
timerSim.ExecutionMode= 'fixedSpacing';
timerSim.Period= 0.02;
timerSim.TasksToExecute= inf;
timerSim.TimerFcn= {@updateSim,obj}
disp('Starting simulation timer');
start(timerSim)
handles.timerSim = timerSim;

% Update handles structure
guidata(hObject, handles);
end

% --- Outputs from this function are returned to the command line.
function varargout = roombaSimGUI_OutputFcn(hObject, ~, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% place simulator figure window in left half of screen
disp('Moving simulator figure window');
set(hObject,'units','normalized','position',[0.05,.05,0.45,.90]);
set(hObject, 'menubar', 'none');
set(hObject, 'toolbar', 'none');
end


% --- Executes on button press in push_map.
function push_map_Callback(hObject, ~, handles)
% hObject    handle to push_map (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import map file
[filename, pathname, filter]= uigetfile('*.txt','Import Map File');
if filename                         % Make sure cancel was not pressed
    loadMap(hObject,handles,filename,pathname);
end
end

% --- Executes on button press in push_config.
function push_config_Callback(~, ~, handles)
% hObject    handle to push_config (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Import configuration file
[filename, pathname, ~]= uigetfile('*.txt','Import Configuration File');
if filename                         % Make sure cancel was not pressed
    % Get robot object for data entry
    obj = handles.simObj;
    loadConfigFile(obj,pathname,filename);
end

end


% --- Executes on button press in push_origin.
function push_origin_Callback(~, ~, handles)
% hObject    handle to push_origin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set origin data in robot object
placeRobot(handles.simObj);

end


function placeRobot(obj)
placing= 1;
state= getState(obj);
x= state(1);
y= state(2);
th= state(3);
yoff=-0.3;
ht = text(x,y+yoff,'Click on Robot to start placing it','color','r','background','w','horizontalalign','center');
% two nested functions
    function mousemove(~,~)
        pt = get(gca,'CurrentPoint');
        if placing==1
            return
        elseif placing==2
            set(ht,'string','Specify Location ','position',[x,y+yoff,0]);
            x=pt(1,1);
            y=pt(1,2);
        elseif placing==3
            set(ht,'string','Specify Rotation','position',[x,y+yoff,0]);
            th=atan2(pt(1,2)-y,pt(1,1)-x);
        end
        origin= [x,y,th];
        setMapStart(obj,origin);
    end

    function mousebutton(~,~)
        placing=mod(placing+1,4);
    end

set(gcf,'WindowButtonMotionFcn',@mousemove,'WindowButtonDownFcn',@mousebutton)
while placing
    pause(.05);
end
delete(ht);
set(gcf,'WindowButtonMotionFcn','','WindowButtonDownFcn','');

end

% while toc<5
%     [pt] = get(gca,'CurrentPoint')
%     origin= [pt(1) pt(2) th];         % Start point is set by first mouse click
%     setMapStart(obj,origin)
% end


function chkbx_lightbumper_Callback(~, ~, handles)
% hObject    handle to chkbx_lightbumper (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Toggle visibility of sensor visualization with the checkbox
set(handles.sensors.lightBumpers,'Visible',iff(get(handles.chkbx_lightbumper,'Value'),'on','off'));
end

function chkbx_bump_Callback(~, ~, handles)
% hObject    handle to chkbx_bump (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkbx_bump

% Toggle visibility of sensor visualization with the checkbox
if get(handles.chkbx_bump,'Value')
    set(handles.sensors.bumpers,'Visible','on')
else
    set(handles.sensors.bumpers,'Visible','off')
end
end

function chkbx_cliff_Callback(~, ~, handles)
% hObject    handle to chkbx_cliff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chkbx_cliff

% Toggle visibility of sensor visualization with the checkbox
if get(handles.chkbx_cliff,'Value')
    set(handles.sensors.cliffSensors,'Visible','on')
else
    set(handles.sensors.cliffSensors,'Visible','off')
end
end


% --- Executes on button press in push_sensors.
function push_sensors_Callback(~, ~, handles)
% hObject    handle to push_sensors (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get robot object to query sensors and output to command window
[rad rIR rSon rLid angRLid numPtsLid]= getConstants(handles.simObj);

% Bump sensors
bump= genBump(obj);
bumpR= bump(1);
bumpF= bump(2);
bumpL= bump(3);
fprintf('Bump Sensors:\n')
fprintf('\tRight: %.0f\n',bumpR)
fprintf('\tFront: %.0f\n',bumpF)
fprintf('\tLeft: %.0f\n\n',bumpL)

% Cliff sensors
cliffR= 0;
cliffFR= 0;
cliffFL= 0;
cliffL= 0;
cliff= genCliff(obj);
cliffRstr= cliff(1);
cliffFRstr= cliff(2);
cliffFLstr= cliff(3);
cliffLstr= cliff(4);
fprintf('Cliff Sensors:\n')
fprintf('\tRight: State %.0f Strength %%%.3f\n',cliffR,cliffRstr)
fprintf('\tFront-Right: State %.0f Strength %%%.3f\n',cliffFR,cliffFRstr)
fprintf('\tFront-Left: State %.0f Strength %%%.3f\n',cliffFL,cliffFLstr)
fprintf('\tLeft: State %.0f Strength %%%.3f\n\n',cliffL,cliffLstr)


% Virtual Wall sensor
vwall= genVWall(obj);
fprintf('Virtual Wall Sensor: %.0f\n\n',vwall)

% Odometry
distOdom= genOdomDist(obj);
angOdom= genOdomAng(obj);
fprintf('Odometry Data (since last call):\n')
fprintf('\tDistance: %.3f m\n',distOdom)
fprintf('\tAngle: %.3f rad\n\n',angOdom)

% Encoders
fprintf('Encoder Counts (cumulative):\n')
[left,right]= genEncoderCount(obj);
fprintf('\tLeft Encoder: %.0f\n',left)
fprintf('\tRight Encoder: %.0f\n\n',right)

% Overhead localization system
[x y th]= genOverhead(obj);
fprintf('Overhead Localization System output:\n')
fprintf('\tX-Coordinate: %.3f m\n',x)
fprintf('\tY-Coordinate: %.3f m\n',y)
fprintf('\tAngle relative to horizontal: %.3f rad\n\n',th)

% Camera
[angCameraBeacon,distCameraBeacon,colorCam,ID]= genCamera(obj);
% Also see ReadBeacon function in roombaSim.m for details on camera.

X = -distCameraBeacon.*sin(angCameraBeacon);    % Minus, because camera x-axis is to the right
Z = +distCameraBeacon.*cos(angCameraBeacon);    % Camera's positive z-axis points forward (depth)
fprintf('Camera Data:\n')
for i= 1:length(angCameraBeacon)
    fprintf('Beacon at: theta = %.0f deg, R = %.3f m, with color [%.2f %.2f %.2f] \n',...
        rad2deg(angCameraBeacon(i)),distCameraBeacon(i),colorCam(i,1),colorCam(i,2),...
        colorCam(i,3))
    fprintf('           X = %.2f m, Y = %.2f m, Z = %.2f m, rot = %.0f deg, with tag number %.0f \n',...
        X(i),0,Z(i),0,ID(i))
end
fprintf('\n')
end




function toggle_clock_Callback(hObject, ~, handles)
handles.simObj.buttons.clock = get(hObject,'Value');
end

function toggle_schedule_Callback(hObject, ~, handles)
handles.simObj.buttons.schedule = get(hObject,'Value');
end

function toggle_hour_Callback(hObject, ~, handles)
handles.simObj.buttons.hour = get(hObject,'Value');
end

function toggle_day_Callback(hObject, ~, handles)
handles.simObj.buttons.day = get(hObject,'Value');
end

function toggle_minute_Callback(hObject, ~, handles)
handles.simObj.buttons.minute = get(hObject,'Value');
end

function toggle_dock_Callback(hObject, ~, handles)
handles.simObj.buttons.dock = get(hObject,'Value');
end

function toggle_spot_Callback(hObject, ~, handles)
handles.simObj.buttons.spot = get(hObject,'Value');
end

function toggle_clean_Callback(hObject, ~, handles)
handles.simObj.buttons.clean = get(hObject,'Value');
end

function R=iff(condition,trueval,falseval)
if condition
    R=trueval;
else
    R=falseval;
end
end % iff


function loadMap(hObject,handles,filename,pathname)
if (nargin<3)
    fullname = which(filename);
else
    fullname = [pathname filename];
end
fid= fopen(fullname);% Get file handle for parsing
if ~fid
    warning('File not found: %s %s',pathname,filename);
    return;
end
% Parse the file and extract relevant information
walls= [];
lines= [];
beacs= {};
vwalls= [];
while ~feof(fid)
    fline= fgetl(fid);
    line= lower(fline);     % Convert to lowercase if necessary
    line= strtrim(line);    % Delete leading and trailing whitespace
    lineWords= {};          % To keep track of line entries
    while ~isempty(line) && ~strcmp(line(1),'%')% End of line or comment
        [word line]= strtok(line);  % Get next entry
        line= strtrim(line);        % To be able to detect comments
        lineWords= [lineWords word];
    end
    
    %%% Regular expressions would probably be more efficient %%%
    if length(lineWords) == 5 && strcmp(lineWords{1},'wall') && ...
            ~isnan(str2double(lineWords{2})) && ...
            ~isnan(str2double(lineWords{3})) && ...
            ~isnan(str2double(lineWords{4})) && ...
            ~isnan(str2double(lineWords{5}))
        walls= [walls ; str2double(lineWords{2}) ...
            str2double(lineWords{3}) str2double(lineWords{4}) ...
            str2double(lineWords{5})];
    elseif length(lineWords) == 5 && strcmp(lineWords{1},'line') && ...
            ~isnan(str2double(lineWords{2})) && ...
            ~isnan(str2double(lineWords{3})) && ...
            ~isnan(str2double(lineWords{4})) && ...
            ~isnan(str2double(lineWords{5}))
        lines= [lines ; str2double(lineWords{2}) ...
            str2double(lineWords{3}) str2double(lineWords{4}) ...
            str2double(lineWords{5})];
    elseif length(lineWords) == 7 && strcmp(lineWords{1},'beacon') ...
            && ~isnan(str2double(lineWords{2})) && ...
            ~isnan(str2double(lineWords{3})) && ...
            length(lineWords{4}) >= 2 && ...
            strcmp(lineWords{4}(1),'[') && ...
            ~isnan(str2double(lineWords{4}(2:end))) && ...
            ~isnan(str2double(lineWords{5})) && ...
            length(lineWords{6}) >= 2 && ...
            strcmp(lineWords{6}(end),']') && ...
            ~isnan(str2double(lineWords{6}(1:end-1)))
        beacs= [beacs ; str2double(lineWords{2}) ...
            str2double(lineWords{3}) ...
            str2double(lineWords{4}(2:end)) ...
            str2double(lineWords{5}) ...
            str2double(lineWords{6}(1:end-1)) lineWords(7)];
    elseif length(lineWords) == 5 && ...
            strcmp(lineWords{1},'virtwall') && ...
            ~isnan(str2double(lineWords{2})) && ...
            ~isnan(str2double(lineWords{3})) && ...
            ~isnan(str2double(lineWords{4})) && ...
            ~isnan(str2double(lineWords{5})) && ...
            str2double(lineWords{5}) >= 1 && ...
            str2double(lineWords{5}) <= 3
        vwalls= [vwalls ; str2double(lineWords{2}) ...
            str2double(lineWords{3}) str2double(lineWords{4}) ...
            str2double(lineWords{5})];
    elseif ~isempty(lineWords)
        warning('MATLAB:invalidInput',...
            'This line in map file %s is unrecognized:\n\t%s',...
            filename,fline)
    end
end
fclose(fid);

% Set map data in robot object
handles.simObj.setMap(walls,lines,beacs,vwalls);
showMap(hObject,handles,obj);
end % loadMap

function showMap(hObject,handles,obj)
% Clear old map
axes(handles.axes_map)
delete(handles.map_items);
handles.map_items=[];
[start,walls,lines,beacs,vwalls] = obj.getMap();

% Plot walls
for i= 1:size(walls,1)
    handles.map_items(end+1) = plot(walls(i,[1 3]),walls(i,[2 4]),'r-','LineWidth',3);
end
% Plot lines
for i= 1:size(lines,1)
    handles.map_items(end+1) = plot(lines(i,[1 3]),lines(i,[2 4]),'k-','LineWidth',1);
end

% Plot beacons
for i= 1:size(beacs,1)
    handles.map_items(end+1) = plot(beacs{i,1},beacs{i,2},...
        'Color',cell2mat(beacs(i,3:5)),'Marker','o');
    handles.map_items(end+1) = text(beacs{i,1},beacs{i,2},['  ' beacs{i,6}]);
end

% Plot virtual walls
% Define virtual wall emitter constants
halo_rad= 0.45;     % Radius of the halo around the emitter
range_short= 2.13;  % Range of the wall on the 0-3' setting
ang_short= 0.33;    % Angular range on the 0-3' setting
range_med= 5.56;    % Range of the wall on the 4'-7' setting
ang_med= 0.49;      % Angular range on the 4'-7' setting
range_long= 8.08;   % Range of the wall on the 8'+ setting
ang_long= 0.61;     % Angular range on the 8'+ setting

% Get points to map virtual walls
for i= 1:size(vwalls,1)
    x_vw= vwalls(i,1);
    y_vw= vwalls(i,2);
    th_vw= vwalls(i,3);
    if vwalls(i,4) == 1
        range_vw= range_short;
        ang_vw= ang_short;
    elseif vwalls(i,4) == 2
        range_vw= range_med;
        ang_vw= ang_med;
    else
        range_vw= range_long;
        ang_vw= ang_long;
    end
    x_1= x_vw+range_vw*cos(th_vw+ang_vw/2);
    y_1= y_vw+range_vw*sin(th_vw+ang_vw/2);
    x_2= x_vw+range_vw*cos(th_vw-ang_vw/2);
    y_2= y_vw+range_vw*sin(th_vw-ang_vw/2);
    
    % Plot halo around emitter and range triangle
    circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
    circ_ang=linspace(0,2*pi,circ_numPts);
    circ_rad=ones(1,circ_numPts)*halo_rad;
    [circ_x,circ_y]= pol2cart(circ_ang,circ_rad);
    circ_x= circ_x+x_vw;
    circ_y= circ_y+y_vw;
    handles.map_items(end+1) = plot(x_vw,y_vw,'g*');
    handles.map_items(end+1) = plot(circ_x,circ_y,'g:','LineWidth',1);
    handles.map_items(end+1) = plot([x_vw x_1 x_2 x_vw],[y_vw y_1 y_2 y_vw],'g:','LineWidth',1);
end
guidata(hObject,handles);
end % showMap


% --- Executes when user attempts to close figure_simulator.
function figure_simulator_CloseRequestFcn(hObject, ~, handles)
% hObject    handle to figure_simulator (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Delete the timer to prevent further updates
try
    if isfield(handles,'timerSim');
        stop(handles.timerSim)
        delete(handles.timerSim)
    end
catch
    disp('Problem deleting timer');
end

% Close the figure
delete(hObject);
end

function updateSim(timerSim,~,obj)
% updateSim(timerSim,event,obj)
% Update the simulation of the robot.
% Use current position and movement
% information to plot the next step and update the robot information
%
% Input:
% timerSim - Timer object (required argument for timers)
% event - Structure, contains information about timer call
%   (required argument for timers)
% obj - CreateRobot object, contains information about robot

% updateSim.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com
try
handles = guidata(obj.figure);
robotRad = obj.radius;

% Get time since last update
tStep= timerSim.InstantPeriod;
if isnan(tStep)                 % First function call
    tStep= timerSim.Period;     % Assume period is correct
end

% Extract values
state= getState(obj);
x= state(1);
y= state(2);
v= state(4:5);

% Check for collisions with walls
collPts= findCollisions(obj);

% Depending on if walls are hit, update position differently
if isempty(collPts)         % No walls
    driveNormal(obj,tStep)
elseif size(collPts,1) == 1 % One wall
    if ~collPts(4)          % No corner
        drive1Wall(obj,tStep,collPts)
    else                    % One corner
        driveCorner(obj,tStep,collPts)
    end
else                        % Two walls
    if ~any(collPts(:,4))   % No corners
        drive2Wall(obj,tStep,collPts)
    elseif xor(collPts(1,4),collPts(2,4))   % One corner
        collPts= collPts(find(~collPts(:,4)),:);
        drive1Wall(obj,tStep,collPts)       % Only look at wall
    else                    % Two corners
        % Only look at corner that is closest to the trajectory
        vec1= [collPts(1,1)-x collPts(1,2)-y]/...
            sqrt((collPts(1,1)-x)^2+(collPts(1,2)-y)^2);
        vec2= [collPts(2,1)-x collPts(2,2)-y]/...
            sqrt((collPts(2,1)-x)^2+(collPts(2,2)-y)^2);
        [closest,closeIdx]= max([dot(v,vec1) dot(v,vec2)]);
        collPts= collPts(closeIdx,:);
        driveCorner(obj,tStep,collPts)
    end
end

% Extract updated state values
oldstate= state;
state= getState(obj);
x= state(1);
y= state(2);
th= state(3);

% Update odometry values
obj.updateOdom(oldstate,state)

% If in robot-centric view mode move plot focal point
if get(handles.radio_centric,'Value')
    curr_xlimit= get(handles.axes_map,'XLim');
    curr_ylimit= get(handles.axes_map,'YLim');
    half_xdist= (curr_xlimit(2)-curr_xlimit(1))/2;
    half_ydist= (curr_ylimit(2)-curr_ylimit(1))/2;
    set(handles.axes_map,'XLim',[x-half_xdist x+half_xdist])
    set(handles.axes_map,'YLim',[y-half_ydist y+half_ydist])
end

% Line approximation of the robot
circ_numPts= 21;    % Estimate circle as circ_numPts-1 lines
circ_ang=linspace(0,2*pi,circ_numPts);
circ_rad=ones(1,circ_numPts)*robotRad;
[circ_x,circ_y]= pol2cart(circ_ang,circ_rad);
circ_x=circ_x+x;
circ_y=circ_y+y;

% Update robot circle and direction line
set(handles.botPlot(1),'XData',circ_x);
set(handles.botPlot(1),'YData',circ_y);
set(handles.botPlot(2),'XData',[x x+1.5*robotRad*cos(th)]);
set(handles.botPlot(2),'YData',[y y+1.5*robotRad*sin(th)]);

% Update sensors visualization
updateSensorVisualization(obj)

drawnow
catch e
    disp(e)
    for i=1:length(e.stack)
        fprintf('  %s %u\n',e.stack(i).name,e.stack(i).line);
    end
end
end
