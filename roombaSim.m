classdef roombaSim < handle
    % Creates definition of class, instances of which will include all
    % properties and controls associated with a single roombaSim robot
    
    % roombaSim.m
    % Copyright (C) 2011 Cornell University
    % This code is released under the open-source BSD license.  A copy of this
    % license should be provided with the software.  If not, email:
    % CreateMatlabSim@gmail.com
    
    properties(Constant, GetAccess= 'public')
        % Values cannot be changed
        % All units are kg/m/s/rad unless otherwise noted
        radius = 0.16;          % Radius of the robot (model as a circle)
        wheelBase = 0.258;      % Distance between wheels of iRobot Create
        wheelDiameter= 0.072;
        wheelEncoderCount= 508.8;
        rangeIR= 0.1;           % Linear range of the infrared sensor
        rangeSonar= 3;          % Linear range of all sonar sensors
        rangeMinSonar= 0.02;    % Minimum linear range of sonar sensor
        rangeLidar= 4;          % Linear range of LIDAR sensor
        rangeMinLidar= 0.02;    % Minimum linear range of LIDAR sensor
        angRangeLidar= pi*240/180;      % Angular range of LIDAR sensor
        numPtsLidar= 681;       % Number of points in LIDAR sensing range
        rangeCamera = 6;        % Linear range of the camera for blob detection
        angRangeCamera = pi*60/180; % Angular range of camera (each side)
        cameraDisplace = 0.13;  % Position of camera along robot's x-axis
        frictionKin= 0.35;    %  Coefficient of kinetic friction of robot and wall
        Create2= 1;           % WRS
        cliffSensorLocations=[-60,-10,10,60]*pi/180;
        lightBumperLocations=[-45,-15,-5,10,30,75]*pi/180;
        bumperLocations=[-67.5,-22.5;-22.5,22.5;22.5,67.5]*pi/180;
        lightBumperThreshold = 2000;
        lightBumperSignalEquation = @(d)round(4000.*exp(-17.5.*d)); % TODO refine
        lightBumperMinRange = 0.01; % TODO implement
        cliffVoid = 10;
        cliffBlack = 1000;
        cliffBWThreshold = 1500;
        cliffWhite = 2200;
    end
    
    %properties(GetAccess= 'private', SetAccess= 'private')
    properties
        % Values require get and set methods to view or manipulate
        
        % Sensor variables
        odomDist;   % (double) Distance traveled since last check from odometry
        odomAng;    % (double) Angle turned since last check from odometry
        encoderCountLeft; % (uint16) current left wheel encoder count
        encoderCountRight; % (uint16) current right wheel encoder count
        noise;      % (stucture) Contains noise data for sensors used
        % fieldnames are sensor names and
        % field values are sensor noise [mean standard_dev]
        comDelay;   % (double) Communication delay of commands to robot
        
        % Robot state variables
        posAbs;     % (double) [x,y] Position in absolute coordinates
        velAbs;     % (double( [vx,vy] Velocity in absolute coordinates
        thAbs;      % (double) Yaw angle relative to positive x-axis, between -pi and pi
        wAbs;       % (double) Angular velocity, positive counter-clockwise
        velInt;     % (double) Forward velocity intended by motor commands
        omegaInt;   % (double) Angular velocity intended by motor commands
        songs;      % Currently defined songs
        port;       % communications port
        wheelVel;    % wheel velocities
        LEDs;
        buttons = struct('clock',0,'schedule',0,'day',0,'hour',0,'minute',0,'dock',0,'spot',0,'clean',0);
        bCommand;
        cmdTimer;
        % Environment variables
        timeElap;   % (uint64) Time of the start of autonomous code execution
        % time syntax - to be used with toc function
        figure; % Handle to GUI figure
        % Format: structure containing handle numbers
        %   See SimulatorGUI.fig and SimulatorGUI.m for more
        %   format information on the contents of the structure
        mapStart; % (double) [x y th] Contains robot start position/orientation information
        mapWalls; % Contains obstacle (wall) start and end points
        % Format: matrix of doubles, columns [x1 y1 x2 y2]
        mapLines;   % Contains line start and endpoints
        % Format: matrix of doubles, columns [x1 y1 x2 y2]
        mapBeacs;   % Containing beacon location, color, and ID information
        % Format: cell array of doubles,
        %   columns {x y red green blue ID}
        mapVWalls;  % Contains virtual wall location, direction, and
        %   strength information
        % Format: matrix of doubles, columns [x y th strength]
        mapLinesImg; % matlab 2D image with line data
        mapLinesImgSc;
        mapLinesImgOrig;
        worldLimits; % limits of the 'world'
    end
    
    methods(Static, Access= 'public')
        % Functions that do not need the object defined to operate
        
        function R=iff(condition,trueval,falseval)
            if condition
                R=trueval;
            else
                R=falseval;
            end
        end % iff
        
        function newAng= wrap2pi(oldAng)
            % newAng = wrap2pi(oldAng)
            % Wrap angle in radians to [-pi pi]
            % Replace wrapToPi to avoid dependence on mapping toolbox
            %
            % Input:
            % oldAng - Angle to be wrapped within limits (rad)
            %
            % Output:
            % newAng - Output angle within limits (rad)
            
            % Increase if to low
            while oldAng < -pi
                oldAng= oldAng+2*pi;
            end
            
            % Decrease if too high
            while oldAng > pi
                oldAng= oldAng-2*pi;
            end
            newAng= oldAng;
        end
        
        function R = playSongWave(data)
            s = [];
            for i=1:2:length(data)
                s = [s roombaSim.getNoteWave(data(i),data(i+1))];
            end
            sound(s,14400);
            %a = audioplayer(s,14400);
            %play(a)
            R = length(s)/14400;
        end
        
        function finishWave(t,~,obj)
            obj.songs.playing=0;
            disp('song finished');
            stop(t);
            delete(t);
        end
        
        function a=getNoteWave(note, duration)
            % play a Roomba note
            % note is a Roomba reference number (69 is A4)
            % duration is in 64/s of a second
            
            % nested function
            function a = notetone(hz, duration, sps)
                % generate data for a given tone
                N = floor(sps * duration);
                i = 0:N;
                a = sin (2 * pi * i * hz / sps);
            end
            
            duration=duration/64; % a quarter note is 1/4 of a second
            if note==255
                hz=0;
            else
                hz = 440 * 2^( (note-69) / 12);
            end
            sps = 14400;
            
            % combine three harmonics for a better sound
            mid = notetone(1.0*hz, duration, sps);
            hi  = notetone(2.0*hz, duration, sps);
            lo  = notetone(0.5*hz, duration, sps);
            a = .5*mid + .25*hi + .25*lo;
            % a(end-1000:end)=0; % put a small gap at the end
        end % getNoteWave
        
        
    end
    
    methods(Access= 'public')
        % Functions available to call from controller function or other files
        
        % Constructor Function
        function obj= roombaSim(varargin)
            % obj = roombaSim
            % Creates instance of the user-defined class roombaSim and
            % initializes all properties. Note that if roombaSim is not
            % called by SimulatorGUI (with handles argument), full
            % functionality impossible.
            %
            % obj = roombaSim(fig,port)
            % Format of function call from SimulatorGUI. Passes along structure
            % of handles for the GUI to allow full functionality
            %
            % Input:
            %   fig - handle to figure window
            %
            % Output:
            % obj - Instance of class roombaSim with all fields initialized
            
            % Deal with input argument
            % Will be handles to SimulatorGUI if called by simulator
            if ~isempty(varargin)
                obj.figure= varargin{1};
            else
                obj.figure= [];
            end
            
            % Assign properties
            walls = [
                -2,-2, 2,-2
                2,-2, 2, 2
                2, 2,-2, 2
                -2, 2,-2,-2
                ];
            th = linspace(0,2*pi);
            x = cos(th);
            y = sin(th);
            lines = [x(1:end-1); y(1:end-1); x(2:end); y(2:end)]';
            obj.setMapStart([0,0,0]);   % (double) [x y th] Contains robot start position/orientation information
            obj.setMap(walls,lines,[],[]);
            obj.timeElap= [];
            obj.noise.wall = [0 ,0.01];
            obj.noise.cliff = [3, 6.90];
            obj.noise.odometry = [-0.02, 0.06];
            obj.noise.sonar = [0, 0.004];
            obj.noise.lidar = [0, 0.001];
            obj.noise.camera = [0, 0.02];
            obj.comDelay= 0.05;
            obj.posAbs= obj.mapStart(1:2); % Initial position
            obj.velAbs= [0 0]; % Assume robot is stationary to start
            obj.thAbs= obj.mapStart(3);
            obj.wAbs= 0;
            obj.velInt= 0;
            obj.wheelVel=[0,0];
            obj.omegaInt= 0;
            obj.odomDist= 0;        % Start odometry at zero
            obj.odomAng= 0;
            obj.encoderCountLeft= 0;
            obj.encoderCountRight= 0;
            obj.songs.data = {[],[],[],[]};
            obj.songs.playing=1;
            obj.songs.current=0;
            obj.worldLimits=[-2 2 -2 2];
            obj.generateAsciiSegCodes();
            
            obj.cmdTimer= timer;
            %obj.cmdTimer.BusyMode= 'drop';
            obj.cmdTimer.ExecutionMode= 'fixedSpacing';
            obj.cmdTimer.Period= 0.02;
            obj.cmdTimer.TasksToExecute= inf;
            obj.cmdTimer.StartDelay=0;
            obj.cmdTimer.TimerFcn= {@obj.processCommand,obj};
            start(obj.cmdTimer);
        end
        
        %destructor function
        function delete(obj)
            delete(obj.cmdTimer);
            fclose(obj.port);
            delete(obj.port);
        end
        
        function R = getWorldLimits(obj)
            R=obj.worldLimits;
        end
        
        function initPosition(obj)
            obj.posAbs=[0,0];
            obj.velAbs=[0,0];
            obj.thAbs=0;
            obj.wAbs=0;
            obj.wheelVel=[0,0];
            obj.velInt = 0;
            obj.omegaInt = 0;
        end
        
        function startServer(obj)
            obj.port= tcpip('localhost',2217);
            set(obj.port,'NetworkRole', 'server');
            set(obj.port,'Terminator','');
            set(obj.port,'InputBufferSize',1024)
            set(obj.port,'OutputBufferSize',1024)
            set(obj.port, 'Timeout', 1)
            set(obj.port, 'ByteOrder','bigEndian');
            set(obj.port, 'Tag', 'RoombaSimulator');
            fclose(fopen('server.tmp','w'));
            disp('waiting for connection...')
            %set(obj.port,'BytesAvailableFcnCount',1);
            %set(obj.port,'BytesAvailableFcnMode','byte');
            %set(obj.port,'BytesAvailableFcn', {@obj.processCommand,obj});
            %set(obj.port, 'ReadAsyncMode', 'continuous');
            fopen(obj.port);
            disp('connection made');
        end %startServer
        
        % SimulatorGUI Control Functions
        function [rad,rIR,rSon,rLid,angRLid,numPtsLid]= getConstants(obj)
            % [rad rIR rSon rLid angRLid numPtsLid] = getConstants(obj)
            % Output constant properties of the robot for simulator usage
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % rad - Double, radius of robot (m)
            % rIR - Double, range of infrared wall sensor (m)
            % rLid - Double, linear range of LIDAR sensor (m)
            % angRLid - Double, angular range of LIDAR sensor (rad)
            % numPtsLid - Double, number of points used by LIDAR sensor
            
            rad= obj.radius;
            rIR= obj.rangeIR;
            rSon= obj.rangeSonar;
            rLid= obj.rangeLidar;
            angRLid= obj.angRangeLidar;
            numPtsLid= obj.numPtsLidar;
        end
        
        function processCommand(obj,varargin)
            if isempty(obj.port)
                disp('starting server in processCommand');
                obj.startServer();
            end
            if obj.bCommand
                disp('in processCommand with command already active');
                return;
            end
            if obj.port.BytesAvailable==0
                %disp('processCommand, nothing waiting');
                return;
            end
            obj.bCommand=true;
            cmd = obj.getData();
            fprintf('processCommand cmd: %u\n',cmd);
            if ~isempty(cmd)
                obj.doCommand(cmd);
            end
            obj.bCommand=false;
        end % processCommand
        
        function [start,walls,lines,beacs,vwalls]= getMap(obj)
            % [start walls lines beacs vwalls] = getMap(obj)
            % Output the obstacle and robot map data for plotting
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % start - Vector containing robot start position information
            % walls - Matrix containing obstacle information
            % lines - Matrix containing line information
            % beacs - Cell array containing beacon information
            % vwalls - Matrix containing virtual wall information
            %   See the properties specification for format information
            
            % Assign variables from object values
            start= obj.mapStart;
            walls= obj.mapWalls;
            lines= obj.mapLines;
            beacs= obj.mapBeacs;
            vwalls= obj.mapVWalls;
        end
        
        function setMap(obj,walls,lines,beacs,vwalls)
            % setMap(obj,walls,lines,beacs,vwalls)
            % Change the obstacle map data in the robot object
            %
            % Input:
            % obj - Instance of class roombaSim
            % walls - % Matrix containing obstacle information
            % lines - % Matrix containing line information
            % beacs - % Cell array containing beacon information
            % vwalls - % Matrix containing virtual wall information
            %   See properties specification for format information
            
            % Save map information to object
            obj.mapWalls= walls;
            obj.mapLines= lines;
            obj.mapBeacs= beacs;
            obj.mapVWalls= vwalls;
            obj.createLineImg();
        end
        
        
        function setMapStart(obj,origin)
            % setMapStart(obj,origin)
            % Change the mapStart data in the robot object. Move the robot to
            % that position.
            %
            % Input:
            % obj - Instance of class roombaSim
            % origin - Vector of doubles, containing information about the
            %   robot start position in the format [x y th]
            
            obj.mapStart= origin;
            obj.posAbs= origin(1:2);
            obj.thAbs= origin(3);
        end
        
        
        
        
        function setComDelay(obj,tDelay)
            % setComDelay(obj,tDelay)
            % Change the communication delay property in the robot object
            % This delay will occur on every function call from the
            % autonomous program
            %
            % Input:
            % obj - Instance of class roombaSim
            % tDelay - Double, communication delay (s)
            
            obj.comDelay= tDelay;
        end
        
        
        
        
        function updateSensorVisualization(obj)
            % updateSensorVisualization(obj,handles,handles.sensors)
            % Updates the data used to plot the sensors visual representation
            %
            % Input:
            % obj - Instance of class roombaSim
            
            handles = guidata(obj.figure);
            
            % LightBumpers TODO finish
            if get(handles.chkbx_lightbumper,'Value')
                LBdata= genLightBumper(obj);
                for i= 1:6
                    th_sensor= obj.thAbs+obj.lightBumperLocations(i);
                    xt_sensor= obj.posAbs(1)+obj.radius*1.1*cos(th_sensor);
                    yt_sensor= obj.posAbs(2)+obj.radius*1.1*sin(th_sensor);
                    
                    set(handles.sensors.lightBumpers(i)...
                        ,'Position',[xt_sensor ,yt_sensor] ...
                        ,'String',num2str(LBdata(i)) ...
                        ,'Rotation',th_sensor*180/pi ...
                        ,'Color',obj.iff(LBdata(i)>=obj.lightBumperThreshold,'m','k') ...
                        );
                end
            end
            
            % Bump sensors
            if get(handles.chkbx_bump,'Value')
                x_robot= obj.posAbs(1);
                y_robot= obj.posAbs(2);
                th_robot= obj.thAbs;
                bump = obj.genBump();
                for b=1:3
                    if bump(b)
                        set(handles.sensors.bumpers(b),'XData',...
                            x_robot+obj.radius.*cos(th_robot+obj.bumperLocations(b,:)));
                        set(handles.sensors.bumpers(b),'YData',...
                            y_robot+obj.radius.*sin(th_robot+obj.bumperLocations(b,:)));
                        set(handles.sensors.bumpers(b),'Visible','on')
                    else
                        set(handles.sensors.bumpers(b),'Visible','off')
                    end
                end
            end
            
            % Cliff sensors
            if get(handles.chkbx_cliff,'Value')
                cliff= genCliff(obj);
                x_robot= obj.posAbs(1);
                y_robot= obj.posAbs(2);
                th_robot= obj.thAbs;
                th_sensors=th_robot+obj.cliffSensorLocations;
                x_sens = x_robot + obj.radius.*cos(th_sensors);
                y_sens = y_robot + obj.radius.*sin(th_sensors);
                
                for cs=1:4
                    set(handles.sensors.cliffSensors(cs)...
                        ,'Position',[x_sens(cs) ,y_sens(cs)] ...
                        ,'String',num2str(cliff(cs)) ...
                        ,'Rotation',th_sensors(cs)*180/pi ...
                        ,'Color',obj.iff(cliff(cs)<=obj.cliffBWThreshold,'m','k') ...
                        );
                end
            end
            
            
        end
        function R = encodeButtons(obj)
            % Encode a value that contains button press information
            
            R=0;
            % note - MATLAB numbers bits from 1 to 8
            R = bitset(R,8,obj.buttons.clock);
            R = bitget(R,7,obj.buttons.schedule);
            R = bitget(R,6,obj.buttons.day);
            R = bitget(R,5,obj.buttons.hour);
            R = bitget(R,4,obj.buttons.minute);
            R = bitget(R,3,obj.buttons.dock);
            R = bitget(R,2,obj.buttons.spot);
            R = bitget(R,1,obj.buttons.clean);
        end % encodeButtons
        
        function startTimeElap(obj)
            % startTimeElap(obj)
            % Start the timer to determine time steps for data output
            %
            % Input:
            % obj - Instance of class roombaSim
            
            obj.timeElap= tic;
        end
        
        
        
        % Sensor Functions
        function bump= genBump(obj)
            % bump = genBump(obj)
            % Generates a reading for all bump sensors
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % bump - Vector of Boolean doubles [right front left],
            %   1 indicates the sensor is activated, 0 is not activated
            
            % Get intersections with walls
            collPts= findCollisions(obj);
            % Check that there are intersections
            bump= [0 0 0];    % Initialize to no bumps
            if ~isempty(collPts)
                % Change intersection points to local coordinates
                x_int= collPts(:,1)-obj.posAbs(1);
                y_int= collPts(:,2)-obj.posAbs(2);
                
                % Get angle relative to robot front
                th_int= obj.wrap2pi(atan2(y_int,x_int)-obj.thAbs);
                
                % Find sensor activated, if any
                for i= 1:length(th_int)
                    if th_int(i) < -0.135*pi && th_int(i) > -pi/2
                        bump(1)= 1;  % Right sensor
                    elseif th_int(i) < 0.135*pi && th_int(i) > -0.135*pi
                        bump(2)= 1;  % Front sensor (both left and right)
                    elseif th_int(i) < pi/2 && th_int(i) > 0.135*pi
                        bump(3)= 1;  % Left sensor
                    end
                end
                
                % Ensure that front sensor doesn't read with other two
                if bump(2)
                    bump(1)= 0;
                    bump(3)= 0;
                elseif bump(1) && bump(3)
                    bump(1)= 0;
                    bump(2)= 1;
                    bump(3)= 0;
                end
            end
        end
        
        function LB=genLightBumper(obj)
            % Output:
            % LB - Vector of doubles [left,left front, left center, right
            %   center, right front, right, combo]
            %   the first 6 are the actual signal value (0-4095)
            %   and the 7th is the binary combination of on/off values
            
            % Cycle through all sensors
            LB = ones(1,7);    % Preallocate for speed
            for i= 1:6
                % Calculate position and orientation of the sensor
                % Assume sensors are at the edge of the robot
                th_sensor= obj.thAbs+obj.lightBumperLocations(i);
                x_sensor= obj.posAbs(1) + obj.radius*cos(th_sensor);
                y_sensor= obj.posAbs(2) + obj.radius*sin(th_sensor);
                
                % Get noise value to change reading of sonar
                % noiseVal= noiseAvg+noiseStDev*randn;
                
                % Solve for distance using general function
                d= findDist(obj,x_sensor,y_sensor,999,th_sensor);
                LB(i) = obj.lightBumperSignalEquation(d);
                
                % Do not add sensor noise if sensor saturated to max value
                % Or if it is less than the minimum range
                % if (distSonar(i)<obj.rangeSonar)&&(distSonar(i)>obj.rangeMinSonar)
                % Compute bounds on the sensor noise such that noise
                % cannot cause sensor saturation in either direction
                % (max or min).
                % NOTE: This results in noise that is not purely Gaussian
                
                %    maxNoiseVal = obj.rangeSonar - distSonar(i) - 0.0001;
                %    minNoiseVal = obj.rangeMinSonar - distSonar(i) + 0.0001;
                
                % Saturate noisey sonar measurement between these values
                %    noiseVal = min(max(noiseVal,minNoiseVal),maxNoiseVal);
                
                %    distSonar(i) = distSonar(i)+noiseVal;
            end
            % LB = randi([0 4095],[1,6]); % TODO fix
            LB(7) = sum( (LB(1:6)>=obj.lightBumperThreshold).*2.^[0:5]);
            % TODO - add noise
        end % genLightBumper
        
        function cliff= genCliff(obj)
            % cliff = genCliff(obj)
            % Generates a reading for the cliff sensors
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % cliff - Vector of doubles [right front-right front-left left],
            %   high values if there are no lines, low values for lines
            
            cliff= ones(1,4).*obj.cliffWhite;   % Initialize to no lines (white)
            
            if size(obj.mapLines,1)>0
                x_rob= obj.posAbs(1);
                y_rob= obj.posAbs(2);
                th_rob= obj.thAbs;
                rad= obj.radius;
                
                x_sens = x_rob + rad.*cos(th_rob+obj.cliffSensorLocations);
                y_sens = y_rob + rad.*sin(th_rob+obj.cliffSensorLocations);
                
                [xx,yy] = obj.mapLinesImgConvert(x_sens,y_sens);
                for i=1:4
                    if obj.mapLinesImg(yy(i),xx(i))
                        cliff(i)=obj.cliffBlack;
                    end
                end
            end
            % Get noise to change the reading of the sensors
            noiseAvg= obj.noise.cliff(1);
            noiseStDev= obj.noise.cliff(2);
            cliff = round(cliff+(noiseAvg+noiseStDev*randn(size(cliff))));
            
        end % genCliff.
        
        function vwall= genVWall(obj)
            % vwall = genVWall(obj)
            % Generates a reading for the virtual wall sensor
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % vwall - Boolean double, 1 if the robot is within the field of a
            % virtual wall, or the "halo" surrounding the emitter
            
            % Get sensor position
            x_sensor= obj.posAbs(1)+obj.radius*cos(obj.thAbs);
            y_sensor= obj.posAbs(2)+obj.radius*sin(obj.thAbs);
            
            % Define virtual wall emitter constants
            halo_rad= 0.45;     % Radius of the halo around the emitter
            range_short= 2.13;  % Range of the wall on the 0-3' setting
            ang_short= 0.33;    % Angular range on the 0-3' setting
            range_med= 5.56;    % Range of the wall on the 4'-7' setting
            ang_med= 0.49;      % Angular range on the 4'-7' setting
            range_long= 8.08;   % Range of the wall on the 8'+ setting
            ang_long= 0.61;     % Angular range on the 8'+ setting
            
            % Check against all virtual wall ranges
            vwall= 0;   % Initialize output to no-wall
            for i= 1:size(obj.mapVWalls,1)
                % Get emitter position
                x_emit= obj.mapVWalls(i,1);
                y_emit= obj.mapVWalls(i,2);
                
                % Check if sensor is within halo
                if sqrt((x_sensor-x_emit)^2+(y_sensor-y_emit)^2) < halo_rad
                    vwall_seen= 1;
                else
                    % Get more emitter constants
                    th= obj.mapVWalls(i,3);
                    if obj.mapVWalls(i,4) == 1
                        range= range_short;
                        ang= ang_short;
                    elseif obj.mapVWalls(i,4) == 2
                        range= range_med;
                        ang= ang_med;
                    else
                        range= range_long;
                        ang= ang_long;
                    end
                    
                    % Find points that define boundary of virtual wall
                    x_1= x_emit+range*cos(th+ang/2);
                    y_1= y_emit+range*sin(th+ang/2);
                    x_2= x_emit+range*cos(th-ang/2);
                    y_2= y_emit+range*sin(th-ang/2);
                    
                    % Find if sensor is within virtual wall triangle
                    % Use determinant definition of the area of a triangle
                    area_vwall= 0.5*abs(det([x_1 y_1 1 ; x_2 y_2 1 ; ...
                        x_emit y_emit 1]));
                    area_1= 0.5*abs(det([x_1 y_1 1 ; x_2 y_2 1 ; ...
                        x_sensor y_sensor 1]));
                    area_2= 0.5*abs(det([x_emit y_emit 1 ; x_2 y_2 1 ; ...
                        x_sensor y_sensor 1]));
                    area_3= 0.5*abs(det([x_1 y_1 1 ; x_emit y_emit 1 ; ...
                        x_sensor y_sensor 1]));
                    area_tot= area_1+area_2+area_3;
                    vwall_seen= abs(area_tot-area_vwall) < 0.001;
                end
                
                % Check for walls between robot and emitter
                dist_emit= sqrt((x_emit-x_sensor)^2+(y_emit-y_sensor)^2);
                dist_vwall= findDist(obj,x_sensor,y_sensor,dist_emit,...
                    atan2(y_emit-y_sensor,x_emit-x_sensor));
                if vwall_seen && dist_vwall == dist_emit
                    vwall= 1;
                end
            end
        end
        
        function distSonar= genSonar(obj)
            % distSonar = genSonar(obj)
            % Generates a reading for all sonar sensors
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % distSonar - Vector of doubles [front left back right],
            %   distance along each line of sight to nearest obstacle
            
            % Get noise parameters
            noiseAvg= obj.noise.sonar(1);
            noiseStDev= obj.noise.sonar(2);
            
            % Cycle through all sensors
            distSonar= obj.rangeSonar*ones(1,4);    % Preallocate for speed
            for i= 1:4
                % Calculate position and orientation of the sensor
                % Assume sensors are at the edge of the robot
                th_sensor= obj.thAbs+(i-1)*pi/2;
                x_sensor= obj.posAbs(1); %+obj.radius*cos(th_sensor);
                y_sensor= obj.posAbs(2); %+obj.radius*sin(th_sensor);
                
                % Get noise value to change reading of sonar
                noiseVal= noiseAvg+noiseStDev*randn;
                
                % Solve for distance using general function
                distSonar(i)= findDist(obj,x_sensor,y_sensor,...
                    obj.rangeSonar+obj.radius,th_sensor);
                
                distSonar(i) = distSonar(i)-obj.radius;
                % Do not add sensor noise if sensor saturated to max value
                % Or if it is less than the minimum range
                if (distSonar(i)<obj.rangeSonar)&&(distSonar(i)>obj.rangeMinSonar)
                    % Compute bounds on the sensor noise such that noise
                    % cannot cause sensor saturation in either direction
                    % (max or min).
                    % NOTE: This results in noise that is not purely Gaussian
                    
                    maxNoiseVal = obj.rangeSonar - distSonar(i) - 0.0001;
                    minNoiseVal = obj.rangeMinSonar - distSonar(i) + 0.0001;
                    
                    % Saturate noisey sonar measurement between these values
                    noiseVal = min(max(noiseVal,minNoiseVal),maxNoiseVal);
                    
                    distSonar(i) = distSonar(i)+noiseVal;
                end
            end
        end
        
        function distLidar= genLidar(obj)
            % distLidar = genLidar(obj)
            % Generates a reading for the LIDAR sensor
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % distLidar - Vector of doubles of length obj.numPtsLidar
            %   Distances correspond to angles [-angRangeLidar/2 angRangeLidar/2]
            %   Remember that angles are defined positive counter-clockwise
            
            % Calculate position of sensor (same for all angles)
            x_sensor= obj.posAbs(1)+obj.radius*cos(obj.thAbs);
            y_sensor= obj.posAbs(2)+obj.radius*sin(obj.thAbs);
            
            % Get noise parameters
            noiseAvg= obj.noise.lidar(1);
            noiseStDev= obj.noise.lidar(2);
            
            % Cycle through all points of sensing
            distLidar= obj.rangeLidar*ones(1,obj.numPtsLidar);
            for i= 1:obj.numPtsLidar
                % Find orientation of line of sight
                th_sensor= obj.thAbs+(i-1)*obj.angRangeLidar/...
                    (obj.numPtsLidar-1)-obj.angRangeLidar/2;
                
                % Get noise value to change reading of LIDAR
                noiseVal= noiseAvg+noiseStDev*randn;
                
                % Solve for distance using general function
                distLidar(i)= findDist(obj,x_sensor,y_sensor,...
                    obj.rangeLidar,th_sensor)+noiseVal;
                
                % Set any readings below minimum to the minimum
                distLidar(i)= max(distLidar(i),obj.rangeMinLidar);
            end
        end
        
        function [ang,dist,color,id]= genCamera(obj)
            % [ang dist color id] = genCamera(obj)
            % Generates the output from the blob detection on the camera,
            % detects only beacons
            % Camera is located at 'cameraDisplace' distance (+0.13m)
            % along the robot's x-axis. Same as lab setup.
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % ang - Vector of doubles, each the angle relative to robot at
            %   which beacon is detected
            % dist - Vector of doubles, each the distance of beacon from camera
            % color - Matrix of doubles of width three (color vector),
            %   each row the color of beacon detected
            % id  - AR tag ID
            
            % Get robot position and orientation
            x_r= obj.posAbs(1);
            y_r= obj.posAbs(2);
            th_r= obj.thAbs;
            
            % Get camera position and orientation
            x_c = x_r + obj.cameraDisplace*cos(th_r);
            y_c = y_r + obj.cameraDisplace*sin(th_r);
            th_c = th_r;
            
            % Check each beacon against camera ranges
            ang= [];
            dist= [];
            color= [];
            id = [];
            
            for i= 1:size(obj.mapBeacs,1)   % Go through all the beacons
                % Get beacon position
                x_b   = obj.mapBeacs{i,1};
                y_b   = obj.mapBeacs{i,2};
                clr_b = [obj.mapBeacs{i,3},obj.mapBeacs{i,4},obj.mapBeacs{i,5}];
                id_b  = str2double(obj.mapBeacs{i,6});
                
                % Find heading and distance from camera to beacon
                ang_b= obj.wrap2pi(atan2(y_b-y_c,x_b-x_c)-th_c);
                dist_b= sqrt((x_b-x_c)^2+(y_b-y_c)^2);
                
                % See if there is a wall in the way (blocks field-of-view)
                walldist= findDist(obj,x_c,y_c,dist_b, ...
                    obj.wrap2pi(ang_b+th_c));
                
                % If camera can see beacon, then save beacon information
                if abs(ang_b) <= obj.angRangeCamera && ...
                        dist_b <= obj.rangeCamera && ...
                        walldist == dist_b
                    ang= [ang ; ang_b];
                    dist= [dist ; dist_b];
                    color= [color ; clr_b];
                    id = [id;id_b];
                end
            end
            
            % Add noise
            if ~isempty(ang)
                noiseAvg= obj.noise.camera(1);
                noiseStDev= obj.noise.camera(2);
                ang= ang+noiseAvg+noiseStDev*randn(length(ang),1);
                dist= dist+noiseAvg+noiseStDev*randn(length(ang),1);
            end
        end
        
        function dist= genOdomDist(obj)
            % dist = genOdomDist(obj)
            % Determines how far the robot has traveled since the last call
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % dist - Double, distance traveled since last call from odometry
            
            % Extract property
            % Noise is already added
            dist= obj.odomDist;
            
            % Wrap to limits
            if dist < -32.768
                dist= -32.768;
                disp('Simulator:overflow')
                disp('Return value capped at minimum')
            elseif dist > 32.768
                dist= 32.768;
                disp('Simulator:overflow')
                disp('Return value capped at maximum')
            end
            
            % Reset sensor to record distance since this call
            obj.odomDist= 0;
        end
        
        function ang= genOdomAng(obj)
            % ang = genOdomAng(obj)
            % Determines how far the robot has turned since the last call
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % ang - Double, angle turned since last call from odometry,
            %   positive counter-clockwise
            
            % Extract property
            % Noise is already added
            ang= obj.odomAng;
            
            % Wrap to limits
            if ang < -571
                ang= -571;
                disp('Simulator:overflow')
                disp('Return value capped at minimum')
            elseif ang > 571
                ang= 571;
                disp('Simulator:overflow')
                disp('Return value capped at maximum')
            end
            
            % Reset sensor to record angle since this call
            obj.odomAng= 0;
        end
        
        function [left,right] = genEncoderCount(obj)
            left=obj.encoderCountLeft;
            right=obj.encoderCountRight;
        end
        
        function E = distToEncoder(obj,enc,dist)
            E = enc + round(dist/(pi*obj.wheelDiameter)*obj.wheelEncoderCount);
            if E>32767, E=E-65536; end;
            if E<-32768, E=E+65536; end;
        end
        
        function updateOdom(obj,oldstate,newstate)
            % updateOdom(obj,oldstate,newstate)
            % Updates the odometry properties in the robot object
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Input:
            % oldstate - Vector of doubles [x y th v w], values of state from
            %   previous time step
            % newstate - Vector of doubles [x y th v w], values of current state
            
            % Extract values
            x_o= oldstate(1);
            y_o= oldstate(2);
            th_o= oldstate(3);
            v_o= oldstate(4:5);
            w_o= oldstate(6);
            x_n= newstate(1);
            y_n= newstate(2);
            th_n= newstate(3);
            
            % Get noise parameters
            noiseAvg= obj.noise.odometry(1);
            noiseStDev= obj.noise.odometry(2);
            
            % Distance sensor
            dist= sqrt((x_n-x_o)^2+(y_n-y_o)^2);    % Distance traveled
            dir= sign(dot(v_o,[cos(th_o) sin(th_o)]));  % For/Backwards
            noiseVal= (noiseAvg+noiseStDev*randn)*dist;
            deltaDist = dir*dist+noiseVal;
            obj.odomDist= obj.odomDist + deltaDist;
            
            % Angle sensor
            % Assume small turning angles, and account for cases such as
            % turning to/from 1st/4th quadrant
            turn= min(abs(th_n-th_o),abs(th_n+th_o));
            dir= sign(w_o);
            noiseVal= (noiseAvg+noiseStDev*randn)*turn;
            deltaAng= dir*turn+noiseVal;
            obj.odomAng= obj.odomAng + deltaAng;
            
            % distance of wheel travel based on angle
            angDist = deltaAng*(obj.wheelBase/2);
            
            obj.encoderCountLeft = obj.distToEncoder(obj.encoderCountLeft, deltaDist - angDist);
            obj.encoderCountRight = obj.distToEncoder(obj.encoderCountRight, deltaDist + angDist);
            
            handles = guidata(obj.figure);
            set(handles.data_encleft,'string',sprintf('Left #: %.0f',obj.encoderCountLeft));
            set(handles.data_encright,'string',sprintf('Right #: %.0f',obj.encoderCountRight));
            set(handles.data_dist,'string',sprintf('Dist: %.3f',obj.odomDist));
            set(handles.data_ang,'string',sprintf('Ang: %.3f',obj.odomAng));
            drawnow;
            
        end
        
        % Computational Functions
        function dist= findDist(obj,x_sensor,y_sensor,range,th)
            % dist = findDist(obj,x_sensor,y_sensor,range,th)
            % Finds the distance a sensor is measuring at a certain angle
            %
            % Input:
            % obj - Instance of class roombaSim
            % x_sensor - X-coordinate of the sensor position
            % y_sensor - Y-coordinate of the sensor position
            % range - Linear range of the sensor
            % th - Angle the sensor is investigating in absolute coordinates
            
            % Output:
            % dist - Linear distance along the line of sight of the sensor to
            % 	the closest obstacle
            
            % Create line of sight
            x_range= x_sensor+range*cos(th);   % Range of sensor
            y_range= y_sensor+range*sin(th);
            
            % Find line equation for sensor line
            m_sensor= (y_range-y_sensor)/(x_range-x_sensor);
            if m_sensor > 1e14
                m_sensor= inf;
            elseif abs(m_sensor) < 1e-14
                m_sensor= 0;
            elseif m_sensor < -1e14
                m_sensor= -inf;
            end
            b_sensor= y_sensor-m_sensor*x_sensor;
            
            % Check against every obstacle (individually)
            j= 1;                   % Count variable for intersections
            x_int= [];              % Position of intersections
            y_int= [];
            for i= 1:size(obj.mapWalls,1)% Count variable for obstacles
                % Find line equations for wall lines
                m_wall= (obj.mapWalls(i,4)-obj.mapWalls(i,2))/...
                    (obj.mapWalls(i,3)-obj.mapWalls(i,1));
                if m_wall > 1e14
                    m_wall= inf;
                elseif abs(m_wall) < 1e-14
                    m_wall= 0;
                elseif m_wall < -1e14
                    m_wall= -inf;
                end
                b_wall= obj.mapWalls(i,2)-m_wall*obj.mapWalls(i,1);
                
                % Find intersection of infinitely long walls
                if ~(m_sensor == m_wall)    % Not parallel lines
                    if isinf(m_sensor)      % Vertical sensor line
                        x_hit= x_sensor;
                        y_hit= m_wall*x_hit+b_wall;
                    elseif isinf(m_wall)    % Vertical wall line
                        x_hit= obj.mapWalls(i,1);
                        y_hit= m_sensor*x_hit+b_sensor;
                    else                    % Normal conditions
                        x_hit= (b_wall-b_sensor)/(m_sensor-m_wall);
                        y_hit= m_sensor*x_hit+b_sensor;
                    end
                    
                    % Verify that intersection is on finite lines
                    % Use tolerances to account for rounding errors on
                    % vertical or horizontal lines
                    if x_hit-min(x_sensor,x_range) > -0.001 && ...
                            x_hit-max(x_sensor,x_range) < 0.001 && ...
                            y_hit-min(y_sensor,y_range) > -0.001 && ...
                            y_hit-max(y_sensor,y_range) < 0.001 && ...
                            x_hit-min(obj.mapWalls(i,[1 3])) > -0.001 && ...
                            x_hit-max(obj.mapWalls(i,[1 3])) < 0.001 && ...
                            y_hit-min(obj.mapWalls(i,[2 4])) > -0.001 && ...
                            y_hit-max(obj.mapWalls(i,[2 4])) < 0.001
                        x_int(j)= x_hit;
                        y_int(j)= y_hit;
                        j= j+1;
                    end
                end
            end
            
            % Find closest wall on sensor line
            dist= range;    % Initialize to max range
            if ~isempty(x_int)
                distVec= sqrt((x_int-x_sensor).^2+(y_int-y_sensor).^2);
                dist= min(distVec);  % Find shortest distance to intersections
            end
        end
        
        function collPts= findCollisions(obj)
            % collPts = findCollisions(obj)
            % Check if the robot intersects any walls
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % collPts - Matrix of doubles [x y i f]
            %   x - x-coordinate of closest point to the center of the robot
            %   y - y-coordinate of closest point to the center of the robot
            %   i - Index of wall, used as obj.mapWalls(i,:)
            %   f - Corner flag, is 1 if intersection point is a corner
            %   An empty matrix means no collisions
            
            % Extract variables
            xR= obj.posAbs(1);  % Position of the robot center
            yR= obj.posAbs(2);
            rad= obj.radius; % Radius of the robot
            
            % Find nearest point on every wall
            collPts= [];
            for i= 1:size(obj.mapWalls,1)
                % Extract wall data
                x1= obj.mapWalls(i,1);
                y1= obj.mapWalls(i,2);
                x2= obj.mapWalls(i,3);
                y2= obj.mapWalls(i,4);
                
                % Assume wall is infinitely long
                m= (y2-y1)/(x2-x1);         % Slope of wall
                if isinf(m)                 % Vertical wall
                    x0= x1;
                    y0= yR;
                elseif m == 0               % Horizontal wall
                    x0= xR;
                    y0= y1;
                else                        % Normal conditions of wall
                    b= y1-m*x1;             % Intercept of wall
                    c= yR+xR/m; % Intercept of perpendicular line through robot
                    
                    % Calculate intersection point of two lines
                    x0= (c-b)/(m+1/m);
                    y0= (b-c)/(m^2+1)+c;
                end
                
                % Check if intersection point is not on finite wall
                endPt= 0;
                if x0 > max(x1,x2) || x0 < min(x1,x2) || ...
                        y0 > max(y1,y2) || y0 < min(y1,y2)
                    % Closest point will be nearest endpoint
                    dist1= sqrt((x1-xR)^2+(y1-yR)^2);
                    dist2= sqrt((x2-xR)^2+(y2-yR)^2);
                    if dist1 <= dist2
                        x0= x1;
                        y0= y1;
                    else
                        x0= x2;
                        y0= y2;
                    end
                    endPt= 1;   % Set corner flag
                end
                
                % Check if intersection point is within robot circle
                if sqrt((x0-xR)^2+(y0-yR)^2) <= rad
                    collPts= [collPts ; x0 y0 i endPt];
                end
            end
            
            % Only keep closest two collision points
            if size(collPts,1) > 2
                [distances, distIdx]= sort(sqrt((collPts(:,1)-xR).^2+...
                    (collPts(:,2)-yR).^2));
                collPts= collPts(distIdx(1:2),:);
            end
        end
        
        % State Manipulator Functions
        function state= getState(obj)
            % state = getState(obj)
            % Extracts current state properties for the simulation program
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % state - Vector of doubles [x y th vx vy w], values of current state
            
            % Put in output format
            state= [obj.posAbs obj.thAbs obj.velAbs obj.wAbs];
        end
        
        function setState(obj,state)
            % setState(obj,state)
            % Imports new state properties from the simulation program
            %
            % Input:
            % obj - Instance of class roombaSim
            % state - Vector of doubles [x y th v w], values of new state
            
            % Update robot object
            obj.posAbs= state(1:2);
            obj.thAbs= state(3);
            obj.velAbs= state(4:5);
            obj.wAbs= state(6);
        end
        
        function driveNormal(obj,tStep)
            % driveNormal(obj,tStep)
            % Updates the new position based on the current position and
            % velocities when no walls affect the robot
            %
            % Input:
            % obj - Instance of class roombaSim
            % tStep - Double, time since the previous state update
            
            % Get important values
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v= obj.velInt;
            w= obj.omegaInt;
            
            % Check zero-velocity cases to avoid inf and NaN values
            % The following code is taken from function
            % SimRobot by Jason Hardy and Francis Havlak
            if w == 0       % Straight path case
                x_new= x+v*cos(th)*tStep;
                y_new= y+v*sin(th)*tStep;
                th_new= th;
                vx= v*cos(th);
                vy= v*sin(th);
            elseif v == 0   % Turning only
                x_new= x;
                y_new= y;
                th_new= th+w*tStep;
                vx= v*cos(th);
                vy= v*sin(th);
            else            % Compute position along arc trajectory
                sign_v= sign(v);
                th_new= th+w*tStep;
                dth= th_new-th;
                dir= sign(dth);
                motionRad = v/w;
                %get relative motion
                l_chord= motionRad*2*sin(dth/2);
                y_rel= -dir*l_chord^2/2/motionRad;
                x_rel= sign_v*sqrt(l_chord^2-y_rel^2);
                %translate into global chords
                R= [cos(th) -sin(th) ; sin(th) cos(th)];
                pos_new= [x ; y]+R*[x_rel ; y_rel];
                x_new= pos_new(1);
                y_new= pos_new(2);
                vx= v*cos(th_new);
                vy= v*sin(th_new);
            end
            % End of code from SimRobot
            
            % Update position
            th_new= obj.wrap2pi(th_new);
            obj.posAbs= [x_new y_new];
            obj.thAbs= th_new;
            obj.velAbs= [vx vy];
            obj.wAbs= w;
        end
        
        function drive1Wall(obj,tStep,collPts)
            % drive1Wall(obj,tStep,collPts)
            % Updates the new position based on the current position and
            % velocities when one wall affects the robot
            %
            % Input:
            % obj - Instance of class roombaSim
            % tStep - Double, time since the previous state update
            % collPts - Vector of doubles [x y i f]
            %   x - x-coordinate of closest point to the center of the robot
            %   y - y-coordinate of closest point to the center of the robot
            %   i - Index of wall, used as obj.mapWalls(i,:)
            %   f - Corner flag, is 1 if intersection point is a corner
            %   To be used in this function, collPts must have exactly 1 row
            
            % Get important values
            r= obj.radius;
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v_int= obj.velInt*[cos(th) sin(th)];
            w_int= obj.omegaInt;
            muK= obj.frictionKin;
            i_wall= collPts(3);
            
            % Get wall data with x1 <= x2
            [x1,wall_idx]= min(obj.mapWalls(i_wall,[1 3]));
            if wall_idx == 1
                y1= obj.mapWalls(i_wall,2);
                x2= obj.mapWalls(i_wall,3);
                y2= obj.mapWalls(i_wall,4);
            else
                y1= obj.mapWalls(i_wall,4);
                x2= obj.mapWalls(i_wall,1);
                y2= obj.mapWalls(i_wall,2);
            end
            
            % Get tangential vector to the wall in the correct direction
            % That is, counter-clockwise around the robot
            if collPts(2) <= y   % Wall is beneath robot
                tV= [x2-x1 y2-y1]/sqrt((x2-x1)^2+(y2-y1)^2);
            else
                tV= [x1-x2 y1-y2]/sqrt((x1-x2)^2+(y1-y2)^2);
            end
            
            % Get normal vector from wall to robot
            nV= [x-collPts(1) y-collPts(2)]/...
                sqrt((x-collPts(1))^2+(y-collPts(2))^2);
            
            % Put intended velocity into tangential and normal directions
            v_t_int= dot(v_int,tV);
            v_n_int= dot(v_int,nV);
            
            % Compute true normal velocity
            v_n= (v_n_int > 0)*v_n_int; % Make it zero if intended normal
            % velocity is negative (towards wall)
            v_n_int= (v_n_int <= 0)*v_n_int;    % Make zero or negative for
            % friction computation
            
            %%%%%%%%%%%%% Check if robot is sliding or is in pure rolling%%%%%%%%%%
            
            % Compute angular and tangential velocity
            % Assume normal force ~ normal velocity
            if -w_int*r > v_t_int
                v_t= v_t_int+v_n_int*muK;
                w= w_int+sign(w_int)*v_n_int*muK/r;
            else
                v_t= v_t_int-v_n_int*muK;
                w= w_int-sign(w_int)*v_n_int*muK/r;
            end
            if sign(v_t_int) ~= sign(v_t)   % Motion opposite of intended
                v_t= 0;     % Friction should only resists motion
            end
            if sign(w_int) ~= sign(w)
                w= 0;       % Friction should only resist motion
            end
            
            % Compute cartesian components of velocity
            v_x= v_t*tV(1)+v_n*nV(1);
            v_y= v_t*tV(2)+v_n*nV(2);
            
            % Update position
            obj.posAbs= [x+v_x*tStep  y+v_y*tStep];
            obj.thAbs= obj.wrap2pi(th+w*tStep);
            obj.velAbs= [v_x v_y];
            obj.wAbs= w;
        end
        
        function drive2Wall(obj,tStep,collPts)
            % drive2Wall(obj,tStep,collPts)
            % Updates the new position based on the current position and
            % velocities when two walls affect the robot
            %
            % Input:
            % obj - Instance of class roombaSim
            % tStep - Double, time since the previous state update
            % collPts - Matrix of doubles, columns [x y i f]
            %   x - x-coordinate of closest point to the center of the robot
            %   y - y-coordinate of closest point to the center of the robot
            %   i - Index of wall, used as obj.mapWalls(i,:)
            %   f - Corner flag, is 1 if intersection point is a corner
            %   To be used in this function, collPts must have exactly 2 rows
            
            % Get important values
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v_int= obj.velInt*[cos(th) sin(th)];
            w_int= obj.omegaInt;
            muK= obj.frictionKin;
            i_wall= collPts(:,3);
            
            % Get wall data with x1 <= x2
            x1= zeros(length(i_wall),1);    % Preallocate for speed
            y1= zeros(length(i_wall),1);
            x2= zeros(length(i_wall),1);
            y2= zeros(length(i_wall),1);
            tV= zeros(length(i_wall),2);
            nV= zeros(length(i_wall),2);
            for j= 1:length(i_wall)
                % Check that wall is not vertical
                if obj.mapWalls(i_wall(j),1) ~= obj.mapWalls(i_wall(j),3)
                    [x1(j), wall_idx]= min(obj.mapWalls(i_wall(j),[1 3]));
                    if wall_idx == 1
                        y1(j)= obj.mapWalls(i_wall(j),2);
                        x2(j)= obj.mapWalls(i_wall(j),3);
                        y2(j)= obj.mapWalls(i_wall(j),4);
                    else
                        y1(j)= obj.mapWalls(i_wall(j),4);
                        x2(j)= obj.mapWalls(i_wall(j),1);
                        y2(j)= obj.mapWalls(i_wall(j),2);
                    end
                    
                    % Get tangential vector to wall in the correct direction
                    % That is, counter-clockwise around the robot
                    if collPts(j,2) < y     % Wall is beneath robot
                        tV(j,:)= [x2(j)-x1(j) y2(j)-y1(j)]/...
                            sqrt((x2(j)-x1(j))^2+(y2(j)-y1(j))^2);
                    else
                        tV(j,:)= [x1(j)-x2(j) y1(j)-y2(j)]/...
                            sqrt((x1(j)-x2(j))^2+(y1(j)-y2(j))^2);
                    end
                else    % Vertical wall
                    [y1(j), wall_idx]= min(obj.mapWalls(i_wall(j),[2 4]));
                    if wall_idx == 1
                        x1(j)= obj.mapWalls(i_wall(j),1);
                        x2(j)= obj.mapWalls(i_wall(j),3);
                        y2(j)= obj.mapWalls(i_wall(j),4);
                    else
                        x1(j)= obj.mapWalls(i_wall(j),3);
                        x2(j)= obj.mapWalls(i_wall(j),1);
                        y2(j)= obj.mapWalls(i_wall(j),2);
                    end
                    
                    % Get tangential vector to wall in the correct direction
                    % That is, counter-clockwise around the robot
                    if collPts(j,1) < x     % Wall is left of robot
                        tV(j,:)= [x1(j)-x2(j) y1(j)-y2(j)]/...
                            sqrt((x1(j)-x2(j))^2+(y1(j)-y2(j))^2);
                    else
                        tV(j,:)= [x2(j)-x1(j) y2(j)-y1(j)]/...
                            sqrt((x2(j)-x1(j))^2+(y2(j)-y1(j))^2);
                    end
                end
                
                % Get normal vector from wall to robot
                nV(j,:)= [x-collPts(j,1) y-collPts(j,2)]/...
                    sqrt((x-collPts(j,1))^2+(y-collPts(j,2))^2);
            end
            
            % Find normal intended velocity components
            v_n1_int= dot(v_int,nV(1,:));
            v_n2_int= dot(v_int,nV(2,:));
            
            % Find if the robot is driving into the corner
            done= false;    % Signal that other sim function was called
            if dot(tV(1,:),tV(2,:)) <= 0    % Walls at acute or right angle
                if (v_n1_int <= 0 && v_n2_int <= 0) || (v_n1_int <= 0 ...
                        && xor(dot(v_int,tV(1,:)) <= 0, ...
                        dot(tV(1,:),nV(1,:)+nV(2,:)) < 0)) || ...
                        (v_n2_int <= 0 && xor(dot(v_int,tV(2,:)) <= 0, ...
                        dot(tV(2,:),nV(1,:)+nV(2,:)) < 0))
                    v= [0 0];   % Stuck in the corner
                    w= w_int+sign(w_int)*muK*(v_n1_int+v_n2_int);
                    if sign(w) ~= sign(w_int)
                        w= 0;   % Friction should only resist motion
                    end
                elseif dot(v_int,nV(1,:)) <= 0
                    collPts= collPts(1,:);  % Drive towards wall 1
                    drive1Wall(obj,tStep,collPts)
                    done= true;
                elseif dot(v_int,nV(2,:)) <= 0
                    collPts= collPts(2,:);  % Drive towards wall 2
                    drive1Wall(obj,tStep,collPts)
                    done= true;
                else                        % Drive away from walls
                    v= v_int;
                    w= w_int;
                end
            else                        % Walls at obtuse
                if v_n1_int > 0 && v_n2_int > 0 % Drive away from walls
                    v= v_int;
                    w= w_int;
                elseif sign(dot(v_int,tV(1,:))) == sign(dot(v_int,tV(2,:)))
                    if v_n1_int < v_n2_int      % Drive towards wall 1
                        collPts= collPts(1,:);
                        drive1Wall(obj,tStep,collPts)
                    else                        % Drive towards wall 2
                        collPts= collPts(2,:);
                        drive1Wall(obj,tStep,collPts)
                    end
                    done= true;
                else                            % Stuck in the corner
                    v= [0 0];   % Stuck in the corner
                    w= w_int+sign(w_int)*muK*(v_n1_int+v_n2_int);
                    if abs(w) > abs(w_int)
                        w= 0;   % Friction should only resist motion
                    end
                end
            end
            
            if ~done    % drive1Wall hasn't been called to update everything
                % Update position
                % Don't use arc calculation to avoid errors
                obj.posAbs= [x+v(1)*tStep  y+v(2)*tStep];
                obj.thAbs= obj.wrap2pi(th+w*tStep);
                obj.velAbs= v;
                obj.wAbs= w;
            end
        end
        
        function driveCorner(obj,tStep,collPts)
            % driveCorner(obj,tStep,collPts)
            % Updates the new position based on the current position and
            % velocities when one corner affects the robot
            %
            % Input:
            % obj - Instance of class roombaSim
            % tStep - Double, time since the previous state update
            % collPts - Matrix of doubles, columns [x y i f]
            %   x - x-coordinate of closest point to the center of the robot
            %   y - y-coordinate of closest point to the center of the robot
            %   i - Index of wall, used as obj.mapWalls(i,:)
            %   f - Corner flag, is 1 if intersection point is a corner
            %   To be used in this function, collPts must have exactly 1 row
            
            % Get important values
            r= obj.radius;
            x= obj.posAbs(1);
            y= obj.posAbs(2);
            th= obj.thAbs;
            v_int= obj.velInt*[cos(th) sin(th)];
            w_int= obj.omegaInt;
            muK= obj.frictionKin;
            
            % Get normal vector from corner to robot
            nV= [x-collPts(1) y-collPts(2)]/...
                sqrt((x-collPts(1))^2+(y-collPts(2))^2);
            
            if dot(v_int,nV) >= 0   % Moving away from the corner
                driveNormal(obj,tStep)
            else
                % Assume rolling only motion around the corner
                tV= cross([nV 0],[0 0 1]);  % Vector tangential to path
                tV= tV(1:2);                % clockwise around corner
                a= -dot(v_int,tV)*tStep/r;  % Angle to travel
                dnV= (cos(a)-1)*r*nV;   % Movement in normal direction
                dtV= -sin(a)*r*tV;      % Movement in tangential direction
                w= w_int+sign(w_int)*dot(v_int,nV)*muK; % Angular velocity
                if sign(w) ~= sign(w_int)
                    w= 0;           % Friction should only resist motion
                end
                
                % Update position
                obj.posAbs= [x+dnV(1)+dtV(1)  y+dnV(2)+dtV(2)];
                obj.thAbs= obj.wrap2pi(th-a+w*tStep);
                obj.velAbs= (dnV+dtV)/tStep;
                obj.wAbs= w;
            end
        end
        
        % Translator Functions
        
        
        function state= VirtualWallSensorCreate(obj)
            % state = VirtualWallSensorCreate(obj)
            % Reads the state of the virtual wall sensor
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % state - Boolean double, 1 if the robot detects a virtual wall
            
            % Check for valid input
            if ~isa(obj,'roombaSim')
                error('Simulator:invalidInput',...
                    ['Input to VirtualWallSensorCreate must '...
                    'have class roombaSim.  Input argument should be '...
                    'the input argument to the control program'])
            else
                try
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Find out if robot is within range of any virtual wall
                    state= genVWall(obj);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf('%.0f= VirtualWallSensorCreate',...
                        state);
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function VirtualWallSensorCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function distance= ReadSonar(obj,varargin)
            % distance = ReadSonar(obj)
            % Reads the distance returned by the front sonar sensor
            % Functionality has changed from the WOOSH software
            %
            % distance = ReadSonar(obj,sonarNum)
            % Reads the distance returned by the specified sonar sensor
            % Identical to ReadSonarMultiple
            %
            %
            % Input:
            % obj - Instance of class roombaSim
            % sonarNum - Number corresponding to sonar to be read
            %   The simulator assumes this sonar setup on the WOOSH board:
            %       1 - Right
            %       2 - Front
            %       3 - Left
            %       4 - Back
            %
            % Output:
            % distance - Double, distance to nearest obstacle in front of
            %   the robot, or the range of the sonar if no obstacle in range
            
            % Check for valid input
            if ~isa(obj,'roombaSim')
                error('Simulator:invalidInput',...
                    ['The first input argument to ReadSonar must have '...
                    'class roombaSim.  Input argument should be the '...
                    'input argument to the control program'])
            elseif nargin > 2
                error('Simulator:invalidInput',...
                    ['There can be no more than 2 input arguments to '...
                    'ReadSonar.'])
            elseif ~isnumeric(varargin{1}) || ~(length(varargin{1}) == 1)
                error('Simulator:invalidInput',...
                    ['The second input argument to ReadSonar must be a '...
                    'single number.'])
            elseif varargin{1} < 1 || varargin{1} > 4
                error('Simulator:invalidInput',...
                    ['The second input argument to ReadSonar must be a '...
                    'number between 1 and 4.'])
            else
                try
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    distSonar= genSonar(obj);
                    
                    % Extract value for output
                    if isempty(varargin)
                        distance= distSonar(2);
                    else
                        sonarNum= varargin{1};
                        % Extract value for output
                        % Note difference between order of output of
                        % genSonar and order of indices for sonarNum
                        if sonarNum == 1
                            distIdx= 4;
                        else
                            distIdx= sonarNum-1;
                        end
                        distance= distSonar(distIdx);
                    end
                    
                    % Check that distance is within limits
                    % Sonar functions output empty vector otherwise
                    if distance <= obj.rangeMinSonar || ...
                            distance >= obj.rangeSonar
                        distance= [];
                    end
                    
                    % Add the translator function call to output data
                    if isempty(distance)
                        fcn_called= sprintf('[]= ReadSonar');
                    else
                        fcn_called= sprintf('%.3f= ReadSonar',distance);
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function ReadSonar.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function distance= ReadSonarMultiple(obj,sonarNum)
            % distance = ReadSonarMultiple(obj,sonarNum)
            % Reads the distance returned by the specified sonar sensor
            %
            % Input:
            % obj - Instance of class roombaSim
            % sonarNum - Number corresponding to sonar to be read
            %   The simulator assumes this sonar setup on the WOOSH board:
            %       1 - Right
            %       2 - Front
            %       3 - Left
            %       4 - Back
            %
            % Output:
            % distance - Double, distance to nearest obstacle in the path of
            %   the specified sonar, or an empty vector no obstacle
            %   in range
            
            % Check for valid input
            if ~isa(obj,'roombaSim')
                error('Simulator:invalidInput',...
                    ['Input to ReadSonarMultiple must have class '...
                    'roombaSim.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    distSonar= genSonar(obj);
                    
                    % Extract value for output
                    % Note difference between order of output of genSonar
                    % and order of indices for sonarNum
                    if sonarNum == 1
                        distIdx= 4;
                    else
                        distIdx= sonarNum-1;
                    end
                    distance= distSonar(distIdx);
                    
                    % Check that distance is within limits
                    % Sonar functions output empty vector otherwise
                    if distance < obj.rangeMinSonar || ...
                            distance > obj.rangeSonar
                        distance= [];
                    end
                    
                    % Add the translator function call to output data
                    if isempty(distance)
                        fcn_called= sprintf('[]= ReadSonarMultiple(%.0f)',...
                            sonarNum);
                    else
                        fcn_called= sprintf(['%.3f= ReadSonarMultiple'...
                            '(%.0f)'],distance,sonarNum);
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function ReadSonarMultiple.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function distScan= LidarSensorCreate(obj)
            % distScan = LidarSensorCreate(obj)
            % Reads the range of distances from the LIDAR sensor
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % distScan - Array of doubles, of length numPtsLidar with the
            %   first value corresponding to the right-most reading and the
            %   last corresponding to the left-most reading on the LIDAR, the
            %   readings will be the distance to the nearest obstacle, or the
            %   range of the LIDAR if no obstacle is in range
            
            % Check for valid input
            if ~isa(obj,'roombaSim')
                error('Simulator:invalidInput',...
                    ['Input to LidarSensorCreate must have class '...
                    'roombaSim.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    distScan= genLidar(obj);
                    
                    % Add the translator function call to output data
                    fcn_called= sprintf(['[%.3f %.3f %.3f %.3f %.3f]= '...
                        'LidarSensorCreate'],distScan(1),...
                        distScan(ceil(obj.numPtsLidar/4)),...
                        distScan(ceil(obj.numPtsLidar/2)),...
                        distScan(ceil(3*obj.numPtsLidar/4)),...
                        distScan(end));
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function LidarSensorCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function [X, Y, Z, ROT, Ntag] = ReadBeacon(obj)
            % [X Y Z ROT Ntag] = ReadBeacon(obj)
            % Reads the ARtag detection camera and reports 3-D cartesian
            % position in the camera reference frame, as well as the rotation
            % about the tag center
            %
            % Camera coordinate frame is defined as (updated May 2014):
            % x - axis points to right
            % y - axis points down
            % z - axis points out of camera (depth)
            %
            % Input:
            % obj - Instance of the class roombaSim
            %
            % Output:
            % X   - column vector of x coordinates in camera coordinates
            % y   - column vector of y coordinates in camera coordinates
            % z   - column vector of z coordinates in camera coordinates
            % rot - column vector of rotation of tag around its center point
            % Ntag- column vector of AR tag numbers detected in camera frame
            
            % Check for valid input
            if ~isa(obj,'roombaSim')
                error('Simulator:invalidInput',...
                    ['Input to ReadBeacon must have class '...
                    'roombaSim.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    [angle,dist,color,Ntag]= genCamera(obj);
                    
                    X = -dist.*sin(angle);      % Minus, because x-axis right
                    Y = zeros(numel(angle),1);  % Assume tags are in same plane as the camera
                    Z = +dist.*cos(angle);
                    ROT = Y;                    % Assume tags are oriented upright
                    
                    % Add the translator function call to output data
                    if isempty(angle)   % No beacons detected
                        fcn_called= '[]= ReadBeacon';
                    else                % Only output first beacon found
                        fcn_called= sprintf(['[%.3f %.3f %.3f %.3f %.3f]'...
                            ' = ReadBeacon'],X(1),Y(1),Z(1),ROT(1),Ntag(1));
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function ReadBeacon.')
                    end
                    rethrow(me)
                end
            end
        end
        
        function [angle, dist, color]= CameraSensorCreate(obj)
            % [angle dist color] = CameraSensorCreate(obj)
            % Reads the output from the blob detection on the camera,
            %   detects only beacons
            %
            % Input:
            % obj - Instance of class roombaSim
            %
            % Output:
            % angle - Vector of doubles, each the angle relative to robot at
            %   which beacon is detected
            % dist - Vector of doubles, each the distance of beacon from camera
            % color - Matrix of doubles of width three (color vector),
            %   each row the color of beacon detected
            
            % Check for valid input
            if ~isa(obj,'roombaSim')
                error('Simulator:invalidInput',...
                    ['Input to CameraSensorCreate must have class '...
                    'roombaSim.  Input argument should be the input '...
                    'argument to the control program'])
            else
                try
                    % Pause for communication delay
                    pause(obj.comDelay)
                    
                    % Generate reading for sensor
                    [angle, dist, color]= genCamera(obj);
                    
                    % Add the translator function call to output data
                    if isempty(angle)   % No beacons detected
                        fcn_called= '[]= CameraSensorCreate';
                    else                % Only output first beacon found
                        fcn_called= sprintf(['[%.3f %.3f [%.3f %.3f '...
                            '%.3f]]= CameraSensorCreate'],angle(1),...
                            dist(1),color(1,1),color(1,2),color(1,3));
                    end
                    addFcnToOutput(obj,fcn_called)
                    
                catch me
                    if ~strcmp(me.identifier,'SIMULATOR:AutonomousDisabled')
                        disp('Simulator:unknownError')
                        disp('Error in function CameraSensorCreate.')
                    end
                    rethrow(me)
                end
            end
        end
        
        
        function ignoreCommand(obj,cmd,descrip,nbytes)
            if nargin<3, descrip='Unknown'; end
            if nargin<4, nbytes=0; end
            fprintf('Unimplemented: %s (%u)\n',descrip,cmd);
            if nbytes>0;
                data = getData(nbytes);
                disp(data);
            end
        end % ignoreCommand
        function doCommand(obj,cmd)
            % TODO - may need to enforce some physical limits on speeds
            % this is where low level Roomba commands are translated
            % The Create Toolbox was searched for sendRoombaCommand to get
            % this list of what needed to be simulated
            
            fprintf('Command: %u\n',cmd);
            switch cmd
                case 7
                    %fclose(obj.port);
                    %delete(obj.port);
                    %delete(obj);
                    %quit;
                    obj.initPosition();
                case 128; beep; obj.ignoreCommand(cmd,'Start');
                case 129; obj.ignoreCommand(cmd,'Baud',1);
                case 131; obj.ignoreCommand(cmd,'Safe Mode');
                case 132; obj.ignoreCommand(cmd,'Full Mode');
                case 133; obj.ignoreCommand(cmd,'Power');
                case 134; obj.ignoreCommand(cmd,'Spot');
                case 143; obj.ignoreCommand(cmd,'Dock');
                case 135; obj.ignoreCommand(cmd,'Clean');
                case 136; obj.ignoreCommand(cmd,'Max');
                case 137 % Drive (velocity/radius)
                    D = obj.getData(2,'int16');
                    obj.velInt= double(D(1))/1000;
                    if D(2)==32767 || D(2)==32768 % drive straight
                        obj.velInt= double(D(1))/1000;
                        obj.omegaInt= 0;
                        obj.wheelVel = [obj.velInt obj.velInt];
                    elseif abs(D(2))==1 % turn in place
                        obj.velInt= 0;
                        obj.omegaInt = double(D(2)*D(1))/1000/(pi*obj.wheelBase);
                        obj.wheelVel = [obj.velInt*D(2) -obj.velInt*D(2)];
                    else % a valid radius value
                        obj.velInt= double(D(1))/1000;
                        obj.omegaInt= double(D(2))/1000/obj.wheelBase;
                    end
                case 138; obj.ignoreCommand(cmd,'Motors',1);
                case 139 % leds
                    obj.LEDs.otherByte = obj.getData();
                    obj.LEDs.centerColor = obj.getData();
                    obj.LEDs.centerIntensity = obj.getData();
                case 140 % store a song
                    isong = obj.getData() + 1;
                    if isong<1 || isong>4
                        warning('Invalid song number: %u', isong);
                        return
                    end
                    cnt = obj.getData();
                    if cnt>16
                        warning('Song truncated, original cnt=%u',cnt);
                        cnt=16;
                    end
                    data = obj.getData(cnt*2);
                    obj.songs.data{isong} = data;
                    
                case 141 % play a song
                    isong = obj.getData() + 1;
                    if isong<1 || isong>4
                        warning('Invalid song number: %u',isong);
                        return
                    end
                    obj.songs.playing=1;
                    obj.songs.number=isong;
                    songdata = obj.songs.data{isong};
                    dur = obj.playSongWave(songdata);
                    tt = timer
                    tt.TimerFcn = {@obj.finishWave, obj};
                    tt.StartDelay = round(dur,3);
                    start(tt);
                case 142 % get single sensor data
                    pkt = obj.getData();
                    fprintf('Getting packet #%u\n',pkt);
                    obj.setReturnData(pkt);
                case 144; obj.ignoreCommand(cmd,'PWM Moters',3);
                case 145 % set drive wheels direct
                    D = obj.getData(2,'int16');
                    rightWheel = double(D(1))/1000;
                    leftWheel = double(D(2))/1000;
                    obj.velInt= (rightWheel+leftWheel)/2;  % Average
                    obj.omegaInt= (rightWheel-leftWheel)/obj.wheelBase;
                    obj.wheelVel = [ rightWheel leftWheel];
                case 146 % set drive wheels direct PWM
                    % Assume that 500mm/s is the max speed
                    % and PWM max value is 255
                    D = obj.getData(2,'int16');
                    D(D>255)=255; D(D<-255)=-255; % limit values to 255
                    rightWheel = double(D(1))*500/255/1000;
                    leftWheel = double(D(2))*500/255/1000;
                    obj.velInt= (rightWheel+leftWheel)/2;  % Average
                    obj.omegaInt= (rightWheel-leftWheel)/obj.wheelBase;
                    obj.wheelVel = [ rightWheel leftWheel];
                case 148; obj.ignoreCommand(cmd,'Stream',obj.getData(1));
                case 149 % send a list of commands
                    cnt = obj.getData();
                    data = obj.getData(cnt);
                    for ii=1:cnt
                        obj.setReturnData(data(ii));
                    end
                case 150; obj.ignoreCommand(cmd,'Pause/Resume Stream',1);
                case 162 % leds
                    obj.LEDs.dayByte = obj.getData();
                    obj.LEDs.scheduleByte = obj.getData();
                case 163;
                    obj.LEDs.segCodes = obj.getData(4);
                    obj.updateDisplay();
                case 164 % set LED Digits
                    txt = char(obj.getData(4));
                    obj.LEDs.digits = txt;
                    fprintf('Setting LED Digits: %s\n',txt);
                    obj.LEDs.segCodes = obj.convertAsciiToSegCodes(txt);
                    obj.updateDisplay();
                case 165; obj.ignoreCommand(cmd,'Buttons',1);
                case 167; obj.ignoreCommand(cmd,'Schedule',15);
                case 168; obj.ignoreCommand(cmd,'Set Day/Time',3);
                case 173; obj.ignoreCommand(cmd,'Stop Mode');
                otherwise
                    obj.ignoreCommand(cmd);
            end
        end % doCommand
        %  _
        % |_|
        % |_|
        %
        function updateDisplay(obj)
            D = char(' '.*ones(3,16));
            for c=1:4
                if bitget(obj.LEDs.segCodes(c),1), D(1,c*4-2)='_'; end
                if bitget(obj.LEDs.segCodes(c),2), D(2,c*4-1)='|'; end
                if bitget(obj.LEDs.segCodes(c),3), D(3,c*4-1)='|'; end
                if bitget(obj.LEDs.segCodes(c),4), D(3,c*4-2)='_'; end
                if bitget(obj.LEDs.segCodes(c),5), D(3,c*4-3)='|'; end
                if bitget(obj.LEDs.segCodes(c),6), D(2,c*4-3)='|'; end
                if bitget(obj.LEDs.segCodes(c),7), D(2,c*4-2)='_'; end
            end
            fprintf('\n');
            disp(D);
        end % updateDisplay
        
        function R = convertAsciiToSegCodes(obj,txt)
            txt(txt==0)=256;
            R = obj.LEDs.asciiSegCodes(txt);
        end % convertAsciiToSegCodes
        
        function generateAsciiSegCodes(obj)
            c = zeros(256,1);
            c('!') = bin2dec('1101011'); %33
            c('"') = bin2dec('0100010');
            c('#') = bin2dec('0110110');
            c('$') = bin2dec('1101101');
            c('%') = bin2dec('0100100');
            c('&') = bin2dec('1111000');
            c('''') = bin2dec('0000010');
            c('(') = bin2dec('0111001'); %40
            c(')') = bin2dec('0001111');
            c('*') = bin2dec('0000000');
            c('+') = bin2dec('0000000');
            c(',') = bin2dec('0010000');
            c('-') = bin2dec('1000000');
            c('.') = bin2dec('1011100');
            c('/') = bin2dec('1010010');
            c('0') = bin2dec('0111111');
            c('1') = bin2dec('0000110');
            c('2') = bin2dec('1011011'); %50
            c('3') = bin2dec('1001111');
            c('4') = bin2dec('1100110');
            c('5') = bin2dec('1101101');
            c('6') = bin2dec('1111101');
            c('7') = bin2dec('0000111');
            c('8') = bin2dec('1111111');
            c('9') = bin2dec('1101111');
            c('\:') = bin2dec('0001001'); % colon is special character
            c(';') = bin2dec('1010001');
            c('<') = bin2dec('1100001'); %60
            c('=') = bin2dec('1001000');
            c('>') = bin2dec('1000011');
            c('?') = bin2dec('1010011');
            c('@') = bin2dec('0000000');
            c('A') = bin2dec('1110111'); %65
            c('B') = bin2dec('1111100');
            c('C') = bin2dec('0111001');
            c('D') = bin2dec('1011110');
            c('E') = bin2dec('1111001');
            c('F') = bin2dec('1110001');
            c('G') = bin2dec('1101111');
            c('H') = bin2dec('1110110');
            c('I') = bin2dec('0000110');
            c('J') = bin2dec('0011110');
            c('K') = bin2dec('1110110');
            c('L') = bin2dec('0111000');
            c('M') = bin2dec('1010100');
            c('N') = bin2dec('0110111');
            c('O') = bin2dec('0111111');
            c('P') = bin2dec('1110011');
            c('Q') = bin2dec('1100111');
            c('R') = bin2dec('1010000');
            c('S') = bin2dec('1101101');
            c('T') = bin2dec('0000111');
            c('U') = bin2dec('0111110');
            c('V') = bin2dec('0111110');
            c('W') = bin2dec('1110110');
            c('X') = bin2dec('0111110');
            c('Y') = bin2dec('1101110');
            c('Z') = bin2dec('1011011'); %90
            c('[') = bin2dec('0111001');
            
            c('\') = bin2dec('1100100');
            c(']') = bin2dec('0001111');
            c('^') = bin2dec('0100011');
            c('_') = bin2dec('0001000');
            c('`') = bin2dec('0000011');
            
            % make upper and lowercase the same
            c('a':'z') = c('A':'Z');
            
            c('{') = bin2dec('1000110');
            c('|') = bin2dec('0000110');
            c('}') = bin2dec('1110000');
            c('~') = bin2dec('0000001');
            
            % save chart with object
            obj.LEDs.asciiSegCodes = c;
        end % generateAsciiSegCodes
        
        function flushBuffer(obj,keep)
            % Flush Buffer
            if nargin<2, keep=0; end
            N = obj.port.BytesAvailable();
            while N>keep
                fprintf('Flushing %u bytes\n',N-keep);
                fread(obj.port,N-keep);
                N = obj.port.BytesAvailable();
            end
        end
        
        function R = packBytes(~,data)
            % Convert a variable (usually an int16 or uint16) into separate bytes
            % suitable for sending as Roomba data.
            % This is a low-level routine and typically not used by the end-user
            
            % Input:
            %   data - data to be packed
            % Output:
            %   R - a uint8 list of bytes
            % Example:
            %   obj.packBytes(uint16(1))  returns [ 0  1 ]
            %   obj.packBytes(int16(-2))  returns [ 255 254 ]
            
            % tecnhically we need this, but all supported MATLAB platforms are little Endian
            % so we skip the check and always swap bytes
            % [~,~,endian] = computer;
            %if endian=='L'
            data = swapbytes(data);
            % end
            R = typecast(data, 'uint8');
            
        end % end of obj.packBytes function
        
        function setReturnData(obj,pkt)
            % This function maps packet requests to simulated data
            
            % handle packet 'packages'
            switch(pkt)
                case 0
                    pktlist = 7:26;
                case 1
                    pktlist = 7:16;
                case 2
                    pktlist = 17:20;
                case 3
                    pktlist = 21:26;
                case 4
                    pktlist = 27:34;
                case 5
                    pktlist = 35:42;
                case 6
                    pktlist = 7:42;
                case 100
                    pktlist = 7:58;
                case 101
                    pktlist = 43:58;
                case 106
                    pktlist = 46:51;
                case 107
                    pktlist = 54:58;
                otherwise
                    pktlist = pkt;
            end
            
            % loop thru pktlist
            for pkt=pktlist
                disp(pkt);
                switch pkt
                    case 7
                        Bump= genBump(obj);
                        data = Bump(1) + Bump(3)*2 + Bump(2)*3;
                        % no wheel drops in simulator
                    case 8
                        data= 0; % use right light bumper instead
                    case {9,10,11,12}  % left, front left, front right, right
                        Cliff= genCliff(obj);
                        data = Cliff(13-pkt) <= obj.cliffVoid;
                    case 13 % virtual wall
                        data= genVWall(obj);
                    case 14 % wheel overcurrents
                        data= 0; % not used
                    case 15 % dirt detect
                        data= 0; % not used
                    case 16 % unused byte
                        data= 0;
                    case 17 % IR TODO?
                        data= 0;
                    case 18 % Buttons
                        data= 0;
                        if ~isempty(obj.handles)
                            h=obj.handles;
                            data = bitset(data,8,get(h.toggle_clock,'Value'));
                            data = bitset(data,7,get(h.toggle_schedule,'Value'));
                            data = bitset(data,6,get(h.toggle_day,'Value'));
                            data = bitset(data,5,get(h.toggle_hour,'Value'));
                            data = bitset(data,4,get(h.toggle_minute,'Value'));
                            data = bitset(data,3,get(h.toggle_dock,'Value'));
                            data = bitset(data,2,get(h.toggle_spot,'Value'));
                            data = bitset(data,1,get(h.toggle_clean,'Value'));
                        end
                    case 19 % distance in mm
                        data = obj.packBytes(int16( genOdomDist(obj)*1000));
                    case 20 % angle in degrees
                        data = obj.packBytes(int16( genOdomAng(obj)*180/pi));
                    case 21 % charging state
                        data = 0;
                    case 22 % voltage mv
                        data = obj.packBytes(uint16(17200));
                    case 23 % current mA
                        data = obj.packBytes(int16(-3000));
                    case 24 % temperature
                        data = 25;
                    case 25 % battery charge
                        data = obj.packBytes(uint16(2000));
                    case 26 % battery capacity
                        data = obj.packBytes(uint16(3000));
                    case 27 % wall signal
                        data = 0; % use right light bumper
                    case {28,29,30,31} % left, front left, front right, right signals
                        Cliff= genCliff(obj);
                        data = obj.packBytes(uint16(Cliff(32-pkt)));
                    case 32 % unused 1 byte
                        data = 0;
                    case 33 % unused 2 byte
                        data = [0 0];
                    case 34 % Charging source
                        data = 0;
                    case 35 % OI mode
                        data = 3;
                    case 36 % song number
                        data = obj.songs.current;
                    case 37 % is song playing
                        data = obj.songs.playing;
                    case 38 % number of stream packets
                        data = 0; % not used
                    case {39,40,41,42} % requested velocities TODO
                        data = obj.packBytes(int16(0));
                    case 43 % left encoder count
                        [left,~] = genEncoderCount(obj);
                        data = obj.packBytes(int16(left));
                    case 44 % right encoder count
                        [~,right] = genEncoderCount(obj);
                        data = obj.packBytes(int16(right));
                    case 45 % light bumper
                        R = genLightBumper(obj);
                        data = uint8(R(7));
                    case {46,47,48,49,50,51} % light bumper signals
                        R = genLightBumper(obj);
                        data = obj.packBytes(uint16(R(pkt-45)));
                    case {52,53} % IR TODO?
                        data = 0;
                    case {54,55,56,57} % motor currents
                        data = obj.packBytes(int16(-3000));
                    case 58  % statis (TODO)
                        data = 0;
                    otherwise
                        warning('Unsupported sensor packet %u',pkt)
                end
                % data can be any numeric type here, but it gets converted
                % to uint8
                obj.sendData(uint8(data));
            end
        end %setReturnData
        
        function R = sendData(obj,data)
            fprintf('Sending %u bytes of data\n',length(data));
            fwrite(obj.port,data);
        end % sendData
        
        function R = getData(obj,numitems,datatype)
            
            if nargin<2, numitems=1; end
            if nargin<3, datatype = 'uint8'; end
            R = [];
            switch datatype
                case {'uint8','int8','uint16','int16'}
                    R = double(fread(obj.port, numitems, datatype));
                otherwise
                    warning('Unsupported datatype: %s',datatype);
                    return;
            end
        end % getData
        
        function  [xs,ys] = mapLinesImgConvert(obj,x,y)
            xs = round(obj.mapLinesImgSc*x)+obj.mapLinesImgOrig(1);
            ys = round(obj.mapLinesImgSc*y)+obj.mapLinesImgOrig(2);
        end
        
        function createLineImg(obj)
            % create an image for line data - the image is used to optimize
            % the lines are 'drawn' on the image so that the cliff sensor
            % check can check pixels based on x,y position rather
            % than calculate many distances each time step
            
            % assume that there are walls in the map
            if size(obj.mapWalls,1)<2
                % not enough walls, set arbitrary world boundary
                obj.worldLimits = [-2 2 -2,2];
            else
                % set world boundary based on walls
                x1=min([obj.mapWalls(:,1);obj.mapWalls(:,3)]);
                y1=min([obj.mapWalls(:,2);obj.mapWalls(:,4)]);
                x2=max([obj.mapWalls(:,1);obj.mapWalls(:,3)]);
                y2=max([obj.mapWalls(:,2);obj.mapWalls(:,4)]);
                % make it square and a bit bigger
                range= [x2-x1, y2-y1];
                center= [x1,y1] + range./2;
                side = max(range*1.1);
                obj.worldLimits = [center(1)-side/2, center(1)+side/2, center(2)-side/2, center(2)+side/2];
            end
            axis(obj.worldLimits);
            w = obj.worldLimits(2)-obj.worldLimits(1);
            h = obj.worldLimits(4)-obj.worldLimits(3);
            % w and h are the width of the 'world' in meters
            % make the image size 1000 pixels / meter if w and h are less
            % than 4. If w and h are large, limit image size to a max of 4000 pixels
            sc = min([4/h,4/w,1])*1000; % make the scale
            iw = round(w*sc);
            ih = round(h*sc);
            xo = 1+round(-obj.worldLimits(1)*sc);
            yo = 1+round(-obj.worldLimits(3)*sc);
            obj.mapLinesImg = zeros(ih,iw,'uint8');
            obj.mapLinesImgSc = sc;
            obj.mapLinesImgOrig = [xo,yo];
            numlines = size(obj.mapLines,2);
            lt=5; % line thickness (mm)
            for i=1:numlines
                [x1,y1] = obj.mapLinesImgConvert(obj.mapLines(i,1),obj.mapLines(i,2));
                [x2,y2] = obj.mapLinesImgConvert(obj.mapLines(i,3),obj.mapLines(i,4));
                dx = abs(x2-x1);
                dy = abs(y2-y1);
                if dx==0 && dy==0, continue; end;
                if dx>dy % lines closer to horizontal
                    if x2<x1
                        t=x2; x2=x1; x1=t;
                        t=y2; y2=y1; y1=t;
                    end
                    m=(y2-y1)/(x2-x1);
                    b=y1-m*x1;
                    for xx=x1:x2
                        yy=round(m*xx+b);
                        xx2=min(xx+lt,iw);
                        yy2=min(yy+lt,ih);
                        obj.mapLinesImg(yy:yy2,xx:xx2)=1;
                    end
                else % lines closer to vertical
                    if y2<y1
                        t=x2; x2=x1; x1=t;
                        t=y2; y2=y1; y1=t;
                    end
                    m=(x2-x1)/(y2-y1);
                    b=x1-m*y1;
                    for yy=y1:y2
                        xx=round(m*yy+b);
                        xx2=min(xx+lt,iw);
                        yy2=min(yy+lt,ih);
                        obj.mapLinesImg(yy:yy2,xx:xx2)=1;
                    end
                    
                end
            end
        end
        
        
    end % public methods
end % classdef roombaSim
