classdef roomba < handle
    % this object is a subclass of the handle class so that all
    % modifications to the object are automatically saved with the object
    properties
        quiet = 0; % flag for controlling connect and close songs - 0 or 1
        timedelay = 0.015; % time delay for making sure communications are sent/received
        bCreate2 = 1; % flag set to true if the bot is a Create2 - used to determine sensor availability
        server = struct(); % structure used to store network information
        wheelDiameter = 0.072; % drive wheel diameter (m)
        wheelBase = 0.258; % distance between drive wheels (m)
        wheelEncoderCount = 508.8; % encoder counts per revolution
        botID = 0; % numeric ID of the bot
        botName = ''; % text name of the bot
        botList; % list of all bot names
        port = []; % MATLAB object produced by fopen (serial, tcpip, or bluetooth)
        comID = '';
        imageURL; % web address for loading an image
        cameraURL; % web address for showing the camera video
        song = struct(); % structure to store song information
        defaultVelocity = 0.2; % velocity used if omitted (m/s)
        defaultDistance = 0.2; % distance used if omitted (m)
        defaultAngle = 90; % angle used if omitted (degrees)
        maxMoveTime = 15; % maximum time for turnAngle and moveDistance (seconds)
        bMoving = 0;
        bTurning = 0;
        bUseEncoders = 1; % use encoders to calculate distances and angles
        
        LEDs; % structure for storing current LED status
        tunes; % structure with built-in songs
        dataStream = struct('packetIDs',[1 4],'packetSizes',[10 14]);
        timeZero = now;
        userData;
    end % properties
    methods (Access=public)
        function this = roomba(varargin)
            % Create an object used to reference the iRobot Create
            % all inputs passed to the connect method - see that for details
            
            % the output (this in this case) is magically defined by virtue
            % of this being a class constructor
            this.LEDs.dayByte=0;
            this.LEDs.schedByte=0;
            this.LEDs.otherByte=0;
            this.LEDs.centerColor=0;
            this.LEDs.centerIntensity=0;
            this.LEDs.digits='    ';
            this.LEDs.segCodes=[0 0 0 0];
            this.song.index=0;
            this.song.isPlaying=0;
            this.tunes.connect = 'T300,O5,KF#,KC#,A,A,A,A,B,B,A/2,F/2,D'; % start of rocky top
            this.tunes.close = 'T300,O5,KF#,KC#,D/2,D/2,D,E,E,D*4'; % end of rocky top
            this.tunes.dock = 'T300,O5,KBb,C^*3,B,A*1.5,G,A*4';
            this.tunes.charge = 'O5,T500,C,E,G,C^*2,G,C^*4';
            this.tunes.shave = 'O4,T300,C^,G/2,G/2,A,G,x,B,C^';
            this.tunes.work = 'o5,t350,c,f*3,e,d*3,f,g*3/2,a/2,g,f,e*3';
            this.tunes.beep = 'O5,F/2';
            this.botList{1} = 'c3po';
            this.botList{2} = 'irongiant';
            this.botList{3} = 'eve';
            this.botList{4} = 'wall-e';
            this.botList{5} = 'johnny 5';
            this.botList{6} = 'rosie';
            this.botList{8} = 'jinx';
            this.botList{9} = 'r2d2';
            this.botList{11} = 't-1000';
            this.botList{12} = 'optimusprime';
            this.botList{13} = 'dotmatrix';
            this.botList{14} = 'tx';
            this.botList{17} = 'hal9000';
            this.botList{18} = 'marvin';
            
            if nargin, this.connect(varargin{:}); end
        end
        function beep(this)
            % Play a short song
            % The song is defined in the beepTone property of the object.
            try
                this.songPlay(this.tunes.beep);
            catch err
                this.showError(err);
            end
        end % beep
        function close(this)
            % Disconnect from the robot
            
            this.setDriveVelocity(0,0);
            if ~this.quiet
                this.songPlay(this.tunes.close);
            end
            this.sendCommand(173); % stop
            this.sendCommand(7); % reset
            disp('Roomba Close');
            fclose(this.port);
            delete(this.port);
            delete(this)
            
        end % close
        function stop(this)
            % Set all velocities to 0, i.e. stop moving
            this.setDriveVelocity(0);
        end % stop
        function moveDistance(this, distance, speed)
            % Moves the distance entered in meters
            % This function does not return until the move is complete
            % Negative distances move backwards.
            % speed should be between 0.025 and 0.5 m/s
            
            % fprintf('moveDist: Moving %.3f m at %.3f m/s\n',distance,speed);
            if nargin<2, distance=this.defaultDistance; end
            if nargin<3, speed=this.defaultVelocity; end
            try
                this.flushBuffer;
                
                if (abs(speed) < .025) %Speed too low
                    disp('WARNING: Speed is too low. Setting speed to minimum, .025 m/s');
                    speed = .025;
                end
                
                if (distance < 0) %Definition of SetFwdVelRAdius Roomba, speed has to be negative to go backwards. Takes care of this case. User shouldn't worry about negative speeds
                    speed = -speed;
                end
                
                if (distance ~=0)
                    this.setDriveVelocity(speed);
                    this.waitForDistance(distance);
                    this.setDriveVelocity(0);
                end
            catch err
                this.showError(err);
            end
        end % travelDist
        function turnAngle(this, angle, speed)
            % Turns in place angle degrees.
            % This function does not return until the turn is complete
            % Positive angle turns counterclockwise.
            % speed should be between 0 and 0.2 m/s
            
            if nargin<2, angle=this.defaultAngle; end;
            if nargin<3, speed=this.defaultVelocity; end;
            
            try
                
                if (speed < 0) %Speed shouldn't be negative
                    disp('WARNING: Speed is negative. Should be positive. Taking the absolute value');
                    speed = abs(speed);
                end
                
                if (abs(speed) < .025) %Speed is too low
                    disp('WARNING: Speed is too low. Setting speed to minimum, .025 m/s');
                    speed = .025;
                end
                
                turnDir = sign(angle)*eps;
                %turnAngle = mod(abs(angle),360)*sign(angle); %sets range to +/- 360 degrees to avoid excess turning
                turnAngle = angle;
                
                if turnAngle ~= 0
                    this.setDriveVelocityRadius(speed, turnDir);
                    this.waitForAngle(turnAngle);
                    this.setDriveVelocityRadius(0, 0);
                    % disp('Done turnAngle')
                end
            catch err
                this.showError(err);
            end
            
        end % turnAngle
        function testSensors(this)
            hf = figure;
            set(hf,'units','normalized',...
                'menubar','none',...
                'position',[.7 .1 .29 .8],...
                'name','iRobot Sensor Readings', ...
                'numbertitle','off',...
                'closerequestfcn',@shutdown ...
                );
            axis off
            t=timer;
            set(t,'Period',0.25,...
                'TimerFcn', {@timerfcn,this,hf},...
                'ExecutionMode','FixedSpacing');
            UD.h=[];
            UD.rob=this;
            UD.timer=t;
            set(hf,'UserData',UD)
            start(t);
            
            
            function shutdown(obj,~)
                try
                    UD = get(obj,'userdata');
                    if isvalid(UD.timer)
                        stop(UD.timer);
                        delete(UD.timer);
                    end
                catch
                end
                delete(obj);
            end
            
            function txt = structToText(structarr)
                % Extract field data
                fields = repmat(fieldnames(structarr), numel(structarr), 1);
                values = struct2cell(structarr);
                
                % Convert all numerical values to strings
                idx = cellfun(@isnumeric, values);
                values(idx) = cellfun(@num2str, values(idx), 'UniformOutput', 0);
                
                % save fields as text
                n = length(fields);
                txt = cell(1,n);
                for i=1:n
                    txt{i} = sprintf('%s = %s', fields{i},values{i});
                end
            end
            
            function timerfcn(~,~,rob,hf)
                if ~ishandle(hf)
                    return
                end
                if strcmp(hf.BeingDeleted,'on')
                    return
                end
                UD = get(hf,'UserData');
                s = rob.getAllSensors;
                txt = structToText(s);
                oldfig = get(groot,'CurrentFigure');
                figure(hf);
                delete(UD.h);
                n = length(txt);
                n1 = round(n/2);
                UD.h(1) = text(-.1,0.5,txt(1:n1),'Interpreter','none');
                UD.h(2) = text(.4,0.5,txt(n1+1:n),'Interpreter','none');
                set(hf,'UserData',UD);
                if ishandle(oldfig) && strcmp(get(oldfig, 'type'), 'figure')
                    figure(oldfig);
                end
                
                
            end
            
        end % testSensors
        function R = getLightBumpers(this)
            % Retrieves the state of all Light Bumper Sensors
            % Return value is a structure containing all results
            
            try
                R = struct(); %Initialize preliminary return values
                this.sendCommand( [149,6,46:51]);
                s = this.getData(6,'uint16');
                
                R.left  = s(1);
                R.leftFront  = s(2);
                R.leftCenter  = s(3);
                R.rightCenter  = s(4);
                R.rightFront  = s(5);
                R.right  = s(6);
                
            catch err
                this.showError(err);
            end
        end % getLightBumpers
        function R = getImage(this)
            R = [];
            if strcmp(this.comID,'sim')
                disp('This function does not work in the simulator.')
                return
            end
            if isempty(this.imageURL)
                disp('This function only works with network connections.');
                return
            end
            R = imread(this.imageURL);
        end % getImage
        function R = getData(this, numitems, datatype)
            % Get raw data from the robot
            % This is a low-level routine and typically not used by the end-user
            
            if nargin<2, numitems=1; end
            if nargin<3, datatype = 'uint8'; end
            fprintf('Reading %u items of type %s\n',numitems,datatype);
            switch datatype
                case {'uint8','int8','uint16','int16'}
                    R = double(fread(this.port, numitems, datatype));
                otherwise
                    warning('Unknown datatype %s',datatype);
            end
        end % getData
        function R = getCliffSensors(this)
            % Determine the state of all Cliff Sensors
            % Return value is a structure containing all results
            
            try
                R = struct(); %Initialize preliminary return values
                this.sendCommand( [149,4,28,29,30,31]);
                s = this.getData(4,'uint16');
                
                R.left = s(1);
                R.leftFront = s(2);
                R.rightFront = s(3);
                R.right = s(4);
                
            catch err
                this.showError(err);
            end
            
        end % getCliffSensors
        function R = getButtons(this)
            % Get state of robot Buttons
            
            R = [];
            
            try
                this.sendCommand( [142,18]);
                stat = this.getData();
                R = this.decodeButtons(stat);
            catch err
                this.showError(err)
            end
            
        end % getButtons
        function R = getBumpers(this)
            % Specifies the state of the bump and wheel drop sensors, either triggered
            % or not triggered.
            
            %Initialize preliminary return values
            R = struct();
            R.right = nan;
            R.left = nan;
            R.front = nan;
            R.leftWheelDrop = nan;
            R.rightWheelDrop = nan;
            
            try
                this.sendCommand( [142,7]);
                data = this.getData();
                R = this.decodeBumpers(data);
            catch err
                this.showError(err);
            end
        end % getBumpers
        function R = getBatteryInfo(this)
            % Retrieves the current battery information
            
            try
                this.sendCommand([142,3]);
                R.chargingState = this.getData();
                R.voltage = this.getData(1,'uint16')/1000;
                R.current =  this.getData(1,'int16')/1000;
                R.temperature = this.getData(1,'int8');
                R.charge = this.getData(1,'uint16')/1000;
                R.capacity = this.getData(1,'uint16')/1000;
                R.percent=R.charge/R.capacity*100;
            catch err
                this.showError(err);
            end
            
        end % getBatteryInfo
        function R = getAllSensors(this)
            % Reads all sensors and returns a structure
            R = struct();
            
            
            try
                
                %% Get (142) ALL(0) data fields (Change to packet set 6 WRS)
                this.sendCommand([142 6]);
                
                %% Read data fields
                data = this.getData();
                R.bumpers = this.decodeBumpers(data);
                
                R.Wall = this.getData();  %0 no wall, 1 wall
                
                % ignore binary cliff data
                this.getData(4); % no cliff, 1 cliff
                
                R.virtWall = this.getData(); %0 no wall, 1 wall
                
                byte = this.getData();
                R.SideCurrentOver  = bitset(byte,1);  % 0 no over curr, 1 over Curr
                R.MainCurrentOver = bitset(byte,3);  % 0 no over curr, 1 over Curr
                R.LeftCurrentOver = bitset(byte,4);  % 0 no over curr, 1 over Curr
                R.RightCurrentOver = bitset(byte,5);  % 0 no over curr, 1 over Curr
                
                R.Dirt = this.getData();
                this.getData();
                
                R.RemoteCode =  this.getData(); % could be used by remote or to communicate with sendIR command
                data = this.getData();
                R.buttons = this.decodeButtons(data);
                %buttnames = fieldnames(buttons);
                %for i=1:length(buttnames)
                %bname = buttnames{i};
                %capname = bname;
                %capname(1) = upper(capname(1));
                %R.(['Button' capname]) = buttons.(bname);
                %end
                R.Dist = this.getData(1,'int16')/1000; % convert to Meters, signed, average dist wheels traveled since last time called...caps at +/-32
                R.Angle = this.getData(1,'int16')*pi/180; % convert to radians, signed,  since last time called, CCW positive
                
                R.ChargingState = this.getData(); % this wasn't returend in original version
                R.Volts = this.getData(1,'uint16')/1000;
                R.Current = this.getData(1,'int16')/1000; % neg sourcing, pos charging
                R.Temp  = this.getData(1,'int8');
                R.Charge =  this.getData(1,'uint16'); % in mAhours
                R.Capacity =  this.getData(1,'uint16');
                R.pCharge = R.Charge/R.Capacity *100;  % May be inaccurate
                
                % additional data from packet set 6
                R.WallSignal = this.getData(1,'uint16');
                R.cliff.left = this.getData(1,'uint16');
                R.cliff.leftFront = this.getData(1,'uint16');
                R.cliff.rightFront = this.getData(1,'uint16');
                R.cliff.right = this.getData(1,'uint16');
                this.getData(3);
                R.ChargingSourcesAvailable = this.getData();
                R.OIMode = this.getData();
                R.SongNumber = this.getData();
                R.SongPlaying = this.getData();
                R.NumberOfStreamPackets = this.getData();
                R.RequestedVelocity = this.getData(1,'int16');
                R.RequestedRadius = this.getData(1,'int16');
                R.RequestedRightVelocity = this.getData(1,'int16');
                R.RequestedLeftVelocity = this.getData(1,'int16');
                
                % additional types on Create2
                if this.bCreate2
                    this.sendCommand( [142 101]);
                    R.LeftEncoderCounts = this.getData(1,'uint16');
                    R.RightEncoderCounts = this.getData(1,'uint16');
                    this.getData();
                    R.lightBumper.left = this.getData(1,'uint16');
                    R.lightBumper.leftFront = this.getData(1,'uint16');
                    R.lightBumper.leftCenter = this.getData(1,'uint16');
                    R.lightBumper.rightCenter = this.getData(1,'uint16');
                    R.lightBumper.rightFront = this.getData(1,'uint16');
                    R.lightBumper.right = this.getData(1,'uint16');
                    R.irOpcode.left = this.getData();
                    R.irOpcode.right = this.getData();
                    R.LeftMotorCurrent = this.getData(1,'int16');
                    R.RightMotorCurrent = this.getData(1,'int16');
                    R.MainBrushMotorCurrent = this.getData(1,'int16');
                    R.SideBrushMotorCurrent = this.getData(1,'int16');
                    R.Statis = this.getData();
                end
                
                %checksum =  fread(rob.port, 1)
            catch err
                this.showError(err);
            end
            
        end % getAllSensors
        function sendCommand(this,cmd)
            % Send raw data to the robot
            % This is a low-level routine and typically not used by the end-user
            
            disp('Sending command');
            disp(uint8(cmd));
            this.flushBuffer;
            fwrite(this.port,cmd)
            pause(this.timedelay);
            
        end % sendCommand
        function setDayTime(this, day, hour, minute)
            try
                this.sendCommand([168,day,hour,minute]);
            catch err
                this.showError(err);
            end
        end % setDayTime
        function setDriveVelocity(this, rightWheelVel, leftWheelVel )
            %  Specify linear velocity of left wheel and right wheel in meters/ sec
            %  with a min/max of [-0.5, 0.5].
            %  Negative velocity is backward.  Caps overflow.
            
            if nargin<2, rightWheelVel = this.defaultVelocity; end
            if nargin<3, leftWheelVel = rightWheelVel; end
            try
                rightWheelVel = int16(min( max(1000* rightWheelVel, -500) , 500));
                leftWheelVel = int16(min( max(1000* leftWheelVel, -500) , 500));
                this.sendCommand([145, this.packBytes(rightWheelVel), this.packBytes(leftWheelVel)]);
            catch err
                this.showError(err);
            end
            
        end % setDriveVelocity
        function setDriveVelocityRadius(this, FwdVel, Radius)
            %  Moves Roomba by setting forward vel and turn radius
            %  FwdVel is forward vel in m/sec [-0.5, 0.5],
            %  Radius in meters, postive turns left, negative turns right [-2,2].
            %    Special cases: Straight = inf
            %    Turn in place clockwise = -eps
            %    Turn in place counter-clockwise = eps
            if nargin<2, FwdVel = this.defaultVelocity; end
            if nargin<3, Radius = inf; end
            try
                %% Convert to millimeters
                FwdVelMM = int16(min( max(FwdVel,-.5), .5)*1000);
                if isinf(Radius)
                    RadiusMM = int16(32768);
                elseif Radius == eps
                    RadiusMM = int16(1);
                elseif Radius == -eps
                    RadiusMM = int16(-1);
                else
                    RadiusMM = int16(min( max(Radius*1000,-2000), 2000));
                end
                
                this.sendCommand( [137, this.packBytes(FwdVelMM), this.packBytes(RadiusMM)]);
            catch err
                this.showError(err);
            end
            
        end % setDriveVelocityRadius
        function reset(this,bShowBootData)
            if nargin<2, bShowBootData=0; end
            % reset the bot and display bootup info
            disp('***Roomba Reset Start');
            this.sendCommand(7); % reset
            if bShowBootData
                disp('***Start of reset data');
                pause(6);
                datalen = this.port.bytesAvailable;
                if datalen>0
                    txt = fread(this.port,datalen,'uint8');
                    txt(txt==13) = [];  % remove carriage returns that cause extra blank line
                    fprintf('%s\n',txt)
                end
                disp('***End of reset data');
            end
            disp('***Roomba Reset Done');
        end % reset
        function checkVersion(~)
            %Check current version of this toolbox
            
            www_site = 'http://ef.engr.utk.edu/ef230/projects/roomba/';
            pkg_name = 'MatlabToolboxroomba';
            thisversion = 3.13;
            
            webversion = urlread([www_site pkg_name '-version.txt']);
            webversion = str2double(webversion);
            thisversiontxt = sprintf('%.02f',thisversion);
            webversiontxt = sprintf('%.02f',webversion);
            msg = { ['Current version: ' thisversiontxt ]
                ['Latest version:  ',webversiontxt ]
                };
            fprintf('%s\n',msg{:});
            
        end % checkVersion
        function showCamera(this)
            % display a live camera feed in a web browser window
            if strcmp(this.comID,'sim')
                warning('This function does not work in the simulator.')
                return
            end
            try
                web(this.cameraURL);
            catch
                warning('Error connecting to this.cameraURL');
            end
        end% showCamera
        function dock(rob)
            % dock the robot
            batt = rob.getBatteryInfo;
            if batt.chargingState~=0
                disp('Already charging - can''t dock');
                return;
            end
            rob.setLEDDigits('HOME');
            rob.songPlay(this.tunes.dock);
            rob.sendCommand(143);
        end % dock
        
        function connect(this, botID, comID, netID )
            % Initialize communication with robot
            % input: rob : roomba object
            %        botID : string or integer (0 for simulator)
            %        comID : 'wifi' (Default)
            %              : 'bt'
            %              : com port name (comxx or /)
            %        netID : 'lan'  192.168.1.2xx (default)
            %              : 'router'   efdroombaserver.engr.utk.edu
            %              : IP name or number
            %
            % output:  updated object defining the robot and its connection
            %
            % EXAMPLES:
            %   r = r.connect(0)                % simulator
            %   r = r.connect(1,'com3')         % by number, PC tethered
            %   r = r.connect('c3po')           % by name, PC tethered
            %   r = r.connect('/dev/ttyusb0',2) % mac tethered
            %   r = r.connect('wifi',4)          % 192.168.1.204
            %   r = r.connect('wifi',4,'lan')    % 192.168.1.204
            %   r = r.connect('bt',4)           % bluetooth name roo4
            %   r = r.connect('bt',4,'router')  % bluetooth name roo4, wifi via router
            
            % ROUTING
            %   all of the roombas are configured to connect to the local wifi router
            %   named efdroombaserver.engr.utk.edu with local address of 192.168.1.1
            %   the bots will have a local wifi address of 192.168.1.xx
            %   The router must have port forwarding in place so that external requests
            %   get forwarded to the correct roomba
            %             60xx maps to 192.168.1.2xx:2222  (command server)
            %             70xx maps to 192.168.1.2xx:22  (ssh)
            %             80xx maps to 192.168.1.2xx:80  (http)
            %             90xx maps to 192.168.1.2xx:2217  (net2tty)
            %   string that doesn't start with a / or 'com' or
            %             use as a host name for a TCPIP connection
            %             ex. RoombaInit('efdpi006.nomad.utk.edu')
            
            if nargin<2, comID = 'wifi'; end
            if nargin<3, botID=0; end
            if nargin<4, netID = 'lan'; end
            
            % if the botID is text convert it to a number
            if ~isnumeric(botID)
                this.botID = find(strcmpi(this.botList,botID));
                if this.botID==0
                    error('Unknown bot name: %s',botID);
                end
            end
            
            % validate botID
            this.botID = botID;
            if botID==0
                this.botName='Simulator';
                comID = 'sim';
            elseif botID>0 && botID<length(this.botList)
                this.botName = this.botList{botID};
            else
                this.botName = 'Unknown';
            end
            
            % determine network address (this is done here because it
            % may be needed by the command connection
            switch netID
                case 'lan'
                    this.server.IP = ['192.168.1.' num2str(200+this.botID)];
                    this.server.control = 2217;
                    this.server.command = 2222;
                    this.server.web = 80;
                case 'router'
                    this.server.IP = 'efdroombaserver.engr.utk.edu';
                    this.server.control = 9000+this.botID;
                    this.server.command = 6000+this.botID;
                    this.server.web = 8000+this.botID;
                otherwise
                    this.server.IP = netID;
                    this.server.control = 2217;
                    this.server.command = 2222;
                    this.server.web = 80;
            end
            this.imageURL = sprintf('http://%s:%u/cam_pic.php',this.server.IP,this.server.web);
            this.cameraURL = sprintf('http://%s:%u',this.server.IP,this.server.web);
            
            % serial (tethered) connection
            if isnumeric(comID)
                comname = strcat('COM', num2str(comID));
                comID='com';
            elseif comID(1)=='/'
                comname = comID;
                comID='com'
            elseif length(comID)>3 && strcmpi(comID(1:3),'com')
                comname = comID;
                comID='com'
            end
            
            this.comID = comID; % store it
            if strcmp(comID,'sim')
                comID='wifi';
                this.server.IP = 'localhost';
                disp('Starting simulator, please wait');

                % set this window to be right half of screen
                ss = get(0,'screensize');
                screen_width = ss(3);
                screen_height = ss(4);
                disp('Setting window position and layout');
                desktop = com.mathworks.mde.desk.MLDesktop.getInstance;
                frame = desktop.getMainFrame;
                frame.setBounds(java.awt.Rectangle(screen_width/2,0,screen_width/2,screen_height-50));
                tic;
                % system('matlab -nodesktop -softwareopengl -r roombaSimGUI &');
                system('matlab -softwareopengl -r roombaSimGUI &');
                if exist('./server.tmp','file'), delete('./server.tmp'); end
                while ~exist('./server.tmp','file')
                    pause(1)
                    fprintf('...waiting on simulator (%.0f)\n',toc);
                end
                delete('./server.tmp');
                pause(1);
                disp('Simulator ready');
            end
            % TODO - need to add error handling
            switch comID
                case 'com'
                    % a serial connection
                    a = instrfind('portID',comname);
                    if ~isempty(a)
                        disp('That com port is in use.   Closing it.')
                        fclose(a);
                        pause(1)
                        delete(a);
                        pause(1)
                    end
                    disp(['Defining com port: ' comname]);
                    this.port = serial(comname,'BaudRate', 115200);
                    %this.portName = comname;
                    set(this.port,'Terminator','LF')
                    
                case 'wifi'
                    % a tcpip connection
                    disp('Resetting Roomba Network Server');
                    this.sendSystemCommand('reset-roomba');
                    pause(1)
                    disp(['Defining network port: ' this.server.IP ':' num2str(this.server.control)]);
                    this.port = tcpip(this.server.IP,this.server.control);
                    set(this.port,'Terminator','')
                case 'bt'
                    % Bluetooth
                    btname = ['roo' num2str(this.botID)];
                    disp('Resetting Roomba Bluetooth Server');
                    this.sendSystemCommand('reset-roomba-bluetooth');
                    pause(1)
                    disp(['Defining bluetooth port: ' btname]);
                    this.port = Bluetooth(btname,1);
                    set(this.port,'Terminator','')
                otherwise
                    error('Unknown comID: %s',comID)
            end
            
            set(this.port,'InputBufferSize',1024)
            set(this.port, 'Timeout', 2)
            set(this.port, 'ByteOrder','bigEndian');
            set(this.port, 'Tag', 'Roomba');
            disp('Opening connection to Roomba...');
            fopen(this.port);
            
            % Confirm two way communication
            
            disp('Setting Roomba to Control Mode...');
            % Start! and see if its alive
            this.sendCommand(128);
            pause(0.1)
            
            % This code puts the robot in CONTROL(132) mode, which means does NOT stop
            % when cliff sensors or wheel drops are true; can also run while plugged
            % into charger
            this.sendCommand(132);
            pause(0.1)
            
            disp('starting connection lights')
            % light LED
            this.setLEDs('spot,dock');
            this.setLEDCenter(100,50);
            
            disp('starting connect song')
            if ~this.quiet
                this.songPlay(this.tunes.connect);
            end
            disp('showing battery charge');
            this.showBatteryCharge();
            disp('setting start time');
            this.timeStart();
            disp('Connect complete');
            
        end % connect
        function sendSystemCommand(this,cmd)
            if strcmp(this.comID,'sim')
                return;
            end
            tmp = tcpip(this.server.IP,this.server.command);
            fopen(tmp);
            fwrite(tmp,cmd);
            fclose(tmp);
            delete(tmp);
        end % sendSystemCommand
        function showBatteryCharge(this)
            % Display the current battery charge (%) on the LED
            battery = this.getBatteryInfo()
            this.setLEDDigits(sprintf('b%3.0f',battery.percent));
        end % showBatteryCharge
        function timeStart(this)
            % reset the timer
            this.timeZero = now;
        end % timeStart
        function R = timeGet(this)
            % get # of seconds since timer was started
            R = (now-this.timeZero)*24*60*60;
        end % timeGet
        function timeShow(this)
            %shows the elapsed time in seconds
            sec = this.timeGet;
            txt = sprintf('%4.0f',sec);
            this.setLEDDigits(txt);
        end % showElapsedTime
        function setLEDDigits(this, txt)
            % Set text shown in 4 digit LED display
            if nargin<2, txt=''; end
            txt = [txt '    ']; % append blanks to ensure 4 bytes are available
            disp('in setLEDDigits')
            disp(txt)
            try
                this.sendCommand([164,txt(1:4)]);
                this.LEDs.digits = txt(1:4);
                this.LEDs.segCodes = [0 0 0 0];
            catch err
                this.showError(err);
            end
        end % setLEDDigits
        function setLEDs(this, ledNames, bOnOff)
            % ledNames : comma separated text string specifying which LEDs
            % should be affected
            %   valid names are
            %   All,Sun,Mon,Tue,Wed,Thu,Fri,Sat,Schedule,Clock,AM,PM,Colon,Dirt,Spot,Dock,Check
            %   defaults to All
            % bOnOff: 0 or 1, defaults to 1
            % Ex: rob.setLEDs('sat,am,colon',1);
            if nargin<2, ledNames='All'; end
            if nargin<3, bOnOff = 1; end
            % order is important in these cell arrays
            % the order should match the bit position
            dayNames = {'Sun','Mon','Tue','Wed','Thu','Fri','Sat'};
            schedNames = {'colon','pm','am','clock','schedule'};
            otherNames = {'dirt','spot','dock','check'};
            if strcmpi(ledNames,'all')
                ledNames = [strjoin(dayNames,','),',' strjoin(schedNames,','), ',', strjoin(otherNames,',')];
            end
            parts = strsplit(ledNames,',');
            for i=1:length(parts)
                code = strtrim(parts{i});
                fnd = 0;
                b1 = find(strcmpi(dayNames,code));
                if b1, this.LEDs.dayByte = bitset(this.LEDs.dayByte,b1,bOnOff); fnd=1; end
                b2 = find(strcmpi(schedNames,code));
                if b2, this.LEDs.schedByte = bitset(this.LEDs.schedByte,b2,bOnOff); fnd=1; end
                b3 = find(strcmpi(otherNames,code));
                if b3, this.LEDs.otherByte = bitset(this.LEDs.otherByte,b3,bOnOff); fnd=1; end
                if ~fnd, fprintf('Unrecognized code "%s"\n',code);end
            end
            
            try
                this.sendCommand([139,this.LEDs.otherByte,this.LEDs.centerColor,this.LEDs.centerIntensity]);
                this.sendCommand([162,this.LEDs.dayByte,this.LEDs.schedByte]);
            catch err
                this.showError(err);
            end
            
        end % setLEDs
        function setLEDCenter(this, color, intensity)
            % Set the center button color and intensity
            % color: 0 is pure green, 255 is pure red.
            % intensity : 0 - 255
            if nargin<2, color=255; end
            if nargin<3, intensity=255; end
            this.LEDs.centerColor = color;
            this.LEDs.centerIntensity = intensity;
            try
                this.sendCommand([139,this.LEDs.otherByte,color,intensity]);
            catch err
                this.showError(err);
            end
            
        end % setLEDs
        function songPlay(this,songtext,waitmode,songnum)
            % Play a song of arbitrary length defined by a readable text string
            % Since the roomba only supports 16 notes in a 'song',
            % and it has a relatively low-level coding for the note data,
            % this function provides the capability to play a longer song
            % and it allows the notes be defined as a text string that is semi-readable
            
            % inputs:   rob - structure describing robot connection
            %           songtext - text describing the song to be played (format details below)
            %           waitmode - true - this function waits on the song to end
            %                      false - function starts the song playing, if the
            %                      song is longer than 16 notes, the user is
            %                      responsible for calling songPlayUpdate to
            %                      send additional 'chunks' at appropriate times
            %                      (default=true)
            %           songnum - 'slot' in robot to store the song (default=3)
            
            % If waitmode is true, this routine splits the requested notes up into 16 note chunks,
            % defines the song on the roomba, and then plays and waits.
            % It repeats this process until the entire song is played.
            
            % if waitmode is false, the user must continually call
            % songPlayUpdate until the song is finished. See the posted example.
            
            % Song Format - a song is a comma separated list of codes
            %    The first letter of the code determines its meaning
            %    Codes:
            %        O : switches octaves - default octave is 4, and octaves 1-7 are valid (see chart)
            %        T : switches tempo - specify as beats/minute, default is 120
            %        A-G : A musical note
            %           These optional values can follow a note;
            %               #(sharp) or b(flat) or %(natural),
            %               one or more octave modifiers ^(up) or v(down)
            %               a numeric expression for the duration.
            %               The default duration is a 1/4 note. The duration can any valid arithmetic
            %               expression that starts with an operator (* or /)
            %        K : defines a key - must be followed by two characters
            %        Anything else : a rest
            %            the rest can contain the duration expression
            %
            % Example notes
            %      O2 - set default octave as 2
            %      C - C quarter note
            %      C^ - C quarter note one octave above the default octave
            %      G# - G sharp quarter note
            %      Bb/2 - B flat eighth note
            %      F#*2 - F sharp half note
            %      R*2 - half note rest
            %      D*3/2 or D*1.5 - dotted quarter note
            %      E*2*1.5 - dotted half note
            %      KF# = sets the 'key' so that all subsequent F's will be played as F#
            %
            % Example Songs
            %      charge = 'O5,T400,C,E,G,C^*2,G,C^*4'
            %      mary = 'T180,O4,E,D,C,D,E,E,E*2'
            %      fastscale = 'T400,C,D,E,F,G,A,B,C^'
            
            
            if nargin<3, waitmode=1; end
            if nargin<4, songnum=3; end
            
            songdata = this.songParse(songtext);
            this.song.isPlaying=1;
            if waitmode
                % play a long song by redefining song #songnum as needed
                for i=1:32:length(songdata)
                    this.songStore(songnum,songdata(i:end));
                    this.songPlayRaw(songnum,1); % play and wait
                end
                this.song.isPlaying=0;
                this.song.index=0;
            else
                this.songStore(songnum,songdata); % only stores the first 16 notes
                this.songPlayRaw(songnum,0); % start playing the song without waiting
                this.song.num = songnum;
                this.song.data = songdata; % save the songdata
                this.song.index = 1; % current location
            end
        end % SongPlay
        function songPlayUpdate(this)
            % Updates the currently playing asyncronous song
            % This is typically called during a processing loop
            
            if nargin<1
                warning('Invalid input')
                return;
            end
            
            if ~isprop(this,'song')
                warning('song field does not exist');
                return;
            end
            
            if ~this.song.isPlaying
                return % not currently playing a song
            end
            
            this.sendCommand([142,37]);
            playing = this.getData();
            
            if playing
                return; % still playing part of a song
            end
            
            nextloc = this.song.index + 32;
            if nextloc <= length(this.song.data)
                this.song.index = nextloc;
                this.songStore(this.song.num,this.song.data(nextloc:end)); % send this segment
                this.songPlayRaw(this.song.num,0); % play and continue
            else
                this.song.index = 0;
                this.song.isPlaying = 0;
            end
            
        end % songPlayUpdate
        function songPlayRaw(this, songnumber, waitflag)
            % this function plays a song already stored on the roomba
            % it gives the option of waiting or returning immediately
            % note that 'songs' on the roomba are a max of 16 notes.
            
            if nargin<3
                waitflag=0;
            end
            try
                this.sendCommand([141,songnumber]); % play the song
                if waitflag
                    playing=true;
                    while playing
                        this.sendCommand([142,37]); % check if still playing
                        playing = this.getData();
                        pause(0.1);
                    end
                end
            catch err
                this.showError(err);
            end
            
        end % songPlayRaw
        function songStore(this, songnumber, songdata)
            % Store a song on the Roomba in its own format
            % see the documentation for the note codes
            % the max number of notes is 16
            % the length of each note is given in 1/64's of a second
            
            cnt = length(songdata)/2;
            if cnt>16
                %disp('Warning: song truncated');
                cnt=16;
            end
            if round(cnt)~=cnt
                disp('Warning: invalid song length - must be multiple of 2');
                cnt = floor(cnt);
            end
            
            try
                this.sendCommand( [140,songnumber,cnt,songdata(1:cnt*2)]);
            catch err
                this.showError(err);
            end
            
        end % songStore
        function setLEDDigitsRaw(rob, LEDS)
            % set individual segments of the 4 7-segment LED displays
            % LEDS should be an array of 4 bytes, with each byte representing one of the characters
            % Enable a segment by setting the appropriate bit
            % Bits are numbered as shown below
            %    0
            %  5   1
            %    6
            %  4   2
            %    3
            %
            disp('Due to a bug in the Create2 Firmware, this command does not work correctly');
            
            try
                rob.sendCommand( [163,LEDS]);
                rob.LEDs.segCodes = LEDS;
                rob.LEDs.digits='';
            catch err
                rob.showError(err);
            end
        end %setLEDDigitsRaw
        function Distance = getDistance(this)
            % Gives the distance traveled in meters since last requested.
            % Positive values indicate travel in the forward direction.
            % If not polled frequently enough, it is
            % capped at its minimum or maximum of +/- 32.768 meters.
            
            %Initialize preliminary return values
            Distance = nan;
            try
                this.sendCommand( [142,19]);
                Distance = double(this.getData(1,'int16'))/1000;
                
                % BIGTIME KLUDGE FOR CREATE2 BUG - TODO: remove when fixed
                if this.bCreate2
                    %Distance = -Distance*8;
                end
                
                if (Distance > 32) || (Distance <-32)
                    disp('Warning:  May have overflowed')
                end
            catch err
                this.showError(err);
            end
        end % getDistance
        function Angle = getAngle(this)
            % Displays the angle in degrees that the robot has turned since the angle was last requested.
            % Counter-clockwise angles are positive
            
            %Initialize preliminary return values
            Angle = nan;
            
            try
                this.sendCommand( [142,20]);
                Angle = double(this.getData(1,'int16'));
            catch err
                this.showError(err);
            end
            
        end % getAngle
        function R = getWheelEncoders(this)
            % Returns the encoder counts for both wheels
            
            R.leftEncoderCount = nan;
            R.rightEncoderCount = nan;
            try
                this.sendCommand( [149,2,43,44]);
                R.leftEncoderCount = double(this.getData(1,'int16'));
                R.rightEncoderCount = double(this.getData(1,'int16'));
            catch err
                this.showError(err);
            end
                       
        end % getWheelEncoders
    end % public methods
    methods (Access=private)
        function R = decodeBumpers(~, byte)
            % Decode a value that contains bumper information
            % This is a low-level routine and typically not used by the end-user
            
            % note - MATLAB numbers bits from 1 to 8
            R.right = bitget(byte,1);
            R.left = bitget(byte,2);
            R.rightWheelDrop = bitget(byte,3);
            R.leftWheelDrop = bitget(byte,4);
            if R.right && R.left
                R.front=1;
                R.right=0;
                R.left=0;
            else
                R.front=0;
            end
        end % decodeBumpers
        function R = decodeButtons(~, byte)
            % Decode a value that contains button press information
            % This is a low-level routine and typically not used by the end-user
            
            % note - MATLAB numbers bits from 1 to 8
            R.clock = bitget(byte,8);    % BUG - doesn't work
            R.schedule = bitget(byte,7); % BUG - doesn't work
            R.day = bitget(byte,6);
            R.hour = bitget(byte,5);
            R.minute = bitget(byte,4);
            R.dock = bitget(byte,3);
            R.spot = bitget(byte,2);
            R.clean = bitget(byte,1);
        end % decodeButtons
        function flushBuffer(this,keep)
            % Flush Buffer
            if nargin<2, keep=0; end
            N = this.port.BytesAvailable();
            while N>keep
                fprintf('Flushing %u bytes\n',N-keep);
                fread(this.port,N-keep);
                N = this.port.BytesAvailable();
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
            %   packBytes(uint16(1))  returns [ 0  1 ]
            %   packBytes(int16(-2))  returns [ 255 254 ]
            
            % tecnhically we need this, but all supported MATLAB platforms are little Endian
            % so we skip the check and always swap bytes
            % [~,~,endian] = computer;
            %if endian=='L'
            data = swapbytes(data);
            % end
            R = typecast(data, 'uint8');
            
        end % end of packBytes function
        function R = unpackBytes(~,data,datatype)
            % Convert a variable (usually an int16 or uint16) into separate bytes
            % suitable for sending as Roomba data.
            % This is a low-level routine and typically not used by the end-user
            
            % Input:
            %   data - byte array  of data to be unpacked
            %   datatype - type to be unpacked to, defaults to uint16
            % Output:
            %   R - an array of the results
            % Example:
            %   unpackBytes([0 1]) returns uint16(1)
            %   unpackBytes([255 254],'int16') returns int16(-2))
            
            if nargin<2
                datatype='uint16';
            end
            
            R = typecast(data, datatype);
            
            % tecnhically we need this, but all supported MATLAB platforms are little Endian
            % so we skip the check and always swap bytes
            % [~,~,endian] = computer;
            % if endian=='L'
            R = swapbytes(R);
            % end
            
        end % end of unpackBytes function
        function waitForAngle(this, angle)
            % Wait for the robot to turn a specified number of degrees
            % This is a low-level routine and typically not used by the end-user
            
            % fprintf('waitForAngle: a=%.0f, to=%u\n',angle);
            
            % convert angle to distance
            rad = this.wheelBase/2; % roomba radius in meters
            d = abs(angle)*pi/180*rad;
            waitForDistance(this,d)
            
        end % waitForAngle
        function waitForDistance(this, distance)
            % Wait for the robot to move a specified distance in meters
            % This is a low-level routine and typically not used by the end-user
            
            % fprintf('waitForDistance: d=%.3f, to=%u\n',distance);
            
            tic
            dtot=0;
            % rob.sendCommand([142,19]); % reset the distance reading
            encleft0 = NaN;
            encright0 = NaN;
            
            while( abs(dtot)<abs(distance) )
                this.songPlayUpdate;
                this.sendCommand([149,2,43,44]);
                pause(0.1);
                encleft = this.getData(1,'uint16')
                encright = this.getData(1,'uint16')
                
                if isnan(encleft0)
                    dd=0;
                else
                    ddleft = abs(double(encleft - encleft0));
                    ddright = abs(double(encright - encright0));
                    if ddleft>30000
                        fprintf('adjusting left for wraparound: %u %u\n',encleft0,encleft);
                        ddleft=abs(ddleft-65536);
                    end
                    if ddright>30000
                        fprintf('adjusting right for wraparound: %u %u\n',encright0,encright);
                        ddright=abs(ddright-65536);
                    end
                    dd = (ddleft+ddright)/this.wheelEncoderCount*pi*this.wheelDiameter*1000/2;
                end
                dtot = dtot + double(dd)/1000;
                encleft0=encleft;
                encright0=encright;
                if toc>this.maxMoveTime
                    disp('maxMoveTime exceeded!');
                    break;
                end
            end
            % fprintf('waitForDistance Done: %.3f\n',dtot);
        end % waitForDistance
        function songdata = songParse(~,songtext)
            % Purpose: Parse a song definition (text string) and create numeric song
            %          data formatted for a roomba robot. Note that you woul
            % Input:
            %    songdata - text string of comma separated notes
            
            tempo = 120; % beats per minute
            
            songtext(songtext==' ') = ''; % remove all blanks
            notes = strsplit(songtext,',');
            octave = 4;
            cnt = 0;
            songdata = [];
            defkey = linspace(0,0,7);
            for n=1:length(notes)
                ntxt=upper(notes{n});
                dura = 1;
                octa = octave;
                
                switch ntxt(1)
                    case 'C'
                        nn = 24;
                    case 'D'
                        nn = 26;
                    case 'E'
                        nn = 28;
                    case 'F'
                        nn = 29;
                    case 'G'
                        nn = 31;
                    case 'A'
                        nn = 33;
                    case 'B'
                        nn = 35;
                    case 'O'
                        octave = str2num(ntxt(2)); %#ok<ST2NM>
                        continue;
                    case 'T'
                        tempo = str2num(ntxt(2:end)); %#ok<ST2NM>
                        continue;
                    case 'K'
                        idk = ntxt(2)-64;
                        switch ntxt(3)
                            case '#'
                                defkey(idk) = 1;
                            case 'B'
                                defkey(idk) = -1;
                            case '%'
                                defkey(idk) = 0;
                        end
                        continue;
                    otherwise
                        nn = 255; % the doc lies - values below 31 still play
                end
                
                % process and remove any octave shifts for this note
                iup = find(ntxt=='^');
                if iup
                    octa = octa + length(iup);
                    ntxt(iup) = [];
                end
                idn = find(ntxt=='V');
                if idn
                    octa = octa - length(idn);
                    ntxt(idn) = [];
                end
                
                
                % process sharp/flat/natural
                usedefkey = true;
                if length(ntxt)>1
                    i=2;
                    if ntxt(2)=='#'
                        nn=nn+1;
                        usedefkey=false;
                        i=3;
                    end
                    if ntxt(2)=='B'
                        nn=nn-1;
                        usedefkey=false;
                        i=3;
                    end
                    if ntxt(2)=='%'
                        usedefkey=false;
                        i=3;
                    end
                    % now get the duration
                    dura = 1;
                    if length(ntxt)>=i
                        if ntxt(i)=='*' || ntxt(i)=='/'
                            duraprefix='1';
                        else
                            duraprefix='';
                        end
                        % must use str2num here because it evaluates expressions
                        dura=str2num([duraprefix ntxt(i:end)]); %#ok<ST2NM>
                    end
                end
                
                if usedefkey && ntxt(1)>='A' && ntxt(1)<='G'
                    nn = nn + defkey(ntxt(1)-'A'+1);
                end
                cnt = cnt + 1;
                j=cnt*2;
                songdata(j-1:j) = [nn+(octa-1)*12, round(dura*64*60/tempo)];
            end
            
        end % songParse
        function showError(~,err)
            % display a generic error message along with the function name
            fprintf('WARNING: Unexpected error! Output may be unreliable.\n');
            fprintf('Error: %s\n',err.message);
            for i=1:length(err.stack)
                fprintf('%s at line %u\n',err.stack(i).name,err.stack(i).line);
            end
        end % showError
        
         
    end % private methods
    methods (Hidden) % Buggy or experimental methods
        function dataStreamStart(this)
            % Start streaming sensor data (experimental)
            % According to the manual, this is better for wifi connections,
            % but I don't see how because it sends even more data, and it
            % seems like you have more potential for get 'old' data
            this.sendCommand([148 length(this.dataStream.packetIDs) this.dataStream.packetIDs]);
        end % dataStreamStart
        function dataStreamStop(this)
            % Stop streaming sensor data
            this.sendCommand([150,0]);
            this.flushBuffer;
        end % dataStreamStop
        function R = dataStreamGetNext(this)
            % get the next streaming packet
            a = this.port.bytesAvailable;
            n = length(this.dataStream.packetIDs) + sum(this.dataStream.packetSizes);
            R=[];
            if a<(n+3)
                disp('not enough data');
                return;
            end
            % look for packet start
            byte=-1;
            while a>=(n+3) && byte~=19
                byte = fread(this.port,1,'uint8');
                a = this.port.bytesAvailable;
            end
            if byte~=19
                disp('no packet start');
                return;
            end
            % a valid packet has the size as the second value
            pktsize = fread(this.port,1,'uint8');
            if pktsize~=n;
                disp('not a packet');
                return
            end
            % read the packet data and checksum
            data = fread(this.port,n+1,'uint8');
            % compute and validate checksum
            chk = 19+n+sum(data(1:n));
            if mod(chk+data(end),256) ~= 0
                disp('checksum error');
                return
            end
            % a valid packet has the packetIDs at these locations
            % should not need this test, but keeping it here as an example
            % of how to parse the results
            idx = 1;
            for i=1:length(this.dataStream.packetSizes)
                if i>1
                    idx = idx + 1 + this.dataStream.packetSizes(i-1);
                end
                if data(idx)~=this.dataStream.packetIDs(i)
                    disp('problem with data packet')
                    return
                end
            end
            % everything is OK, return the data
            R = data(1:n);
        end % dataStreamGetNext
        function R = dataStreamGetLast(this)
            R = [];
            cnt = 0;
            n = 2+length(this.dataStream.packetIDs) + sum(this.dataStream.packetSizes);
            this.flushBuffer(2*n);
            while true
                N = this.dataStreamGetNext;
                if isempty(N)
                    break
                end
                R = N;
                cnt = cnt + 1;
            end
            fprintf('%u packets read in dataStreamGetLast\n',cnt);
        end % dataStreamGetLast
    end
    methods (Hidden) % stubs  to prevent handle methods from showing up in the help
        
        function lh = addlistener(varargin)
            lh = addlistener@handle(varargin{:});
        end
        function notify(varargin)
            notify@handle(varargin{:});
        end
        function delete(varargin)
            delete@handle(varargin{:});
        end
        function Hmatch = findobj(varargin)
            Hmatch = findobj@handle(varargin{:});
        end
        function p = findprop(varargin)
            p = findprop@handle(varargin{:});
        end
        function TF = eq(varargin)
            TF = eq@handle(varargin{:});
        end
        function TF = ne(varargin)
            TF = ne@handle(varargin{:});
        end
        function TF = lt(varargin)
            TF = lt@handle(varargin{:});
        end
        function TF = le(varargin)
            TF = le@handle(varargin{:});
        end
        function TF = gt(varargin)
            TF = gt@handle(varargin{:});
        end
        function TF = ge(varargin)
            TF = ge@handle(varargin{:});
        end
    end % hidden methods
end % classdef