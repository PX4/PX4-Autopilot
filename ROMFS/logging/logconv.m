% This Matlab Script can be used to import the binary logged values of the
% PX4FMU into data that can be plotted and analyzed.

%% ************************************************************************
% PX4LOG_PLOTSCRIPT: Main function
% ************************************************************************
function PX4Log_Plotscript

% Clear everything
clc
clear all
close all

% ************************************************************************
% SETTINGS
% ************************************************************************

% Set the path to your sysvector.bin file here
filePath = 'sysvector.bin';

% Set the minimum and maximum times to plot here [in seconds]
mintime=0;        %The minimum time/timestamp to display, as set by the user [0 for first element / start]
maxtime=0;        %The maximum time/timestamp to display, as set by the user [0 for last element / end]

%Determine which data to plot. Not completely implemented yet.
bDisplayGPS=true;

%conversion factors
fconv_gpsalt=1E-3; %[mm] to [m]
fconv_gpslatlong=1E-7; %[gps_raw_position_unit] to [deg]
fconv_timestamp=1E-6; % [microseconds] to [seconds]

% ************************************************************************
% Import the PX4 logs
% ************************************************************************
ImportPX4LogData();

%Translate min and max plot times to indices
time=double(sysvector.timestamp) .*fconv_timestamp;
mintime_log=time(1);        %The minimum time/timestamp found in the log
maxtime_log=time(end);      %The maximum time/timestamp found in the log
CurTime=mintime_log;        %The current time at which to draw the aircraft position

[imintime,imaxtime]=FindMinMaxTimeIndices();

% ************************************************************************
% PLOT & GUI SETUP
% ************************************************************************
NrFigures=5;
NrAxes=10;
h.figures(1:NrFigures)=0.0;  % Temporary initialization of figure handle array - these are numbered consecutively
h.axes(1:NrAxes)=0.0;        % Temporary initialization of axes handle array - these are numbered consecutively
h.pathpoints=[];             % Temporary initiliazation of path points

% Setup the GUI to control the plots
InitControlGUI();
% Setup the plotting-GUI (figures, axes) itself.
InitPlotGUI();

% ************************************************************************
% DRAW EVERYTHING
% ************************************************************************
DrawRawData();
DrawCurrentAircraftState();

%% ************************************************************************
%  *** END OF MAIN SCRIPT ***
%  NESTED FUNCTION DEFINTIONS FROM HERE ON
%  ************************************************************************


%% ************************************************************************
%  IMPORTPX4LOGDATA (nested function)
%  ************************************************************************
%  Attention:  This is the import routine for firmware from ca. 03/2013.
%              Other firmware versions might require different import 
%              routines.

function ImportPX4LogData()
    % Work around a Matlab bug (not related to PX4)
    % where timestamps from 1.1.1970 do not allow to
    % read the file's size
    if ismac
        system('touch -t 201212121212.12 sysvector.bin');
    end

    % ************************************************************************
    % RETRIEVE SYSTEM VECTOR
    % *************************************************************************
    % //All measurements in NED frame
    %
    % uint64_t timestamp; //[us]
    % float gyro[3]; //[rad/s]
    % float accel[3]; //[m/s^2]
    % float mag[3]; //[gauss] 
    % float baro; //pressure [millibar]
    % float baro_alt; //altitude above MSL [meter]
    % float baro_temp; //[degree celcius]
    % float control[4]; //roll, pitch, yaw [-1..1], thrust [0..1]
    % float actuators[8]; //motor 1-8, in motor units (PWM: 1000-2000,AR.Drone: 0-512)
    % float vbat; //battery voltage in [volt]
    % float bat_current - current drawn from battery at this time instant
    % float bat_discharged - discharged energy in mAh
    % float adc[4]; //remaining auxiliary ADC ports [volt]
    % float local_position[3]; //tangent plane mapping into x,y,z [m]
    % int32_t gps_raw_position[3]; //latitude [degrees] north, longitude [degrees] east, altitude above MSL [millimeter]
    % float attitude[3]; //pitch, roll, yaw [rad]
    % float rotMatrix[9]; //unitvectors
    % float actuator_control[4]; //unitvector
    % float optical_flow[4]; //roll, pitch, yaw [-1..1], thrust [0..1]
    % float diff_pressure; - pressure difference in millibar
    % float ind_airspeed;
    % float true_airspeed;

    % Definition of the logged values
    logFormat{1} = struct('name', 'timestamp',             'bytes', 8, 'array', 1, 'precision', 'uint64',  'machineformat', 'ieee-le.l64');
    logFormat{2} = struct('name', 'gyro',                  'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{3} = struct('name', 'accel',                 'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{4} = struct('name', 'mag',                   'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{5} = struct('name', 'baro',                  'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{6} = struct('name', 'baro_alt',              'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{7} = struct('name', 'baro_temp',             'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{8} = struct('name', 'control',               'bytes', 4, 'array', 4, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{9} = struct('name', 'actuators',             'bytes', 4, 'array', 8, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{10} = struct('name', 'vbat',                 'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{11} = struct('name', 'bat_current',          'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{12} = struct('name', 'bat_discharged',       'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{13} = struct('name', 'adc',                  'bytes', 4, 'array', 4, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{14} = struct('name', 'local_position',       'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{15} = struct('name', 'gps_raw_position',     'bytes', 4, 'array', 3, 'precision', 'uint32',  'machineformat', 'ieee-le');
    logFormat{16} = struct('name', 'attitude',             'bytes', 4, 'array', 3, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{17} = struct('name', 'rot_matrix',           'bytes', 4, 'array', 9, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{18} = struct('name', 'vicon_position',       'bytes', 4, 'array', 6, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{19} = struct('name', 'actuator_control',     'bytes', 4, 'array', 4, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{20} = struct('name', 'optical_flow',         'bytes', 4, 'array', 6, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{21} = struct('name', 'diff_pressure',        'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{22} = struct('name', 'ind_airspeed',         'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');
    logFormat{23} = struct('name', 'true_airspeed',        'bytes', 4, 'array', 1, 'precision', 'float',   'machineformat', 'ieee-le');

    % First get length of one line
    columns = length(logFormat);
    lineLength = 0;

    for i=1:columns
        lineLength = lineLength + logFormat{i}.bytes * logFormat{i}.array;
    end


    if exist(filePath, 'file')

        fileInfo = dir(filePath);
        fileSize = fileInfo.bytes;

        elements = int64(fileSize./(lineLength));

        fid = fopen(filePath, 'r');
        offset = 0;
        for i=1:columns
            % using fread with a skip speeds up the import drastically, do not
            % import the values one after the other
            sysvector.(genvarname(logFormat{i}.name)) = transpose(fread(...
                fid, ...
                [logFormat{i}.array, elements], [num2str(logFormat{i}.array),'*',logFormat{i}.precision,'=>',logFormat{i}.precision], ...
                lineLength - logFormat{i}.bytes*logFormat{i}.array, ...
                logFormat{i}.machineformat) ...
            );
            offset = offset + logFormat{i}.bytes*logFormat{i}.array;
            fseek(fid, offset,'bof');
        end

        % shot the flight time
        time_us = sysvector.timestamp(end) - sysvector.timestamp(1);
        time_s = time_us*1e-6;
        time_m = time_s/60;

        % close the logfile
        fclose(fid);

        disp(['end log2matlab conversion' char(10)]);
    else
        disp(['file: ' filePath ' does not exist' char(10)]);
    end
end

%% ************************************************************************
%  INITCONTROLGUI (nested function)
%  ************************************************************************
%Setup central control GUI components to control current time where data is shown
function InitControlGUI()
    %**********************************************************************
    % GUI size definitions
    %**********************************************************************
    dxy=5; %margins
    %Panel: Plotctrl
    dlabels=120;
    dsliders=200;
    dedits=80;
    hslider=20;
        
    hpanel1=40; %panel1
    hpanel2=220;%panel2
    hpanel3=3*hslider+4*dxy+3*dxy;%panel3.
    
    width=dlabels+dsliders+dedits+4*dxy+2*dxy;  %figure width
    height=hpanel1+hpanel2+hpanel3+4*dxy;       %figure height
        
    %**********************************************************************
    % Create GUI
    %**********************************************************************
    h.figures(1)=figure('Units','pixels','position',[200 200 width height],'Name','Control GUI');
    h.guistatepanel=uipanel('Title','Current GUI state','Units','pixels','Position',[dxy dxy width-2*dxy hpanel1],'parent',h.figures(1));
    h.aircraftstatepanel=uipanel('Title','Current aircraft state','Units','pixels','Position',[dxy hpanel1+2*dxy width-2*dxy hpanel2],'parent',h.figures(1));
    h.plotctrlpanel=uipanel('Title','Plot Control','Units','pixels','Position',[dxy hpanel1+hpanel2+3*dxy width-2*dxy hpanel3],'parent',h.figures(1));
    
    %%Control GUI-elements
    %Slider: Current time
    h.labels.CurTime=uicontrol(gcf,'style','text','Position',[dxy dxy dlabels hslider],'String','Current time t[s]:','parent',h.plotctrlpanel,'HorizontalAlignment','left');
    h.sliders.CurTime=uicontrol(gcf,'style','slider','units','pix','position',[2*dxy+dlabels dxy dsliders hslider],...
        'min',mintime,'max',maxtime,'value',mintime,'callback',@curtime_callback,'parent',h.plotctrlpanel);
    temp=get(h.sliders.CurTime,'Max')-get(h.sliders.CurTime,'Min');
    set(h.sliders.CurTime,'SliderStep',[1.0/temp 5.0/temp]);
    h.edits.CurTime=uicontrol(gcf,'style','edit','position',[3*dxy+dlabels+dsliders dxy dedits hslider],'String',get(h.sliders.CurTime,'value'),...
        'BackgroundColor','white','callback',@curtime_callback,'parent',h.plotctrlpanel);

    %Slider: MaxTime
    h.labels.MaxTime=uicontrol(gcf,'style','text','position',[dxy 2*dxy+hslider dlabels hslider],'String','Max. time t[s] to display:','parent',h.plotctrlpanel,'HorizontalAlignment','left');
    h.sliders.MaxTime=uicontrol(gcf,'style','slider','units','pix','position',[2*dxy+dlabels 2*dxy+hslider dsliders hslider],...
        'min',mintime_log,'max',maxtime_log,'value',maxtime,'callback',@minmaxtime_callback,'parent',h.plotctrlpanel);
    h.edits.MaxTime=uicontrol(gcf,'style','edit','position',[3*dxy+dlabels+dsliders 2*dxy+hslider dedits hslider],'String',get(h.sliders.MaxTime,'value'),...
        'BackgroundColor','white','callback',@minmaxtime_callback,'parent',h.plotctrlpanel);
    
    %Slider: MinTime
    h.labels.MinTime=uicontrol(gcf,'style','text','position',[dxy 3*dxy+2*hslider dlabels hslider],'String','Min. time t[s] to dispay :','parent',h.plotctrlpanel,'HorizontalAlignment','left');
    h.sliders.MinTime=uicontrol(gcf,'style','slider','units','pix','position',[2*dxy+dlabels 3*dxy+2*hslider dsliders hslider],...
        'min',mintime_log,'max',maxtime_log,'value',mintime,'callback',@minmaxtime_callback,'parent',h.plotctrlpanel);
    h.edits.MinTime=uicontrol(gcf,'style','edit','position',[3*dxy+dlabels+dsliders 3*dxy+2*hslider dedits hslider],'String',get(h.sliders.MinTime,'value'),...
        'BackgroundColor','white','callback',@minmaxtime_callback,'parent',h.plotctrlpanel);

    %%Current data/state GUI-elements (Multiline-edit-box)
    h.edits.AircraftState=uicontrol(gcf,'style','edit','Units','normalized','position',[.02 .02 0.96 0.96],'Min',1,'Max',10,'String','This shows the current aircraft state',...
                                    'HorizontalAlignment','left','parent',h.aircraftstatepanel);
    
    h.labels.GUIState=uicontrol(gcf,'style','text','Units','pixels','position',[dxy dxy width-4*dxy hslider],'String','Current state of this GUI',...
                                'HorizontalAlignment','left','parent',h.guistatepanel);
    
end

%% ************************************************************************
%  INITPLOTGUI (nested function)
%  ************************************************************************
function InitPlotGUI()
    
    % Setup handles to lines and text
    h.markertext=[];
    templinehandle=0.0;%line([0 1],[0 5]);   % Just a temporary handle to init array
    h.markerline(1:NrAxes)=templinehandle;   % the actual handle-array to the lines  - these are numbered consecutively
    h.markerline(1:NrAxes)=0.0;
        
    % Setup all other figures and axes for plotting
    %  PLOT WINDOW 1: GPS POSITION
    h.figures(2)=figure('units','normalized','Toolbar','figure', 'Name', 'GPS Position');
    h.axes(1)=axes();
    set(h.axes(1),'Parent',h.figures(2));

    %  PLOT WINDOW 2: IMU, baro altitude
    h.figures(3)=figure('Name', 'IMU / Baro Altitude');
    h.axes(2)=subplot(4,1,1);
    h.axes(3)=subplot(4,1,2);
    h.axes(4)=subplot(4,1,3);
    h.axes(5)=subplot(4,1,4);
    set(h.axes(2:5),'Parent',h.figures(3));
    
    %  PLOT WINDOW 3: ATTITUDE ESTIMATE, ACTUATORS/CONTROLS, AIRSPEEDS,...
    h.figures(4)=figure('Name', 'Attitude Estimate / Actuators / Airspeeds');
    h.axes(6)=subplot(4,1,1);
    h.axes(7)=subplot(4,1,2);
    h.axes(8)=subplot(4,1,3);
    h.axes(9)=subplot(4,1,4);
    set(h.axes(6:9),'Parent',h.figures(4));
    
    %  PLOT WINDOW 4: LOG STATS
    h.figures(5) = figure('Name', 'Log Statistics');
    h.axes(10)=subplot(1,1,1);
    set(h.axes(10:10),'Parent',h.figures(5));
    
end

%% ************************************************************************
%  DRAWRAWDATA (nested function)
%  ************************************************************************
%Draws the raw data from the sysvector, but does not add any
%marker-lines or so
function DrawRawData() 
    % ************************************************************************
    %  PLOT WINDOW 1: GPS POSITION & GUI
    %  ************************************************************************
    figure(h.figures(2));
    % Only plot GPS data if available
    if (sum(double(sysvector.gps_raw_position(imintime:imaxtime,1)))>0) && (bDisplayGPS)
        %Draw data
        plot3(h.axes(1),double(sysvector.gps_raw_position(imintime:imaxtime,1))*fconv_gpslatlong, ...
                        double(sysvector.gps_raw_position(imintime:imaxtime,2))*fconv_gpslatlong, ...
                        double(sysvector.gps_raw_position(imintime:imaxtime,3))*fconv_gpsalt,'r.');
        title(h.axes(1),'GPS Position Data(if available)');
        xlabel(h.axes(1),'Latitude [deg]');
        ylabel(h.axes(1),'Longitude [deg]');
        zlabel(h.axes(1),'Altitude above MSL [m]');
        grid on
        
        %Reset path
        h.pathpoints=0;
    end

    % ************************************************************************
    %  PLOT WINDOW 2: IMU, baro altitude
    %  ************************************************************************
    figure(h.figures(3));
    plot(h.axes(2),time(imintime:imaxtime),sysvector.mag(imintime:imaxtime,:));
    title(h.axes(2),'Magnetometers [Gauss]');
    legend(h.axes(2),'x','y','z');
    plot(h.axes(3),time(imintime:imaxtime),sysvector.accel(imintime:imaxtime,:));
    title(h.axes(3),'Accelerometers [m/s²]');
    legend(h.axes(3),'x','y','z');
    plot(h.axes(4),time(imintime:imaxtime),sysvector.gyro(imintime:imaxtime,:));
    title(h.axes(4),'Gyroscopes [rad/s]');
    legend(h.axes(4),'x','y','z');
    plot(h.axes(5),time(imintime:imaxtime),sysvector.baro_alt(imintime:imaxtime),'color','blue');
    if(bDisplayGPS)
        hold on;
        plot(h.axes(5),time(imintime:imaxtime),double(sysvector.gps_raw_position(imintime:imaxtime,3)).*fconv_gpsalt,'color','red');
        hold off
        legend('Barometric Altitude [m]','GPS Altitude [m]');
    else
        legend('Barometric Altitude [m]');
    end
    title(h.axes(5),'Altitude above MSL [m]');

    % ************************************************************************
    %  PLOT WINDOW 3: ATTITUDE ESTIMATE, ACTUATORS/CONTROLS, AIRSPEEDS,...
    % ************************************************************************
    figure(h.figures(4));
    %Attitude Estimate
    plot(h.axes(6),time(imintime:imaxtime), sysvector.attitude(imintime:imaxtime,:).*180./3.14159);
    title(h.axes(6),'Estimated attitude [deg]');
    legend(h.axes(6),'roll','pitch','yaw');
    %Actuator Controls
    plot(h.axes(7),time(imintime:imaxtime), sysvector.actuator_control(imintime:imaxtime,:));
    title(h.axes(7),'Actuator control [-]');
    legend(h.axes(7),'0','1','2','3');
    %Actuator Controls
    plot(h.axes(8),time(imintime:imaxtime), sysvector.actuators(imintime:imaxtime,1:8));
    title(h.axes(8),'Actuator PWM (raw-)outputs [µs]');
    legend(h.axes(8),'CH1','CH2','CH3','CH4','CH5','CH6','CH7','CH8');
    set(h.axes(8), 'YLim',[800 2200]);
    %Airspeeds
    plot(h.axes(9),time(imintime:imaxtime), sysvector.ind_airspeed(imintime:imaxtime));
    hold on
    plot(h.axes(9),time(imintime:imaxtime), sysvector.true_airspeed(imintime:imaxtime));
    hold off
    %add GPS total airspeed here
    title(h.axes(9),'Airspeed [m/s]');
    legend(h.axes(9),'Indicated Airspeed (IAS)','True Airspeed (TAS)','GPS Airspeed');
    %calculate time differences and plot them
    intervals = zeros(0,imaxtime - imintime);
    for k = imintime+1:imaxtime
        intervals(k) = time(k) - time(k-1);
    end
    plot(h.axes(10), time(imintime:imaxtime), intervals);

    %Set same timescale for all plots
    for i=2:NrAxes
        set(h.axes(i),'XLim',[mintime maxtime]);
    end
    
    set(h.labels.GUIState,'String','OK','BackgroundColor',[240/255 240/255 240/255]);
end

%% ************************************************************************
%  DRAWCURRENTAIRCRAFTSTATE(nested function)
%  ************************************************************************
function DrawCurrentAircraftState()
    %find current data index        
    i=find(time>=CurTime,1,'first');

    %**********************************************************************
    % Current aircraft state label update
    %**********************************************************************
    acstate{1,:}=[sprintf('%s \t\t','GPS Pos:'),'[lat=',num2str(double(sysvector.gps_raw_position(i,1))*fconv_gpslatlong),'°, ',...
                        'lon=',num2str(double(sysvector.gps_raw_position(i,2))*fconv_gpslatlong),'°, ',...
                        'alt=',num2str(double(sysvector.gps_raw_position(i,3))*fconv_gpsalt),'m]'];
    acstate{2,:}=[sprintf('%s \t\t','Mags[gauss]'),'[x=',num2str(sysvector.mag(i,1)),...
                               ', y=',num2str(sysvector.mag(i,2)),...
                               ', z=',num2str(sysvector.mag(i,3)),']'];
    acstate{3,:}=[sprintf('%s \t\t','Accels[m/s²]'),'[x=',num2str(sysvector.accel(i,1)),...
                               ', y=',num2str(sysvector.accel(i,2)),...
                               ', z=',num2str(sysvector.accel(i,3)),']'];
    acstate{4,:}=[sprintf('%s \t\t','Gyros[rad/s]'),'[x=',num2str(sysvector.gyro(i,1)),...
                               ', y=',num2str(sysvector.gyro(i,2)),...
                               ', z=',num2str(sysvector.gyro(i,3)),']'];
    acstate{5,:}=[sprintf('%s \t\t','Altitude[m]'),'[Barometric: ',num2str(sysvector.baro_alt(i)),'m, GPS: ',num2str(double(sysvector.gps_raw_position(i,3))*fconv_gpsalt),'m]'];
    acstate{6,:}=[sprintf('%s \t','Est. attitude[deg]:'),'[Roll=',num2str(sysvector.attitude(i,1).*180./3.14159),...
                               ', Pitch=',num2str(sysvector.attitude(i,2).*180./3.14159),...
                               ', Yaw=',num2str(sysvector.attitude(i,3).*180./3.14159),']'];
    acstate{7,:}=sprintf('%s \t[','Actuator Ctrls [-]:');
    for j=1:4
        acstate{7,:}=[acstate{7,:},num2str(sysvector.actuator_control(i,j)),','];
    end
    acstate{7,:}=[acstate{7,:},']'];
    acstate{8,:}=sprintf('%s \t[','Actuator Outputs [PWM/µs]:');
    for j=1:8
        acstate{8,:}=[acstate{8,:},num2str(sysvector.actuators(i,j)),','];
    end
    acstate{8,:}=[acstate{8,:},']'];
    acstate{9,:}=[sprintf('%s \t','Airspeed[m/s]:'),'[IAS: ',num2str(sysvector.ind_airspeed(i)),', TAS: ',num2str(sysvector.true_airspeed(i)),']'];
    
    set(h.edits.AircraftState,'String',acstate);
    
    %**********************************************************************
    % GPS Plot Update
    %**********************************************************************
    %Plot traveled path, and  and time.
    figure(h.figures(2));
    hold on;
    if(CurTime>mintime+1) %the +1 is only a small bugfix
        h.pathline=plot3(h.axes(1),double(sysvector.gps_raw_position(imintime:i,1))*fconv_gpslatlong, ...
                                    double(sysvector.gps_raw_position(imintime:i,2))*fconv_gpslatlong, ...
                                    double(sysvector.gps_raw_position(imintime:i,3))*fconv_gpsalt,'b','LineWidth',2); 
    end;
    hold off
    %Plot current position
    newpoint=[double(sysvector.gps_raw_position(i,1))*fconv_gpslatlong double(sysvector.gps_raw_position(i,2))*fconv_gpslatlong double(sysvector.gps_raw_position(i,3))*fconv_gpsalt];
    if(numel(h.pathpoints)<=3) %empty path
        h.pathpoints(1,1:3)=newpoint;
    else %Not empty, append new point 
        h.pathpoints(size(h.pathpoints,1)+1,:)=newpoint;
    end
    axes(h.axes(1));    
    line(h.pathpoints(:,1),h.pathpoints(:,2),h.pathpoints(:,3),'LineStyle','none','Marker','.','MarkerEdge','black','MarkerSize',20);
    
    
    % Plot current time (small label next to current gps position) 
    textdesc=strcat('  t=',num2str(time(i)),'s');
    if(isvalidhandle(h.markertext))
            delete(h.markertext); %delete old text
    end 
    h.markertext=text(double(sysvector.gps_raw_position(i,1))*fconv_gpslatlong,double(sysvector.gps_raw_position(i,2))*fconv_gpslatlong,...
                        double(sysvector.gps_raw_position(i,3))*fconv_gpsalt,textdesc);
    set(h.edits.CurTime,'String',CurTime);
        
    %**********************************************************************
    % Plot the lines showing the current time in the 2-d plots
    %**********************************************************************
    for i=2:NrAxes
        if(isvalidhandle(h.markerline(i))) delete(h.markerline(i)); end
        ylims=get(h.axes(i),'YLim');
        h.markerline(i)=line([CurTime CurTime] ,get(h.axes(i),'YLim'),'Color','black');
        set(h.markerline(i),'parent',h.axes(i));
    end
    
    set(h.labels.GUIState,'String','OK','BackgroundColor',[240/255 240/255 240/255]);
end

%% ************************************************************************
%  MINMAXTIME CALLBACK (nested function)
%  ************************************************************************
function minmaxtime_callback(hObj,event) %#ok<INUSL>
    new_mintime=get(h.sliders.MinTime,'Value');
    new_maxtime=get(h.sliders.MaxTime,'Value');

    %Safety checks:
    bErr=false;
    %1: mintime must be < maxtime
    if((new_mintime>maxtime) || (new_maxtime<mintime))
        set(h.labels.GUIState,'String','Error: Mintime cannot be bigger than maxtime! Values were not changed.','BackgroundColor','red');
        bErr=true;
    else
        %2: MinTime must be <=CurTime
        if(new_mintime>CurTime)
            set(h.labels.GUIState,'String','Error: Mintime cannot be bigger than CurTime! CurTime set to new mintime.','BackgroundColor','red');
            mintime=new_mintime;
            CurTime=mintime;
            bErr=true;
        end
        %3: MaxTime must be >CurTime
        if(new_maxtime<CurTime)
            set(h.labels.GUIState,'String','Error: Maxtime cannot be smaller than CurTime! CurTime set to new maxtime.','BackgroundColor','red');
            maxtime=new_maxtime;
            CurTime=maxtime;
            bErr=true;
        end
    end

    if(bErr==false)
        maxtime=new_maxtime;
        mintime=new_mintime;
    end

    %Needs to be done in case values were reset above
    set(h.sliders.MinTime,'Value',mintime);
    set(h.sliders.MaxTime,'Value',maxtime);

    %Update curtime-slider
    set(h.sliders.CurTime,'Value',CurTime);
    set(h.sliders.CurTime,'Max',maxtime);
    set(h.sliders.CurTime,'Min',mintime);
    temp=get(h.sliders.CurTime,'Max')-get(h.sliders.CurTime,'Min'); 
    set(h.sliders.CurTime,'SliderStep',[1.0/temp 5.0/temp]);  %Set Stepsize to constant [in seconds]

    %update edit fields
    set(h.edits.CurTime,'String',get(h.sliders.CurTime,'Value'));
    set(h.edits.MinTime,'String',get(h.sliders.MinTime,'Value'));
    set(h.edits.MaxTime,'String',get(h.sliders.MaxTime,'Value'));

    %Finally, we have to redraw. Update time indices first.
    [imintime,imaxtime]=FindMinMaxTimeIndices();
    DrawRawData();                  %Rawdata only
    DrawCurrentAircraftState();     %path info & markers
end


%% ************************************************************************
%  CURTIME CALLBACK (nested function)
%  ************************************************************************
function curtime_callback(hObj,event) %#ok<INUSL>
    %find current time
    if(hObj==h.sliders.CurTime)
        CurTime=get(h.sliders.CurTime,'Value');
    elseif (hObj==h.edits.CurTime)
        temp=str2num(get(h.edits.CurTime,'String'));
        if(temp<maxtime && temp>mintime)
            CurTime=temp;
        else
            %Error
            set(h.labels.GUIState,'String','Error: You tried to set an invalid current time! Previous value restored.','BackgroundColor','red');
        end
    else
        %Error
        set(h.labels.GUIState,'String','Error: curtime_callback','BackgroundColor','red');        
    end
    
    set(h.sliders.CurTime,'Value',CurTime);
    set(h.edits.CurTime,'String',num2str(CurTime));
    
    %Redraw time markers, but don't have to redraw the whole raw data
    DrawCurrentAircraftState();
end

%% ************************************************************************
%  FINDMINMAXINDICES (nested function)
%  ************************************************************************
function [idxmin,idxmax] = FindMinMaxTimeIndices()
    for i=1:size(sysvector.timestamp,1)
        if time(i)>=mintime; idxmin=i; break; end
    end
    for i=1:size(sysvector.timestamp,1)
        if maxtime==0; idxmax=size(sysvector.timestamp,1); break; end
        if time(i)>=maxtime; idxmax=i; break; end
    end
    mintime=time(idxmin); 
    maxtime=time(idxmax);
end

%% ************************************************************************
%  ISVALIDHANDLE (nested function)
%  ************************************************************************
function isvalid = isvalidhandle(handle)
   if(exist(varname(handle))>0 && length(ishandle(handle))>0) 
        if(ishandle(handle)>0)
            if(handle>0.0)
                isvalid=true;
                return;
            end
        end
   end
   isvalid=false;
end

%% ************************************************************************
%  JUST SOME SMALL HELPER FUNCTIONS (nested function)
%  ************************************************************************
function out = varname(var)
   out = inputname(1);
end

%This is the end of the matlab file / the main function
end
