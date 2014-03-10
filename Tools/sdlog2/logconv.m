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
filePath = 'log001.bin';

% Set the minimum and maximum times to plot here [in seconds]
mintime=0;        %The minimum time/timestamp to display, as set by the user [0 for first element / start]
maxtime=0;        %The maximum time/timestamp to display, as set by the user [0 for last element / end]

%Determine which data to plot. Not completely implemented yet.
bDisplayGPS=true;

%conversion factors
fconv_gpsalt=1; %[mm] to [m]
fconv_gpslatlong=1; %[gps_raw_position_unit] to [deg]
fconv_timestamp=1E-6; % [microseconds] to [seconds]

% ************************************************************************
% Import the PX4 logs
% ************************************************************************
ImportPX4LogData();

%Translate min and max plot times to indices
time=double(sysvector.TIME_StartTime) .*fconv_timestamp;
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

%% ************************************************************************
%  IMPORTPX4LOGDATA (nested function)
%  ************************************************************************
%  Attention:  This is the import routine for firmware from ca. 03/2013.
%              Other firmware versions might require different import 
%              routines.

function ImportPX4LogData()

    % ************************************************************************
    % RETRIEVE SYSTEM VECTOR
    % *************************************************************************
    % //All measurements in NED frame
    
    % Convert to CSV
    %arg1 = 'log-fx61-20130721-2.bin';
    arg1 = filePath;
    delim = ',';
    time_field = 'TIME';
    data_file = 'data.csv';
    csv_null = '';
    
    if not(exist(data_file, 'file'))
        s = system( sprintf('python sdlog2_dump.py "%s" -f "%s" -t"%s" -d"%s" -n"%s"', arg1, data_file, time_field, delim, csv_null) );
    end

    if exist(data_file, 'file')

        %data = csvread(data_file);
        sysvector = tdfread(data_file, ',');

        % shot the flight time
        time_us = sysvector.TIME_StartTime(end) - sysvector.TIME_StartTime(1);
        time_s = uint64(time_us*1e-6);
        time_m = uint64(time_s/60);
        time_s = time_s - time_m * 60;
        
        disp([sprintf('Flight log duration: %d:%d (minutes:seconds)', time_m, time_s) char(10)]);

        disp(['logfile conversion finished.' char(10)]);
    else
        disp(['file: ' data_file ' does not exist' char(10)]);
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
    if (sum(double(sysvector.GPS_Lat(imintime:imaxtime)))>0) && (bDisplayGPS)
        %Draw data
        plot3(h.axes(1),double(sysvector.GPS_Lat(imintime:imaxtime))*fconv_gpslatlong, ...
                        double(sysvector.GPS_Lon(imintime:imaxtime))*fconv_gpslatlong, ...
                        double(sysvector.GPS_Alt(imintime:imaxtime))*fconv_gpsalt,'r.');
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
    plot(h.axes(2),time(imintime:imaxtime),[sysvector.IMU_MagX(imintime:imaxtime), sysvector.IMU_MagY(imintime:imaxtime), sysvector.IMU_MagZ(imintime:imaxtime)]);
    title(h.axes(2),'Magnetometers [Gauss]');
    legend(h.axes(2),'x','y','z');
    plot(h.axes(3),time(imintime:imaxtime),[sysvector.IMU_AccX(imintime:imaxtime), sysvector.IMU_AccY(imintime:imaxtime), sysvector.IMU_AccZ(imintime:imaxtime)]);
    title(h.axes(3),'Accelerometers [m/s²]');
    legend(h.axes(3),'x','y','z');
    plot(h.axes(4),time(imintime:imaxtime),[sysvector.IMU_GyroX(imintime:imaxtime), sysvector.IMU_GyroY(imintime:imaxtime), sysvector.IMU_GyroZ(imintime:imaxtime)]);
    title(h.axes(4),'Gyroscopes [rad/s]');
    legend(h.axes(4),'x','y','z');
    plot(h.axes(5),time(imintime:imaxtime),sysvector.SENS_BaroAlt(imintime:imaxtime),'color','blue');
    if(bDisplayGPS)
        hold on;
        plot(h.axes(5),time(imintime:imaxtime),double(sysvector.GPS_Alt(imintime:imaxtime)).*fconv_gpsalt,'color','red');
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
    plot(h.axes(6),time(imintime:imaxtime), [sysvector.ATT_Roll(imintime:imaxtime), sysvector.ATT_Pitch(imintime:imaxtime), sysvector.ATT_Yaw(imintime:imaxtime)] .*180./3.14159);
    title(h.axes(6),'Estimated attitude [deg]');
    legend(h.axes(6),'roll','pitch','yaw');
    %Actuator Controls
    plot(h.axes(7),time(imintime:imaxtime), [sysvector.ATTC_Roll(imintime:imaxtime), sysvector.ATTC_Pitch(imintime:imaxtime), sysvector.ATTC_Yaw(imintime:imaxtime), sysvector.ATTC_Thrust(imintime:imaxtime)]);
    title(h.axes(7),'Actuator control [-]');
    legend(h.axes(7),'ATT CTRL Roll [-1..+1]','ATT CTRL Pitch [-1..+1]','ATT CTRL Yaw [-1..+1]','ATT CTRL Thrust [0..+1]');
    %Actuator Controls
    plot(h.axes(8),time(imintime:imaxtime), [sysvector.OUT0_Out0(imintime:imaxtime), sysvector.OUT0_Out1(imintime:imaxtime), sysvector.OUT0_Out2(imintime:imaxtime), sysvector.OUT0_Out3(imintime:imaxtime), sysvector.OUT0_Out4(imintime:imaxtime), sysvector.OUT0_Out5(imintime:imaxtime), sysvector.OUT0_Out6(imintime:imaxtime), sysvector.OUT0_Out7(imintime:imaxtime)]);
    title(h.axes(8),'Actuator PWM (raw-)outputs [µs]');
    legend(h.axes(8),'CH1','CH2','CH3','CH4','CH5','CH6','CH7','CH8');
    set(h.axes(8), 'YLim',[800 2200]);
    %Airspeeds
    plot(h.axes(9),time(imintime:imaxtime), sysvector.AIRS_IndSpeed(imintime:imaxtime));
    hold on
    plot(h.axes(9),time(imintime:imaxtime), sysvector.AIRS_TrueSpeed(imintime:imaxtime));
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
    acstate{1,:}=[sprintf('%s \t\t','GPS Pos:'),'[lat=',num2str(double(sysvector.GPS_Lat(i))*fconv_gpslatlong),'°, ',...
                        'lon=',num2str(double(sysvector.GPS_Lon(i))*fconv_gpslatlong),'°, ',...
                        'alt=',num2str(double(sysvector.GPS_Alt(i))*fconv_gpsalt),'m]'];
    acstate{2,:}=[sprintf('%s \t\t','Mags[gauss]'),'[x=',num2str(sysvector.IMU_MagX(i)),...
                               ', y=',num2str(sysvector.IMU_MagY(i)),...
                               ', z=',num2str(sysvector.IMU_MagZ(i)),']'];
    acstate{3,:}=[sprintf('%s \t\t','Accels[m/s²]'),'[x=',num2str(sysvector.IMU_AccX(i)),...
                               ', y=',num2str(sysvector.IMU_AccY(i)),...
                               ', z=',num2str(sysvector.IMU_AccZ(i)),']'];
    acstate{4,:}=[sprintf('%s \t\t','Gyros[rad/s]'),'[x=',num2str(sysvector.IMU_GyroX(i)),...
                               ', y=',num2str(sysvector.IMU_GyroY(i)),...
                               ', z=',num2str(sysvector.IMU_GyroZ(i)),']'];
    acstate{5,:}=[sprintf('%s \t\t','Altitude[m]'),'[Barometric: ',num2str(sysvector.SENS_BaroAlt(i)),'m, GPS: ',num2str(double(sysvector.GPS_Alt(i))*fconv_gpsalt),'m]'];
    acstate{6,:}=[sprintf('%s \t','Est. attitude[deg]:'),'[Roll=',num2str(sysvector.ATT_Roll(i).*180./3.14159),...
                               ', Pitch=',num2str(sysvector.ATT_Pitch(i).*180./3.14159),...
                               ', Yaw=',num2str(sysvector.ATT_Yaw(i).*180./3.14159),']'];
    acstate{7,:}=sprintf('%s \t[','Actuator Ctrls [-]:');
    %for j=1:4
    acstate{7,:}=[acstate{7,:},num2str(sysvector.ATTC_Roll(i)),','];
    acstate{7,:}=[acstate{7,:},num2str(sysvector.ATTC_Pitch(i)),','];
    acstate{7,:}=[acstate{7,:},num2str(sysvector.ATTC_Yaw(i)),','];
    acstate{7,:}=[acstate{7,:},num2str(sysvector.ATTC_Thrust(i)),','];
    %end
    acstate{7,:}=[acstate{7,:},']'];
    acstate{8,:}=sprintf('%s \t[','Actuator Outputs [PWM/µs]:');
    %for j=1:8
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out0(i)),','];
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out1(i)),','];
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out2(i)),','];
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out3(i)),','];
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out4(i)),','];
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out5(i)),','];
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out6(i)),','];
    acstate{8,:}=[acstate{8,:},num2str(sysvector.OUT0_Out7(i)),','];
    %end
    acstate{8,:}=[acstate{8,:},']'];
    acstate{9,:}=[sprintf('%s \t','Airspeed[m/s]:'),'[IAS: ',num2str(sysvector.AIRS_IndSpeed(i)),', TAS: ',num2str(sysvector.AIRS_TrueSpeed(i)),']'];
    
    set(h.edits.AircraftState,'String',acstate);
    
    %**********************************************************************
    % GPS Plot Update
    %**********************************************************************
    %Plot traveled path, and  and time.
    figure(h.figures(2));
    hold on;
    if(CurTime>mintime+1) %the +1 is only a small bugfix
        h.pathline=plot3(h.axes(1),double(sysvector.GPS_Lat(imintime:i))*fconv_gpslatlong, ...
                                    double(sysvector.GPS_Lon(imintime:i))*fconv_gpslatlong, ...
                                    double(sysvector.GPS_Alt(imintime:i))*fconv_gpsalt,'b','LineWidth',2); 
    end;
    hold off
    %Plot current position
    newpoint=[double(sysvector.GPS_Lat(i))*fconv_gpslatlong double(sysvector.GPS_Lat(i))*fconv_gpslatlong double(sysvector.GPS_Alt(i))*fconv_gpsalt];
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
    h.markertext=text(double(sysvector.GPS_Lat(i))*fconv_gpslatlong,double(sysvector.GPS_Lon(i))*fconv_gpslatlong,...
                        double(sysvector.GPS_Alt(i))*fconv_gpsalt,textdesc);
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
    for i=1:size(sysvector.TIME_StartTime,1)
        if time(i)>=mintime; idxmin=i; break; end
    end
    for i=1:size(sysvector.TIME_StartTime,1)
        if maxtime==0; idxmax=size(sysvector.TIME_StartTime,1); break; end
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
