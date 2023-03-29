clear all;
close all;

VIDEO_FILE = '4.mp4';
ESC_FILE = 'flight1_ESC.mat';
LOG_FILE = 'cdata.mat';
OUTPUT_VIDEO_FILE = 'flight0.mp4';

RESYNC = false; % true this if resync should be performed
RETIME = false; % true this if another interval of interest is desired

VIDEO_TEMPLATE_TYPE = 2; % this can be 1 or 2 to modify video templates
% 1: airspeed, video, rc_roll and rc_yaw
% 2: attitude, video, throttle, rc_pitch

%% load log data from log files
load(LOG_FILE);
load(ESC_FILE);

%% reset sync data if resync is neccessary
if (RESYNC)
    landing_frame = -1;
    throttle_pix_index = -1;
end

if (RETIME)
    landing_pix_index = -1;
end

%% get landing frame number from watching the video
% if config variable already present, do nothing
if (~exist('landing_frame','var') || landing_frame < 0)
    fig = implay(VIDEO_FILE);
    landing_frame = input('What is the landing frame number? ');
    close all;
    save(ESC_FILE,'landing_frame','-append')
end
vid = VideoReader(VIDEO_FILE);

%% get landing pixhawk-time index from altitude plot
[x,y] = grn2eqa(cdata.position.lat,cdata.position.lon);

%filtered data
lpos.x = x;
lpos.y = y;
lpos.z = cdata.position.alt;
lpos.time = cdata.position.Time;

%find MSL of aircraft on runway (beware with NAN)
%lpos.z_offset = cdata.position.alt(5);

%remove MSL offset
lpos.z = cdata.position.alt;

acc = cdata.Accel.AccelZ;

roll = cdata.Gyro.GyroX;
pitch = cdata.Gyro.GyroY;
yawrate = cdata.Gyro.GyroZ;
rc_yaw = cdata.control.yaw_control_input;
rc_pitch = cdata.control.pitch_control_input;
rc_roll = cdata.control.roll_control_input;
thr = cdata.control.throttle_control_input;
airspeed = cdata.airspeed.indicated_airspeed;


if ( ~exist('landing_pix_index','var') || landing_pix_index < 0 )
    
    figure
    plot(1:length(lpos.z),lpos.z,1:length(acc),acc)
    ylabel('Acc [m/s2]')
    
    landing_pix_index = input('What is the landing time index? ');
    save(ESC_FILE,'landing_pix_index','-append')
    close all;
    
end

if ( ~exist('begin_pix_index','var') )
    
    figure
    plot(1:length(lpos.z),lpos.z,1:length(acc),acc)
    ylabel('Alt [m]')
    
    figure
    plot(cdata.Gyro.Time)
    ylabel('Time [s]')
    
    begin_pix_index = input('What is the beggining of analysis time index? ');
    save(ESC_FILE,'begin_pix_index','-append')
    end_pix_index = input('What is the end of analysis time index? ');
    save(ESC_FILE,'end_pix_index','-append')
    close all;

end

%% choose a point of reference in the pixhawk throttle data

if ( ~exist('throttle_pix_index','var') || throttle_pix_index < 0 )

    figure;
    
    subplot(2,1,1);
    plot(1:length(thr),thr);
    ylabel('Throttle');
    title('Pixhawk');
     
    subplot(2,1,2);
    plot(1:length(ESC_throttle.Throttle),ESC_throttle.Throttle);
    ylabel('Throttle');
    title('ESC');
    
    throttle_pix_index = input('What is a notable throttle time index (Pixhawk)? ');
    save(ESC_FILE,'throttle_pix_index','-append')
    throttle_esc_index = input('What is a notable throttle time index (ESC)? ');
    save(ESC_FILE,'throttle_esc_index','-append')
    close all;

end

%% create time vector of time instants of video frames
time = cdata.Gyro.Time;
FRAMERATE = vid.FrameRate;
dt = 1/FRAMERATE; % time period between frames
tland = time(landing_pix_index):-dt:time(begin_pix_index);
t = time(end_pix_index):-dt:time(begin_pix_index);
t = flip(t)';

%% discover which video-time corresponds to throttle notable instant
% init time for ESC
t_esc = zeros(length(ESC_throttle.Throttle),1);
% set notable sync time to correct value
t_esc(throttle_esc_index) = seconds(time(throttle_pix_index));
% construct time previously to notable instant
for i=throttle_esc_index-1:-1:1
    t_esc(i) = t_esc(i+1) - ESC_DT;
end
% construct time after notable instant
for i=throttle_esc_index+1:length(t_esc)
    t_esc(i) = t_esc(i-1) + ESC_DT;
end

%% remove repetitive time samples
for i=1:length(time)-1 
    % time vector might be reduced and we should terminate loop before
    % expected!
    if (i==length(time))
        break;
    end
    if (time(i)==time(i+1))
        time(i+1) = [];
        lpos.z(i+1) = [];
        roll(i+1) = [];
        pitch(i+1) = [];
        yawrate(i+1) = [];
        thr(i+1) = [];
        rc_yaw(i+1) = [];
        rc_pitch(i+1) = [];
        rc_roll(i+1) = [];
        airspeed(i+1) = [];
    end
end

%% remove NaN time instants
for i=length(time):-1:1
    if (isnan(time(i)))
        time(i) = [];
        lpos.z(i) = [];
        roll(i) = [];
        pitch(i) = [];
        yawrate(i) = [];
        thr(i) = [];
        rc_yaw(i) = [];
        rc_pitch(i) = [];
        rc_roll(i) = [];
        airspeed(i) = [];
    end
end

%% replace NaNs for zeros
% ESC_throttle(isnan(ESC_throttle.Throttle)) = 0;
% ESC_RPM(isnan(ESC_RPM.Speed)) = 0;

%% resample altitude values 
altf = interp1(time,lpos.z,t);
rollf = interp1(time,roll,t);
pitchf = interp1(time,pitch,t);
yawratef = interp1(time,yawrate,t);
thrf = interp1(time,thr,t);
ESC_throttlef = interp1(t_esc,ESC_throttle.Throttle,t);
ESC_RPMf = interp1(t_esc,ESC_RPM.Speed,t);
rc_yawf = interp1(time,rc_yaw,t);
rc_pitchf = interp1(time,rc_pitch,t);
rc_rollf = interp1(time,rc_roll,t);
airspeedf = interp1(time,airspeed,t);

%% reset video to initial frame of interest
vid.CurrentTime = (landing_frame-length(tland))/vid.FrameRate;

%% plot video with data

fig_anim = figure('Color','white','Position',[30 , 260 , 1500, 730]);
% fig_anim = figure('Color','white');
Fsize = 18;

%Embedded video
subplot(2,2,2);
frame_current = readFrame(vid);
plot2 = imshow(frame_current,'Border','tight');

%Euler angles
r2d = 180/pi;
if (VIDEO_TEMPLATE_TYPE == 1)
    subplot(2,2,1);plot1 = animatedline('Color','b');xlim([t(1) t(length(t))]);ylim([min(airspeedf) max(airspeedf)])
    ylabel('Airspeed [m/s]','FontSize',Fsize,'Fontname','Source Sans Pro')
    set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
    subplot(2,2,3);plot3 = animatedline('Color','b');xlim([t(1) t(length(t))]);ylim([min(rc_rollf) max(rc_rollf)])
    ylabel('RC Roll','FontSize',Fsize,'Fontname','Source Sans Pro')
    set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
    subplot(2,2,4);plot4 = animatedline('Color','b');xlim([t(1) t(length(t))]);ylim([min(rc_yawf) max(rc_yawf)])
    ylabel('RC Yaw','FontSize',Fsize,'Fontname','Source Sans Pro')
    set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
end
if (VIDEO_TEMPLATE_TYPE == 2)
    subplot(2,2,1);plot1 = animatedline('Color','b');xlim([t(1) t(length(t))]);ylim([min(altf) max(altf)])
    ylabel('Altitude [m]','FontSize',Fsize,'Fontname','Source Sans Pro')
    set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
    subplot(2,2,3);plot3 = animatedline('Color','b');xlim([t(1) t(length(t))]);ylim([min(thrf) max(thrf)])
    ylabel('Throttle/RPM','FontSize',Fsize,'Fontname','Source Sans Pro')
    set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
    subplot(2,2,3);plot3a = animatedline('Color','r');xlim([t(1) t(length(t))]);ylim([0 1])
    ylabel('Throttle/RPM','FontSize',Fsize,'Fontname','Source Sans Pro')
    set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
    subplot(2,2,4);plot4 = animatedline('Color','b');xlim([t(1) t(length(t))]);ylim([min(rc_pitchf) max(rc_pitchf)])
    ylabel('RC Pitch','FontSize',Fsize,'Fontname','Source Sans Pro')
    set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
end

%Altitude and Speed
% subplot(2,2,1);
% yyaxis left; plot6 = animatedline('Color','b');xlim([t(1) t(length(t))]);ylim([min(altf) max(altf)])
% ax = gca;
% ax.FontSize = Fsize;
% ax.FontName = 'Source Sans Pro';
% ax.YColor = [0 0 1];
% % set(gca,'FontSize',Fsize,'Fontname','Source Sans Pro')
% ylabel('Altitude [m]','FontSize',Fsize,'Fontname','Source Sans Pro')
% xlabel('Time [s]','FontSize',Fsize,'Fontname','Source Sans Pro')

for n = 1:length(t)
    r2d = 180/pi;
    if (VIDEO_TEMPLATE_TYPE == 1)
        addpoints(plot1,t(n),airspeedf(n))
        addpoints(plot3,t(n),rc_rollf(n))
        addpoints(plot4,t(n),rc_yawf(n))
    end
    if (VIDEO_TEMPLATE_TYPE == 2)
        addpoints(plot1,t(n),altf(n))
        addpoints(plot3,t(n),thrf(n))
        addpoints(plot3a,t(n),ESC_RPMf(n)/max(ESC_RPMf))
        addpoints(plot4,t(n),rc_pitchf(n))
    end
    % Ground video
    if (hasFrame(vid))
        frame_current = readFrame(vid);
        plot2.CData = frame_current;
    end
    drawnow
%     pause(0.01)
    M(n)=getframe(fig_anim);
end

video = VideoWriter(OUTPUT_VIDEO_FILE,'MPEG-4');
open(video)
writeVideo(video,M)
close(video)


