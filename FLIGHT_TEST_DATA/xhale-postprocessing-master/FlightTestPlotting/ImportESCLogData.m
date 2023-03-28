%% import raw data
LOG_FILE = '../RRVflights/2019_08_10/flight1.csv';
OUTPUT_FILE = '../RRVflights/2019_08_10/flight1_ESC.mat';
A = xlsread(LOG_FILE);
[n,s,r] = xlsread(LOG_FILE);

%% look for number of magnetic poles in CSV file and sampling time
magnets_ready = true;
sampling_ready = false;
for line=1:length(s)
    str = s{line,1};
    % look for line starting with '# Poles'
    if (length(str)<7) % too little a string to be the right one
        continue; 
    end
    if (strcmp(str(1:7),'# Poles')==true)
        CSV_MAGNETS = textscan(str,'# Poles - %f');
        CSV_MAGNETS = CSV_MAGNETS{1}; % correct for cell usage
        magnets_ready = true;
    end        
    if (length(str)<18) % too little a string to be the right one
        continue; 
    end
    % look for line starting with '# Start Of Session'
    if (strcmp(str(1:18),'# Start Of Session')==true)
        SAMPLING_TIME = textscan(str,'# Start Of Session - Sample Time: %f');
        SAMPLING_TIME = SAMPLING_TIME{1}; % correct for cell usage
        sampling_ready = true;
    end        
end

%% speed to RPM factor computation
% get number of magnets from CSV
REAL_MAGNETS = 14;
SPEED_FACTOR = REAL_MAGNETS/CSV_MAGNETS*360/60;

%% decode data channels
t = 0:length(A)-1; 
t = SAMPLING_TIME*t;
ESC_throttle = A(:,1);
ESC_powerout = A(:,2);
ESC_voltage  = A(:,3);
ESC_ripple   = A(:,4);
ESC_current  = A(:,5);
ESC_temperat = A(:,6);
ESC_RPM      = A(:,7)/SPEED_FACTOR; % in RPM
ESC_DT       = SAMPLING_TIME;

%% export data to .mat file!
save(OUTPUT_FILE,'ESC_throttle','ESC_powerout','ESC_voltage','ESC_ripple','ESC_current','ESC_temperat','ESC_RPM','ESC_DT');

