% ImportPX4LogData('','13_31_02','px4log')
%Only copy the previous line on the command window and enter. The file must
%be with the python code, and python must be on the path of matlab
%Last updated August 22, 2019 by Spencer Chen
%Runtime reduced by 90% and flight duration corrected

function ImportPX4LogData(filepath,filename,format)

% ************************************************************************
% check file format, if matlab file defined, exit
% ************************************************************************
if (strcmp(format,'mat'))
    return;
end

% ************************************************************************
% RETRIEVE SYSTEM VECTOR
% ************************************************************************
% //All measurements in NED frame

% Convert to CSV
arg1 = fullfile(filepath,strcat(filename,'.px4log'));
delim = ',';
time_field = 'TIME_StartTime';
data_file = 'data.csv';

% message fields to be extracted
msg_list = {'"TIME_StartTime"','"ATT"', '"ATSP"', '"ATTC"', '"OUT0"', '"ARSP"', '"GPOS"'};
msg_field = strcat('-m ', msg_list); %append -m option
msg_field = strjoin(msg_field);      %create single string

% note that sdlog2_dump.py must be present in the same directory. This is
% copied from PX4/Tools folder
s = system( sprintf('python sdlog2_dump.py "%s" -f "%s" -d "%s" %s', arg1, data_file, delim, msg_field) );

if exist(data_file, 'file')
    
    %data = csvread(data_file); ### does not work
    %data = tdfread(data_file, ','); ### takes too long ~ 5 mins
    data = readtable(data_file); % ~ 15 seconds
    
    %find flight start time
    start_index = 0;
    index_check = 1;
    while start_index < 1
        if isnan(data.TIME_StartTime(index_check))
            index_check = index_check + 1;
        else
            start_index = index_check;
        end
        if index_check > size(data.TIME_StartTime)
            disp('Error: Start time not found');
            start_index = 1;
        end
    end
    
    % show the flight time
    time_us = data.TIME_StartTime(end) - data.TIME_StartTime(start_index);
    time_s = uint64(time_us*1e-6);
    time_m = uint64(time_s/60);
    time_s = time_s - time_m * 60;
    
    disp([sprintf('Flight log duration: %d:%d (minutes:seconds)', time_m, time_s) char(10)]);
    
    disp(['logfile conversion finished.' char(10)]);
else
    disp(['file: ' data_file ' does not exist' char(10)]);
end

% *************************************************************************
% Post Process Timevector
% *************************************************************************

%conversion factors
fconv_timestamp=1E-6; % [microseconds] to [seconds]

%Create time vector
time=double(data.TIME_StartTime) .*fconv_timestamp;

% *************************************************************************
% Save sysvector as matlab binary file
% *************************************************************************
save(fullfile(filepath,strcat(filename,'.mat')),'data','time');

end