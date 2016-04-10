fname = 'indoor_flight_test.px4log';

%% entire log
wholeLog = importPX4log(fname,{});

%% attitude message
attitudeData = importPX4log(fname,{'ATT'});

%% estimator messages
estimatorData = importPX4log(fname,{'EST0','EST1','EST2','EST3','EST4','EST5','EST6'});

%% to save
save ift estimatorData;

%% to run viewer
estimatorLogViewer('ift.mat');