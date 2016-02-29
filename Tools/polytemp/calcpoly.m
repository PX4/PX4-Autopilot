function [ptemp, pdata, data_tacomp, data_tcomp] = calcpoly(data, npdata, temp, nptemp, time, truetemp)
%Authors:
% - Hessel van der Molen
%
% IN
%  data     = raw sensor data, NxM
%  npdata   = number of polynomials for attitude fit
%  temp     = temperature data, Nx1
%  nptemp   = number of polynomials for temperature fit
%  time     = timestamps of sensor and temperature data, Nx1
%  truetemp = temperature corresponding to 'correct' sensor value
% OUT
%  ptemp    = calculated polynomials for temperature fit, (nptemp+1)xM
%  pdata    = calculated polynomials for temperature compensated data fit, (npdata)xM
%  data_tacomp = temperature and attitude compensated data
%  data_tcomp  = temperature compensated data

%data size
dsize = size(data, 1);
ddim  = size(data, 2);

%difference between timestamps
dtimes = diff(time);

%polynomials result array;
ptemp = zeros(nptemp+1, ddim); % add 1 for constant
pdata = zeros(npdata+0, ddim); % contant is removed by differentiating

%resulting array for temperature compensation
data_tcomp = zeros(dsize, ddim);

%resulting array for attitude of temperature compensated data
attitude_tcomp = [0 0 0];

%resulting array for temperature and attitude compensation
data_tacomp = zeros(dsize, ddim);

%resulting array for attitude of temperature anf attitude compensated data
attitude_tacomp = [0 0 0];

%%%%%
%%%%% TEMPERATURE COMPENSATION
%%%%%

%resulting array for temperature correction
dtemp = zeros(dsize, ddim);
for i=1:ddim
    %calculate polynomials of correlation between temperature and data
    p = polyfit(temp, data(:,i), nptemp);

    %store polynomials
    ptemp(:,i) = p';

    %calculate offset of the 'true temperature' (result of fit should be 0);
    offset = polyval(p, truetemp);

    %calculate temperature difference of fitted data and 'true temperature'
    % assumption is made that the sensor did not move, so this difference can be used for correction!
    dtemp(:,i) = polyval(p, temp)' - offset;
end

%scale data-value according to temperature difference
for i=1:dsize
    data_tcomp(i,:) = data(i,:) - dtemp(i,:);
end

%%%%%
%%%%% ATTITUDE COMPENSATION
%%%%%

%calculate temperature compensated attitude
for i=2:dsize
    attitude_tcomp(i, :) = attitude_tcomp(i - 1, :) + data_tcomp(i, :) * dtimes(i-1);
end

%resulting array for attitude correction
ddata = zeros(dsize, ddim);
for i=1:ddim
    %calculate polynomials of attitude
    p = polyfit(time, attitude_tcomp(:,i), npdata);

    %differentiate polynomial
    dp = p .* [npdata:-1:0];
    %remove the 0 (last element)
    dp = dp(1:npdata);

    %store differentiated polynomials
    pdata(:,i) = dp;

    %calculate the differentiated values!
    ddata(:,i) = polyval(dp, time)';
end

%scale data-value according to temperature difference
for i=1:dsize
    data_tacomp(i,:) = data_tcomp(i,:) - ddata(i,:);
end
