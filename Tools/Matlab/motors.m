clear all;
close all;

% Measurement data
% 1045 propeller
% Robbe Roxxy Motor (1100 kV, data collected in 2010)
data = [ 45, 7.4;...
         38, 5.6;...
         33, 4.3;...
         26, 3.0;...
         18, 2.0;...
         10, 1.0 ];

% Normalize the data, as we're operating later
% anyways in normalized units
data(:,1) = data(:,1) ./ max(data(:,1));
data(:,2) = data(:,2) ./ max(data(:,2));

% Fit a 2nd degree polygon to the data and
% print the x2, x1, x0 coefficients
p = polyfit(data(:,2), data(:,1),2)

% Override the first coffefficient for testing
% purposes
pf = 0.62;

% Generate plotting data
px1 = linspace(0, max(data(:,2)));
py1 = polyval(p, px1);

pyt = zeros(size(data, 1), 1);
corr = zeros(size(data, 1), 1);

% Actual code test
% the two lines below are the ones needed to be ported to C:
%   pf: Power factor parameter.
%   px1(i): The current normalized motor command (-1..1)
%   corr(i): The required correction. The motor speed is:
%            px1(i) 
for i=1:size(px1, 2)
    
    % The actual output throttle
    pyt(i) = -pf * (px1(i) * px1(i)) + (1 + pf) * px1(i);
    
    % Solve for input throttle
    % y = -p * x^2 + (1+p) * x;
    % 
end

plot(data(:,2), data(:,1), '*r');
hold on;
plot(px1, py1, '*b');
hold on;
plot([0 px1(end)], [0 py1(end)], '-k');
hold on;
plot(px1, pyt, '-b');
hold on;
plot(px1, corr, '-m');
