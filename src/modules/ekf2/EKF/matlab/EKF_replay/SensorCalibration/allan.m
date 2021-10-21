function [T,sigma] = allan(omega,fs,pts)
[N,M]  = size(omega);                 % figure out how big the output data set is
n      = 2.^(0:floor(log2(N/2)))';    % determine largest bin size
maxN = n(end);
endLogInc = log10(maxN);
m = unique(ceil(logspace(0,endLogInc,pts)))';  % create log spaced vector average factor
t0     = 1/fs;
T      = m*t0;
theta  = cumsum(omega)/fs;
sigma2 = zeros(length(T),M);
for i=1:length(m)
                                               % t0 = sample interval
% T = length of time for each cluster
% integration of samples over time to obtain output angle ?
                                % array of dimensions (cluster periods) X (#variables)
                                % loop over the various cluster sizes
                              % implements the summation in the AV equation
sigma2 = sigma2./repmat((2*T.^2.*(N-2*m)),1,M);
sigma  = sqrt(sigma2)