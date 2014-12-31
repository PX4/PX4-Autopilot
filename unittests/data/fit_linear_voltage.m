close all;
clear all;
M = importdata('px4io_v1.3.csv');
voltage = M.data(:, 1);
counts = M.data(:, 2);
plot(counts, voltage, 'b*-', 'LineWidth', 2, 'MarkerSize', 15);
coeffs = polyfit(counts, voltage, 1);
fittedC = linspace(min(counts), max(counts), 500);
fittedV = polyval(coeffs, fittedC);
hold on
plot(fittedC, fittedV, 'r-', 'LineWidth', 3);

slope = coeffs(1)
y_intersection = coeffs(2)
