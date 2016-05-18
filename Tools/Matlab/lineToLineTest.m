% This script can be used to create test cases for the line-to-line
% distance function in the flight path tracker. We can define deltas and
% start points for two lines and get the shortest delta between the two
% lines, as well as a visualisation of it. Note that all points are
% discretized, so they will usually not lie exactly on the lines.

function linesToLineTest()
    % plot different scenarios
    delta = plotLines([4; 2; -1], [2; 1; -2], [6; 2; 2], [2; 3; 0])
    delta = plotLines([4; 2; -1], [2; 1; -2], [6; 2; 2], [5; 4; 4])
    delta = plotLines([4; 2; -1], [2; 2; -2], [4; 1; 1], [5; 4; 4])
    delta = plotLines([4; 2; -1], [2; 1; -2], [3; 1; 1], [-1; 2; -2])
    delta = plotLines([4; 2; -1], [2; 1; -2], [-2; 3; 2], [-1; 2; -2])
    delta = plotLines([4; 2; -1], [2; 1; -2], [-2; 3; 2], [-3; 5; 0])
    delta = plotLines([4; 2; -1], [6; 3; -3], [-2; 3; 2], [-3; 5; 0])
    
    delta = plotLines([4; 2; -1], [2; 1; -2], [-2; 3; 2], [-1; 2; -2])
end

% This creates a plot showing the two lines and 5 connections between them:
% - the shortest possible connection
% - the start of line1 projected to line2
% - the end of line1 projected to line2
% - the start of line2 projected to line1
% - the end of line2 projected to line1
% The projected points are constrained such that they don't lie beyond
% the start or end of the other line.
function delta = plotLines(deltaA, endA, deltaB, endB)
    COEF1 = 32768;
    
    startA = endA - coefToFloat(COEF1-1) * deltaA;
    startB = endB - coefToFloat(COEF1-1) * deltaB;

    projectedStartACoef = coefFromFloat(-dot(startA-endB,deltaB) / dot(deltaB,deltaB));
    projectedEndACoef = coefFromFloat(-dot(endA-endB,deltaB) / dot(deltaB,deltaB));
    projectedStartBCoef = coefFromFloat(-dot(startB-endA,deltaA) / dot(deltaA,deltaA));
    projectedEndBCoef = coefFromFloat(-dot(endB-endA,deltaA) / dot(deltaA,deltaA));

    normal = [deltaA(2)*deltaB(3)-deltaA(3)*deltaB(2);deltaA(3)*deltaB(1)-deltaA(1)*deltaB(3);deltaA(1)*deltaB(2)-deltaA(2)*deltaB(1)];
    projectedDelta = normal*dot(endB-endA,normal)/dot(normal,normal);
    
    % Rounding is probably appropriate here, as the precision is not
    % critical and it allows for a more efficient implementation.
    % In a few cases the final result will differ, but visual inspection
    % indicates that this is acceptable.
    projectedDelta = round(projectedDelta);
    
    remainder = endB-endA - projectedDelta;

    A=[-deltaA deltaB];
    lineToLineCoef = (A'*A) \ (A'*remainder);
    lineToLineCoef(1) = coefFromFloatUnconstrained(lineToLineCoef(1));
    lineToLineCoef(2) = coefFromFloatUnconstrained(lineToLineCoef(2));

    
    close all;
    hold on;
    axis equal;
    plot3([endA(1) startA(1)], [endA(2) startA(2)], [endA(3) startA(3)], 'o-r');
    plot3([endB(1) startB(1)], [endB(2) startB(2)], [endB(3) startB(3)], 'x-b');
    delta = plotLine(deltaA, endA, deltaB, endB, lineToLineCoef(1), lineToLineCoef(2))';
    delta1 = plotLine(deltaA, endA, deltaB, endB, COEF1, projectedStartACoef)';
    delta2 = plotLine(deltaA, endA, deltaB, endB, 0, projectedEndACoef)';
    delta3 = plotLine(deltaA, endA, deltaB, endB, projectedStartBCoef, COEF1)';
    delta4 = plotLine(deltaA, endA, deltaB, endB, projectedEndBCoef, 0)';
    legend('line A', 'line B', 'line-to-line', 'start1 to line2', 'end1 to line2', 'start2 to line1', 'end2 to line1');
    hold off;
    
    %disp(lineToLineCoef);
    if (lineToLineCoef(1) >= 0 && lineToLineCoef(1) < COEF1 && lineToLineCoef(2) >= 0 && lineToLineCoef(2) < COEF1)
        return
    end
    
    delta = delta1;
    if (norm(delta2) < norm(delta))
        delta = delta2;
    end
    if (norm(delta3) < norm(delta))
        delta = delta3;
    end
    if (norm(delta4) < norm(delta))
        delta = delta4;
    end

end

function coef = coefFromFloatUnconstrained(f)
    coef = round(f * 32768);
end

function coef = coefFromFloat(f)
    coef = coefFromFloatUnconstrained(f);
    if coef < 0
        coef = 0;
    elseif coef > 32767
        coef = 32767;
    end
end

function f = coefToFloat(coef)
    f = coef / 32768;
end

function delta = plotLine(deltaA, endA, deltaB, endB, coefA, coefB)
    p1 = round(endA - deltaA * coefToFloat(coefA));
    p2 = round(endB - deltaB * coefToFloat(coefB));
    delta = p2 - p1;
    %disp([sqrt(dot(delta, delta)) (p2 - p1)']); % for debugging
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], '*--');
end
