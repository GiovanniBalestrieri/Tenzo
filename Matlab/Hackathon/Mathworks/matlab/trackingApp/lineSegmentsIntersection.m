% Copyright 2016 The MathWorks, Inc.
function [isParallel, isOnSegment, x, y, ua, ub] = lineSegmentsIntersection(x1, y1, x2, y2, X3, Y3, X4, Y4)

denominator = (Y4 - Y3) * (x2 - x1) - (X4 - X3) * (y2 - y1);

isParallel = abs(denominator) <= eps;
ua = ( (X4 - X3) .* (y1 - Y3) - (Y4 - Y3) .* (x1 - X3) ) ./ denominator;
ub = ( (x2 - x1) .* (y1 - Y3) - (y2 - y1) .* (x1 - X3) ) ./ denominator;
ind13 = (X3 == x1) & (Y3 == y1);
ind14 = (X4 == x1) & (Y4 == y1);
ind23 = (X3 == x2) & (Y3 == y2);
ind24 = (X4 == x2) & (Y4 == y2);
ua(ind13 | ind14) = 0;
ua(ind23 | ind24) = 1;
ub(ind13 | ind23) = 0;
ub(ind14 | ind24) = 1;
isOnSegment = (~isParallel) & (ua <= 1) & (ua >= 0) & (ub <= 1) & (ub >= 0);

x = x1 + ua * (x2 - x1);
y = y1 + ua * (y2 - y1);

end
