% Copyright 2016 The MathWorks, Inc.
function in = myInpolygon(xq, yq, xv, yv)

alpha = 0.1;
xout = (1+alpha)*min(xv)-alpha*max(xv);
yout = (1+alpha)*min(yv)-alpha*max(yv);

in = true(length(xq), 1);

for k=1:length(in)
    
    [isParallel, ~, ~, ~, ua, ub] =...
        lineSegmentsIntersection(xout, yout, xq(k), yq(k), xv, yv,...
        circshift(xv,1), circshift(yv,1));
    
    crossings = ~isParallel & ub <=1 & ub >= 0 & ua <= 1;
    in(k) = mod(nnz(crossings), 2) == 1;
    
end





