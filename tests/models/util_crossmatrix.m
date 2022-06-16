% cross product matrix p = [p1 p2 p3]';
function px = util_crossmatrix(p)
px = [0 -p(3) p(2);p(3) 0 -p(1);-p(2) p(1) 0];