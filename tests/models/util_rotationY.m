function C21 = util_rotationY(theta)
c = cos(theta);
s= sin(theta);
C21 = [c 0 -s;0 1 0;s 0 c];