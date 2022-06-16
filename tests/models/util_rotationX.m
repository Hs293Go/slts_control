function C21 = util_rotationX(theta)
c = cos(theta);
s= sin(theta);
C21 = [1 0 0;0 c s;0 -s c];