% R matrix of a quaternion
% Lamda is the quaternion vector Lamda = (Lamda0 Lamda1 Lamda2 Lamda3)'
function R = util_quaternionRmatrix(Lamda)
Lamda0 = Lamda(1);
LamdaA = Lamda(2:end);
LamdaAX  = util_crossmatrix(LamdaA );
R = [-LamdaA LamdaAX+Lamda0*eye(3)];