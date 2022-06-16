% quaternion
function Lamda_dot = util_quaternionderivative(Lamda,omega)
R = util_quaternionRmatrix(Lamda);
Lamda_dot = 0.5*R'*omega;