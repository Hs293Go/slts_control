%% initialization file
clear all;
%%%%% define constants 
g= 9.8;
%%%%% initial condition

%%%%%%
v_q= [4 -1 0]';
X_initial =[0 0 0]';
y_inital = [0.02 -0.02]';
y_dot_initial = [0.1 0]';
U_initial = [y_dot_initial;v_q];
%%%%% parameters
m_q = 1.63;
m_p = 0.5;

L=1;  % string length
K_q=100; % quaternion normalization gain

g_I = [0 0 9.8]';
% way point information
n_1 = [0 1 0]';
v_d = 10;
P_1 = [5 0 -6]';
k_v = 7;
k_e = 7;
c_x = 0.5;
c_v = 0.5;
%%%%%%%%%%%%% disturbance         
W_p = [-0.4 0.8 -0.3]';
W_q = [-1 2 -0.3]';

lambda_p = 0.2;
lambda_q = 0.3;
% W_p = [0 0 0]';
% W_q = [0 0 0]';
% lambda_p = 0;
% lambda_q = 0;
lambda_theta = 1;


%%%%%%%%%%%%%% Attitude tracker %%%%%%%%%%%%%%%%
K_omega = 1*eye(3);
K_R = 1*eye(3);
J = [2 0.3 -0.5;0.3 1 0.2;-0.5 0.2 3]*10^-2;
invJ_B1 = inv(J);
omega_b01 = [0.1 0.3 0]';
initial_Euler = [180/57.3 10/57.3 30/57.3];
Lambda_initalB01 = angle2quat(initial_Euler(3),initial_Euler(2),initial_Euler(1));

n1 = [0 1 0]';
n1 = n1/norm(n1);
n2 = [0.5 1 0.1]';
n2 = n2/norm(n2);
n3 = n2;
n4 = [-0.5 1 0.1]';
n4= n4/norm(n4);
n5 = [-1 0 0]';

v1 = 4;
v2 = 8;
v3 = 1;
v4 = 4;
v5 = 6;

P1 = [5 1 -3]';
P2 = P1+n1*15*v1;
P3 = P2+n2*15*v2;
P4 = P3+n3*30*v3;
P5 = P4+n4*10*v4;

waypoint_n = [n1 n2 n3 n4 n5];
waypoint_p = [P1 P2 P3 P4 P5];
waypoint_v = [v1 v2 v3 v4 v5]';
waypoint_tolerance = [2 2 2 2 2]';

b1 = 220;
b2 = 110;
k=0.6;
n = [1 k]'/norm([1 k]);
n = [n;0];
delta_W = 0*n;

n_ = [-k 1 0]';
n_ = n_/norm(n_);
%%%%%%%%%%%%%%%%%%%%%
l1 = 50;
aa = (b1-b2)/sqrt(k^2+1);
mid_point1 = l1*n+[0 (b1+b2)/2 0]'+n_*aa/2;
mid_point2 = l1*n+[0 (b1+b2)/2 0]'-n_*aa/2;
mid_point = (mid_point1+mid_point2)/2;
sin_W = -0.8;
lambda_x =1/20;

B0 = [eye(2);-y_inital'/sqrt(L^2-y_inital'*y_inital)];
W_0 = (m_p+m_q)*v_q + m_p*B0*y_dot_initial;