%%%%% define constants %%%%%
g = 9.80665;
g_I = [0 0 g]';
%%%%% initial condition %%%%%
v_q = [2 -1 0]';
X_initial = [0 0 0]';
y_inital = [0.02 -0.02]';
y_dot_initial = [0.1 0]';
U_initial = [y_dot_initial; v_q];
%%%%% system parameters %%%%%%%%
uav_mass = 1.63;
pld_mass = 0.5;
sys_mass = uav_mass + pld_mass;
uav_weight = uav_mass * g_I;
pld_weight = pld_mass * g_I;
sys_weight = sys_mass * g_I;
cable_len = 0.98; % string length
cable_len_sq = cable_len.^2;
%%%%%%%%
K_q = 100; % quaternion normalization gain
%%%%%%%% % way point information %%%%%%%%
n_1 = [0 1 0]';
P_1 = [5 0 -6]';
%%%%%%%%  control parameters %%%%%%%%
k_L = 0.15;
K_0 = 5;
Lambda = 0.4;
Kr = 0.2;

k_p = 0.10;
k_v = 0.24;

c_x = 0.5;
c_v = 0.5;

Lambda_T = 0.5;
kappa = 0.9;

%%%%%%%%%%%%% disturbances %%%%%%%%
W_p = [-0.4 0.8 -0.3]';
W_q = [-1 2 -0.3]';
lambda_p = 0.2;
lambda_q = 0.3;

lambda_theta = 1;

%%%%%%%%%%%%%% Attitude tracker parameters %%%%%%%%%%%%%%%%
K_omega = 1 * eye(3);
K_R = 1 * eye(3);
J = [2 0.3 -0.5; 0.3 1 0.2; -0.5 0.2 3] * 10^ - 2;
invJ_B1 = inv(J);
omega_b01 = [0.1 0.3 0]';
initial_Euler = [180/57.3 10/57.3 30/57.3];
Lambda_initalB01 = angle2quat(initial_Euler(3), initial_Euler(2), initial_Euler(1));
