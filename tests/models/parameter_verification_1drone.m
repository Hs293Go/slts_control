% parameter verification
% system parameters
g= 9.8;
g_I = [0 0 g]';

m_p = 0.5;
m1 = 1.63;

L = 0.98;

C_r = tan((pi/6+pi/10)/2);
gamma_j = sqrt(1+C_r ^2);
delta_r = 1/sqrt(0.75);

%% control gains
% kL = 0.35;% kL = 0.35
% K = 5;
% kF1 = 1;
% kF2 = 2;
% Lambda1 = 0.3;
% Lambda2 = 0.4;
% Kr1 = 0.4;
% Kr2 = 0.5;
% kp = 0.10;%0.14
% kv = 0.2;%0.15
% k_r = 0.10;% k_r = 0.1
% k_omega = 0.10;

kL = 0.15;% kL = 0.35
K = 5;
kF = 2;

Lambda = 0.4;
Kr = 0.2;
kp = 0.10;%0.14
kv = 0.24;%0.15

X = Lambda + Kr *kF;

fd = m_p*g + 1;% assume the disturbance is small

V2c = fd- L*X^2*m_p /kF;

H_p  = m_p *[kv    0       -(kv+kp)/2 -kp/2;
        0     kp*kv^2 -kv*kp/2   -kp*kv/2;
      -(kv+kp)/2 -kv*kp/2 Kr1 0;
       -kp/2   -kp*kv/2   0  Lambda*kF];
eig(H_p)


H_rP = [0;0;-m_p*gamma_j*X*Kr/2;-m_p*X1*(Lambda*gamma_j+delta_r*kL)/2];

G_rj = (cos(pi/10)-sin(pi/10)*C_r)*m_p*g/L;

H = [H_p H_rP;
     H_rP' kL*G_rj];

eig(H)
