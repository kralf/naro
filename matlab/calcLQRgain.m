% Semester Project Nanins
% Jonas

clc;
close all;
clear all;

%% --- Calculate LQR Gain ---

parameters;

% use reduced system without constant term

% rename parameters for clarification
h1 = 0.2264;
h2 = 0.2264;
theta = 0;

% Denormalize input
h1 = h1*h_max;
h2 = h2*h_max;

u_sys = [h1, h2]';

% calculate dynamic parameter;
m = m0 + rho*A_t*(h1+h2); % dynamic mass
x1 = x0_t - h1/2;         % CoG of piston tank
x2 = -(x0_t - h2/2);

z_g = 0.005;
x_g = 0;
I_y = 0;

% calculate system model matrices
M = [ m-Z_w,  -m*x_g;...
     -m*x_g, I_y-M_q];
        
G_dyn = g*rho*A_t*[-cos(theta), -cos(theta);...
                    sin(theta)*z0_t+cos(theta)*x1, sin(theta)*z0_t+cos(theta)*x2];

% state space model x_dot = Ax+Bu x = [z theta w_dot q_dot]
A = [zeros(2,2), [cos(theta) 0; 0 1];...
    zeros(2,4)];

B = [zeros(2,2);...
    -inv(M)*G_dyn];

C = eye(4,4);

D = zeros(4,2);

% Design Matrices
Q = 0.1*diag([10,1,1,1]);
R = 10*diag([1,1]);

[K,S,e] = lqr(A,B,Q,R)