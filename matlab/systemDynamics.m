%SemesterProject Nanins

% Non-Linear SystemDynamics

clear all;
close all;
clc;

% ---- PARAMETERS ----

g   = 9.81;                 % [m/s^2]   gravity
rho = 1000;                 % [kg/m^3]  density water

% parameters nanins specific
m0  = 5.7;                    % [kg]      empty weight Nanin
l_n = 0.514;                  % [m]       length Nanin
r_n = 0.06;                  % [m]       radius Nanin
V_n = r_n^2*pi*l_n         % [m^3]     volume Nanin

x0_t = 0.25;                % [m]       displacement piston tank in x direction
z0_t = 0.05;                % [m]       displacement piston tank in z direction
A_t = pi*(65/(2*1000))^2;   % [m^2]     area piston tank
V_t = 250e-6;               % [m^3]     volume of piston tank
h_max = V_t/A_t;            % [m]       max height of water in piston tank

Iy0 = 0.134;                % [kgm^2]   moment of inertia around y-axis

% restoring static forces
W_r = m0*g          % [N]       gravitational force empty
B_r = rho*g*V_n    % [N]       buoancy restoring force

% Added mass coefficents
Z_w = -rho*V_n;             % [kg]
M_q = -1/12*rho*V_n*l_n;    % [km*m]

% ------

% Init inputs
h1 = 1;
h2 = 1;
theta =0.5;
x = zeros(8,1);

% Denormalize input
h1 = h1*h_max;
h2 = h2*h_max;

u_sys = [h1, h2]';

% calculate dynamic parameters
m = m0 + rho*A_t*(h1+h2); % dynamic mass
x1 = x0_t - h1/2;         % CoG of piston tank
x2 = -(x0_t - h2/2);
X_u = -0.1*m;

W = m*g

z_g = 0.005;
x_g = 0;
I_y = Iy0;

% calculate system model matrices
M = [m-X_u, 0,      m*z_g;...
     0,     m-Z_w,  -m*x_g;...
     m*z_g, -m*x_g, I_y-M_q];
 
g_static = [(W_r-B_r)*sin(theta);...
            -(W_r-B_r)*cos(theta);...
            z_g*W_r*sin(theta) + x_g*W_r*cos(theta)];
        
G_dyn = g*rho*A_t*[sin(theta),  sin(theta);...
                    -cos(theta), -cos(theta);...
                    sin(theta)*z0_t+cos(theta)*x1, sin(theta)*z0_t+cos(theta)*x2];
                
% state x = [z theta u w q fu fw fq]
x(6:8) = g_static; % update state

M_inv = inv(M);

A_sys = [zeros(2,2), [-sin(theta) cos(theta) 0; 0 0 1], zeros(2,3);...
    zeros(3,5), -M_inv;...
    zeros(3,8)];

B_sys = [zeros(2,2);...
    -M_inv*G_dyn;...
    zeros(3,2)];

x_dot = A_sys*x+B_sys*u_sys;

%% Calculate LGR Gain


A = [zeros(2,2), [-sin(theta) cos(theta) 0; 0 0 1];...
    zeros(3,5)];

B = [zeros(2,2);...
    -M_inv*G_dyn]

% remove u
Au = [A(1,:); A(2,:); A(4,:); A(5,:)];
Au = [Au(:,1), Au(:,2), Au(:,4), Au(:,5)]
Bu = [B(1,:); B(2,:); B(4,:); B(5,:)]


% Design Matrices
Q = diag([2,2,0.1,0.1]);
R = diag([1,1]);

[K,S,e] = lqr(Au,Bu,Q,R)


%%
g   = 9.81;                 % [m/s^2]   gravity
rho = 1000;                 % [kg/m^3]  density water

% parameters nanins specific
m0  = 12.5;                    % [kg]      empty weight Nanin
l_n = 0.4;                  % [m]       length Nanin
r_n = 0.1;                  % [m]       radius Nanin
V_n = r_n^2*pi*l_n;         % [m^3]     volume Nanin

x0_t = 0.25;                % [m]       displacement piston tank in x direction
z0_t = 0.05;                % [m]       displacement piston tank in z direction
A_t = pi*(65/(2*1000))^2;   % [m^2]     area piston tank
V_t = 250e-6;               % [m^3]     volume of piston tank
h_max = V_t/A_t;            % [m]       max height of water in piston tank

% restoring static forces
W_r = m0*g;          % [N]       gravitational force empty
B_r = rho*g*V_n;    % [N]       buoancy restoring force

% Added mass coefficents
Z_w = -rho*V_n;             % [kg]
M_q = -1/12*rho*V_n*l_n;    % [km*m]

% ------

% rename parameters for clarification

% Denormalize input
h1 = h1*h_max;
h2 = h2*h_max;

u_sys = [h1, h2]';

% calculate dynamic parameters
m = m0 + rho*A_t*(h1+h2); % dynamic mass
x1 = x0_t - h1/2;         % CoG of piston tank
x2 = -(x0_t - h2/2);
X_u = -0.1*m;

z_g = 0.005;
x_g = 0;
I_y = 0;

% calculate system model matrices
M = [ m-Z_w,  -m*x_g;...
     -m*x_g, I_y-M_q];
 
g_static = [-(W_r-B_r)*cos(theta);...
            z_g*W_r*sin(theta) + x_g*W_r*cos(theta)];
        
G_dyn = g*rho*A_t*[-cos(theta), -cos(theta);...
                    sin(theta)*z0_t+cos(theta)*x1, sin(theta)*z0_t+cos(theta)*x2];
                
% calcuate state w_dot, q_dot
x_dot = M\(-g_static -G_dyn*u_sys)
