% Semester Project Jonas
% Controller Nanins

% ---- PARAMETERS ----

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
W_r = m0*g          % [N]       gravitational force empty
B_r = rho*g*V_n    % [N]       buoancy restoring force

% Added mass coefficents
Z_w = -rho*V_n;             % [kg]
M_q = -1/12*rho*V_n*l_n;    % [km*m]

m_max = m0+1*V_t*rho
Fg_max = g*m_max