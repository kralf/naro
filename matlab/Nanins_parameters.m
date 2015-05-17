% Semester Project Jonas
% Controller Nanins

% ---- PARAMETERS --

g   = 9.81;                     % [m/s^2]   gravity
rho = 1000;                     % [kg/m^3]  density water

% parameters nanins specific
m0  = 5.7;                      % [kg]      empty weight Nanin
l_n = 0.514;                    % [m]       length Nanin
r_n = 0.06;                     % [m]       radius Nanin
V_n = r_n^2*pi*l_n;             % [m^3]     volume Nanin

x0_t = 0.25;                    % [m]       displacement piston tank in x direction
z0_t = 0.05;                    % [m]       displacement piston tank in z direction
A_t = pi*(65/(2*1000))^2;       % [m^2]     area piston tank
V_t = 250e-6;                   % [m^3]     volume of piston tank
h_max = V_t/A_t;                % [m]       max height of water in piston tank
A_inlet = (pi*(10/1000)^2);     % [m^2]     Area water inlet of piston tank
t_in = 10;                      % [s]       min time to fill tank
denorm_t = A_inlet*h_max/t_in;  %           denormalize const. of tank for simulation       

Iy0 = 0.134;                    % [kgm^2]   moment of inertia around y-axis

% restoring static forces
W_r = m0*g;                     % [N]       gravitational force empty
B_r = rho*g*V_n;                % [N]       buoancy restoring force

% Added mass coefficents
Z_w = -rho*V_n;                 % [kg]
M_q = -1/12*rho*V_n*l_n;        % [km*m]

% ------

m_max = m0+2*V_t*rho;
Fg_max = g*m_max;

u0 = ((B_r-W_r)/(2*rho*A_t*g))/h_max;

F = W_r + u0*h_max*A_t*rho*g*2;