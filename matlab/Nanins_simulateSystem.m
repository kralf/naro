% SEMESTER PROJECT NANINS
% Jonas Eichenberger

% --- CONTROL SIMULATION OF NANINS ---

%clear all
clc
close all

% load parameters
Nanins_parameters;

%% SET REFERENCE INPUTS
rDepth = 2;  % [m]

rPitch = 0/180*pi;  % [rad]
rW_dot = 0;  % [m/s^2]
rQ_dot = 0;  % [rad/s]

motorSpeed = 0.7;

% integrator initial conditions
iDepth = 0;
iTheta = 10/180*pi;
iTank1 = 0; %u0*h_max/10;
iTank2 = iTank1;

%% PID MODEL

modelPID = 'Nanins_Controller_PID';

% -- PID CONTROLLER
% PID tuning Depth controller Ziegler-Nicholson
Ku = 0.35;
Tu = 106-69;
Kp_d = 0.33*Ku;
Ki_d = 2*Kp_d/Tu;
Kd_d = Kp_d*Tu/3;

% PID tuning pitch controller Ziegler-Nicholson
Kp_p = 0.3;
Ki_p = 0;
Kd_p = 0;

% parameters from dive test
%Kp_d = 0.22;
%Ki_d = 0.02;
%Kd_d = 1.54;

%Kp_p = 0.72;
%Ki_p = 0.24;
%Kd_p = 0.54;

%% LQR Model

modelLQR = 'Nanins_Controller_LQR';

% Design Matrices
Q = diag([2,5,80,50]);
R = 4*diag([10,10]);

Nanins_calcLQRgain % calculate LQR Gain

%% SIMULATE MODEL
% choose model

model = modelPID;
load_system(model)

% set parameters
set_param(strcat(model,'/motorSpeed'), 'Value', num2str(motorSpeed));

set_param(strcat(model,'/AUV Dynamics/z_dot > z'), 'InitialCondition', num2str(iDepth));
set_param(strcat(model,'/AUV Dynamics/theta_dot > theta'), 'InitialCondition', num2str(iTheta));
set_param(strcat(model,'/AUV Dynamics/piston tank dynamic front/h_dot -> h'), 'InitialCondition', num2str(iTank1));
set_param(strcat(model,'/AUV Dynamics/piston tank dynamic rear/h_dot -> h'), 'InitialCondition', num2str(iTank2));

if(strcmp(model, modelPID))
   set_param(strcat(model,'/refDepth'), 'Value', num2str(rDepth));
   % set PID parameters
   set_param(strcat(model,'/PID Dive'), 'P', num2str(Kp_d), 'I', num2str(Ki_d), 'D', num2str(Kd_d));
   set_param(strcat(model,'/PID Pitch'), 'P', num2str(Kp_p), 'I', num2str(Ki_p), 'D', num2str(Kd_p));
elseif(strcmp(model, modelLQR))
   input_lqr = [rDepth rPitch rW_dot rQ_dot];
   input_lqr = strcat('[',num2str(input_lqr),']');
   set_param(strcat(model,'/Input'), 'Value', input_lqr);  
end

% -- START SIMULATION
sim(model, [0 200])

%% PLOT OUTPUTS
gca = figure(1);

n=4; m=1;
% plot depth z
subplot(n,m,1)
plot(tout, sim_z, 'LineWidth', 2)
hold on
plot([tout(1) tout(end)], [rDepth rDepth], 'LineWidth', 2)

title('Depth z')
ylim([-0.5, max(sim_z)+1])
axis ij
ylabel('[m]')

% plot pitch theta
subplot(n,m,2)
plot(tout, sim_theta, 'LineWidth', 2)
title('pitch angle \theta')
ylabel('[Deg]')

% plot input from controller
subplot(n,m,3)
plot(tout, sim_u, 'LineWidth', 2)
ylim([0, 1])
title('controller inputs')

% plot fill height piston tank
subplot(n,m,4)
plot(tout, sim_h, 'LineWidth', 2)
ylim([0, 1])
title('fill height piston tanks')
xlabel('time [s]')

%
titleStr = strrep(model, '_', ' ');
%suptitle(['Simulation: ', titleStr]);

