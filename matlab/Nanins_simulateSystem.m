% SEMESTER PROJECT NANINS
% Jonas Eichenberger

% --- CONTROL SIMULATION OF NANINS ---

clear all
clc
close all

% load parameters
Nanins_parameters;

%% SET REFERENCE INPUTS
rDepth = 1;  % [m]

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

modelPID = 'Nanins_PID_Controller';

% -- PID CONTROLLER
% PID tuning Depth controller Ziegler-Nicholson
Ku = 0.35;
Tu = 106-69;
Kp = 0.33*Ku;
Ki = 2*Kp/Tu;
Kd = Kp*Tu/3;

% PID tuning pitch controller Ziegler-Nicholson
Kp = 0.2;
Ki = 0;
Kd = 0;

%% LQR Model

modelLQR = 'Nanins_LQR_Controller';

% Design Matrices
Q = diag([2,5,0.1,0.1]);
R = 4*diag([10,10]);

Nanins_calcLQRgain % calculate LQR Gain

%% SIMULATE MODEL
% choose model

model = modelLQR;
load_system(model)

% set parameters
set_param(strcat(model,'/motorSpeed'), 'Value', num2str(motorSpeed));

set_param(strcat(model,'/AUV dynamic/z_dot > z'), 'InitialCondition', num2str(iDepth));
set_param(strcat(model,'/AUV dynamic/theta_dot > theta'), 'InitialCondition', num2str(iTheta));
set_param(strcat(model,'/piston tank dynamic front/h_dot -> h'), 'InitialCondition', num2str(iTank1));
set_param(strcat(model,'/piston tank dynamic rear/h_dot -> h'), 'InitialCondition', num2str(iTank2));

if(strcmp(model, modelPID))
   set_param(strcat(model,'/refDepth'), 'Value', num2str(rDepth));
   % set PID parameters
   set_param(strcat(model,'/PID Dive'), 'P', num2str(Kp), 'I', num2str(Ki), 'D', num2str(Kd));
   set_param(strcat(model,'/PID Pitch'), 'P', num2str(Kp), 'I', num2str(Ki), 'D', num2str(Kd));
elseif(strcmp(model, modelLQR))
   input_lqr = [rDepth rPitch rW_dot rQ_dot];
   input_lqr = strcat('[',num2str(input_lqr),']');
   set_param(strcat(model,'/Input'), 'Value', input_lqr);  
end

% -- START SIMULATION
sim(model)

%% PLOT OUTPUTS
figure(1)
n=4; m=1;
% plot depth z
subplot(n,m,1)
plot(tout, sim_z)
hold on
plot([tout(1) tout(end)], [rDepth rDepth])

title('Depth z')
ylim([-0.5, max(sim_z)+1])
axis ij

% plot pitch theta
subplot(n,m,2)
plot(tout, sim_theta)
title('pitch angle theta')

% plot input from controller
subplot(n,m,3)
plot(tout, sim_u)
title('controller input')

% plot fill height piston tank
subplot(n,m,4)
plot(tout, sim_h)
title('fill height pisto tanks')

%
titleStr = strrep(model, '_', ' ');
suptitle(['Simulation: ', titleStr]);

