% SemesterProject Nanins

% --- Control Simulation of Nanins ---

clear all
clc
close all

%% simulate Simulink model
modelPID = 'Nanins_PID_Controller';
load_system(modelPID)

% set parameters
depth = 2;
set_param('Nanins_PID_Controller/refDepth', 'Value', num2str(depth));

% PID tuning Depth controller Ziegler-Nicholson
Ku = 0.35;
Tu = 106-69;
Kp = 0.33*Ku;
Ki = 2*Kp/Tu
Kd = Kp*Tu/3;

set_param('Nanins_PID_Controller/PID Dive', 'P', num2str(Kp), 'I', num2str(Ki), 'D', num2str(Kd));

% PID tuning pitch controller Ziegler-Nicholson
Kp = 0.2;
Ki = 0;
Kd = 0;

set_param('Nanins_PID_Controller/PID Pitch', 'P', num2str(Kp), 'I', num2str(Ki), 'D', num2str(Kd));

% start simulation
sim(modelPID)


%% plot output
figure(1)
n=4; m=1;
% plot depth z
subplot(n,m,1)
plot(tout, sim_z)
hold on
plot([tout(1) tout(end)], [depth depth])

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

% % plot depth z
% figure(2)
% plot(tout, sim_theta)
% hold on
% plot([tout(1) tout(end)], [depth depth])

