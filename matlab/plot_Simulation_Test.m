close all

figure(4)
subplot(2,1,1)
plot([tout(1) tout(end)], [rDepth rDepth], 'LineWidth', 2)
hold on
plot(tout, sim_z, 'LineWidth', 2)
%hold on
%plot(t_d, dataDepth, 'LineWidth', 2)
axis ij
ylim([-0.5 2])
ylabel('[m]')
title('SIMULATION: depth z')

subplot(2,1,2)
plot(t_d, dataDepth, 'LineWidth', 2)
ylim([-0.5 2])
xlim([0, t_d(end)])
%xlabel('time [s]')
ylabel('[m]')
xlabel('time [s]')
title('LAKE TEST: depth z')
axis ij
