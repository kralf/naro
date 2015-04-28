% SemesterProject Nanins
% FS 2015
% Jonas Eichenberger
clc; close all;

% Convert data from timeseries
time = tout;
z_output = sim_z(:,1);
z_input = sim_z(:,2);

N = 800;
time = time(1:N);
z_output = z_output(1:N);
z_input = z_input(1:N);

% plot depth
data = [z_input, z_output];

figure(1)

plot(time, data)
hold on

title('Depth Simulation of Nanin')
xlabel('Time [s]')
ylabel('Depth z [m]')
legend('Input', 'Output');
ylim([-0.5, max(max(data))+1])
axis ij

p = animatedline('Marker','o','MarkerFaceColor','red');

timeFactor = 0.5;
dt = time(end)-time(end-1);
dt = dt/timeFactor;

tic
a=tic;
for k = 1:length(time)
    clearpoints(p)
    addpoints(p,time(k),z_output(k))
    b = toc(a); % check timer
    if b > dt
        drawnow % update screen
        a = tic; % reset timer after updating
    end
end
drawnow

toc
