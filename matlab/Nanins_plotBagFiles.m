% SP Jonas

% open bag file
disp('- open bag file')
filePath = fullfile('../bagFiles', '2015-06-02-16-39-40.bag'); % test 1m
%filePath = fullfile('../bagFiles', '2015-06-02-16-47-34.bag'); % test 2m
%filePath = fullfile('../bagFiles', '2015-05-29-19-01-43.bag'); % 
bag = rosbag(filePath);

%% Get Pitch Data
disp('- read/prepare data')
disp('pitch data')
msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/pitchLog'));
n = length(msgs);
m = length(msgs{1}.Data);

pitchLogData = zeros(n,m);
for i = 1:n
    pitchLogData(i,:) = msgs{i}.Data;
end

% PITCH data
% extract time
t = pitchLogData(:,end);
t0 = t(1);
t = t-t0;
startT = 1;
endT = find(t>80, 1);
t_p = t(startT:endT);
t_p = t_p -t_p(1);
dataPitch = pitchLogData(startT:endT,1:m-1)*180/pi;

%% Get Depth Data
disp('depth data')
msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/depthLog'));
n = length(msgs);
m = length(msgs{1}.Data);

depthLogData = zeros(n,m);
for i = 1:n
    depthLogData(i,:) = msgs{i}.Data;
end

t = depthLogData(:,end);
t0 = t(1);
t = t-t0;
startT = 1;
endT = find(t>200, 1);
t_d = t(startT:endT);
t_d = t_d -t_d(1);
dataDepth = depthLogData(startT:endT,1:m-1);

%% Get Ctrl Inputs
disp('ctrl input data')
msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/ctrlInputs'));
n = length(msgs);
m = length(msgs{1}.Data);

ctrlInputData = zeros(n,m);
for i = 1:n
    ctrlInputData(i,:) = msgs{i}.Data;
end

% CTRL INPUT data
t = ctrlInputData(:,end);
t0 = t(1);
t = t-t0;
startT = 1;
endT = find(t>200, 1);
t_c = t(startT:endT);
t_c = t_c -t_c(1);
dataCtrlInput = ctrlInputData(startT:endT,1:m-1);

%% Get Tank Position Data
disp('tank position data')
% front
msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/TankPositionFront'));
n = length(msgs);
m = length(msgs{1}.Data);

positionDataFront = zeros(n,m);
for i = 1:n
    positionDataFront(i,:) = msgs{i}.Data;
end

% rear
msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/TankPositionRear'));
n = length(msgs);
m = length(msgs{1}.Data);

positionDataRear = zeros(n,m);
for i = 1:n
    positionDataRear(i,:) = msgs{i}.Data;
end

t = positionDataRear(:,end);
t0 = t(1);
t = t-t0;
startT = 1;
endT = find(t>200, 1);
t_t = t(startT:endT);
t_t = t_t -t_t(1);
dataTankPosition = [positionDataFront(startT:endT,1:m-1), positionDataRear(startT:endT,1:m-1)];

%% plot data

disp('- plot data')

figure(1)
subplot(4,1,1)
plot(t_d, dataDepth, 'LineWidth', 2)
ylim([-0.5 max(max(dataDepth)+1)])
xlim([0, t_d(end)])
%xlabel('time [s]')
ylabel('[m]')
axis ij
title('depth z')


subplot(4,1,2)
plot(t_p, dataPitch, 'LineWidth', 2)
ylim([-10 10])
xlim([0, t_p(end)])
%xlabel('time [s]')
ylabel('[Deg]')
title('pitch angle \theta')

subplot(4,1,3)
plot(t_c, dataCtrlInput, 'LineWidth', 2)
ylim([0 1])
xlim([0, t_c(end)])
%xlabel('time [s]')
title('controller inputs')

subplot(4,1,4)
plot(t_t, dataTankPosition, 'LineWidth', 2)
ylim([0 1])
xlim([0, t_t(end)])
xlabel('time [s]')
title('fill height piston tanks')

figure(2)
plot(t_c, dataCtrlInput)
ylim([0 1])
xlim([0, t_c(end)])
%xlabel('time [s]')
title('controller inputs')


