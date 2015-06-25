clear all
close all
clc

filePath = fullfile('../bagFiles', '2015-05-29-18-16-40_pitch.bag'); % 
bag = rosbag(filePath);

disp('pitch data')
msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/pitchLog'));
n = length(msgs);
m = length(msgs{1}.Data);

pitchLogData = zeros(n,m);
for i = 1:n
    pitchLogData(i,:) = msgs{i}.Data;
end

%%
% PITCH data
% extract time
t = pitchLogData(:,end);
t0 = t(1);
t = t-t0;
startT = find(t>105, 1);
t_p = t(startT:end);
t_p = t_p -t_p(1);
dataPitch = pitchLogData(startT:end,1:m-1)*180/pi;
%%
figure(1)
plot(t_p, dataPitch, 'LineWidth', 2)
%ylim([-60 60])
xlim([0, t_p(end)])
xlabel('time [s]')
ylabel('[Deg]')
title('pitch angle \theta')