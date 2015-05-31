% SP Jonas

filePath = fullfile('../bagFiles', '2015-05-29-18-16-40_pitch.bag');
bag = rosbag(filePath);

% Get Pitch Data
msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/pitchLog'));
n = length(msgs);
m = length(msgs{1}.Data);

pitchLogData = zeros(n,m);
for i = 1:n
   pitchLogData(i,:) = msgs{i}.Data; 
end

t = pitchLogData(:,end);
figure(1)
plot(t, pitchLogData(:,1:m-1)*180/pi)


% Get Depth Data
% msgs = readMessages(select(bag, 'Topic', '/naro_monitoring/depthLog'));
% n = length(msgs);
% m = length(msgs{1}.Data);
% 
% depthLogData = zeros(n,m);
% for i = 1:n
%    depthLogData(i,:) = msgs{i}.Data; 
% end
% 
% t = depthLogData(:,end);
% figure(2)
% plot(t, depthLogData(:,1:m-1))
