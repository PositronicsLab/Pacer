% x   = dlmread('x.mat',',');
% xd  = dlmread('xd.mat',',');
xdd = dlmread('xdd.mat',',');
% q   = dlmread('q.mat',',');
% qd  = dlmread('qd.mat',',');
% qdd = dlmread('qdd.mat',',');

foot = 1
plot([xdd(1:4:end,3)])
% inds = 50:450;
% xf = x(1:4:end,3);
% plot(xf(inds))
% axis([0 max(inds)-min(inds) min(xf(inds))-0.002 max(xf(inds))+0.002])
% title('Workspace Z Trajectory over 2 steps (left front foot)')
% xlabel('time [ms]')
% ylabel('Z position')

% plot(x(foot:4:end,1),x(foot:4:end,3))
% axis equal;
% axis([min(x(foot:4:end,1))-0.002 max(x(foot:4:end,1))+0.002 min(x(foot:4:end,3))-0.002 max(x(foot:4:end,3))+0.002])
% title('Workspace Trajectory of a step on x and z axis (left front foot)')
% xlabel('X position')
% ylabel('Z position')

% plot([q(1:4:end,1)])
