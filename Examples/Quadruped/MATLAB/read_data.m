q = dlmread('q.mat',' ');
qd = dlmread('qd.mat',' ');
qdd = dlmread('qdd.mat',' ');
q_des = dlmread('q_des.mat',' ');
qd_des = dlmread('qd_des.mat',' ');
qdd_des = dlmread('qdd_des.mat',' ');

q_err = sum(sqrt((q(2:end,:)-q_des(1:end-1,:)).^2)');
qd_err = sum(sqrt((qd(2:end,:)-qd_des(1:end-1,:)).^2)');
qdd_err = sum(sqrt((qdd(2:end,:)-qdd_des(1:end-1,:)).^2)');
close all; figure;
% plot(q_err,'k')
% hold on;
plot(qd_err,'b')
% plot(qdd_err,'r')