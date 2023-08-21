clc
clear all
close all

%% Parameters

global g h w T

g = 9.81;
h = 0.75;
w = sqrt(g/h);
T = 1/400;  % 400 Hz

%% Reference trajectories

alpha = 0.01;
beta = 10;
ux = 0.0;

sim_tick = [0:1:2/T-1]';
sim_time = sim_tick*T;

for i = 1:1:length(sim_tick)
    com(1,i)    = alpha*sin(beta*sim_time(i,1));
    comdot(1,i) = alpha*beta*cos(beta*sim_time(i,1));
    cop(1,i)    = com(1,1) + (1/(w*w)) * (alpha*beta^2*sin(beta*sim_time(i,1)) + ux);
end

x = [com; comdot; cop];

%% State estimation

sys_s1 = [];
sys_s12 = [];
for i = 1:1:length(sim_tick)-1
    sys_s1  = Naive_StateEstimator_S1(i, x(:,i), sys_s1)
    sys_s12 = Naive_StateEstimator_S12(i, x(:,i), sys_s12)
end

Xhat_s1 = [sys_s1(1:end).xhat]'
Yhat_s1 = [sys_s1(1:end).yhat]'

Xhat_s12 = [sys_s12(1:end).xhat]'
Yhat_s12 = [sys_s12(1:end).yhat]'

%% Figure

figure()
plot(sim_time, com', sim_time, Xhat_s1(:,1), sim_time, Xhat_s12(:,1))
legend('measured', 'estimated (1)', 'estimated (2)')
title('com pos')
figure()
plot(sim_time, comdot', sim_time, Xhat_s1(:,2), sim_time, Xhat_s12(:,2))
legend('measured', 'estimated (1)', 'estimated (2)')
title('com vel')
figure()
plot(sim_time, cop', sim_time, Xhat_s1(:,3), sim_time, Xhat_s12(:,3))
legend('measured', 'estimated (1)', 'estimated (2)')
title('cop')
