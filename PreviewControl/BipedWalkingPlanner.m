clc
close all
clear all

%% Preview control

%% Parameters
dt = 0.005;         % [s] : sampling time
hz = 1/dt;
h = 0.75;           % Desired com height
g = 9.81;           % gravity acceleration
sim_time = 10;
sim_tick = sim_time*hz;
preview_window = 2;   % [s]
preview_tick = preview_window * hz;
Qe = 1.0;            % preview gain
Qx = zeros(3,3);     % preview gain
R = 1.0*10^(-6);
wn = sqrt(g/h);

%% Reference ZMP, CoM Generator

[ZxRef, ZyRef] = ZmpGenerator(dt, sim_tick,0.2, 0.1);

%% Planner
[Gi,Gx,Gd,A,B,C] = PreviewParameter(dt,h,g,preview_tick,Qe,Qx,R);

[com_desired_, dcm_desired_] = PreviewController(dt,sim_tick,preview_tick,ZxRef,ZyRef,A,B,C,Gi,Gx,Gd,wn,g,h);

%% Figure
figure()
plot([1:sim_tick]*dt,com_desired_(:,1), [1:sim_tick]*dt, dcm_desired_(:,1), [1:sim_tick]*dt, ZxRef([1:2000]))
title('X dir')
legend('com desired', 'dcm desired', 'zmp reference')
grid on

figure()
plot([1:sim_tick]*dt,com_desired_(:,2), [1:sim_tick]*dt, dcm_desired_(:,2), [1:sim_tick]*dt, ZyRef([1:2000]))
title('Y dir')
legend('com desired', 'dcm desired', 'zmp reference')
grid on