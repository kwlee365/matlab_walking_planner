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
preview_window =2;   % [s]
preview_tick = preview_window * hz;
Qe = 1.0;            % preview gain
Qx = zeros(3,3);     % preview gain
R = 1.0*10^(-6);
wn = sqrt(g/h);

foot_length_x = 0.3;
foot_length_y = 0.26;

%% Reference ZMP, CoM Generator

[ZxRef, ZyRef] = ZmpGenerator(dt, sim_tick,0.4, 0.2);

%% Planner
[Gi,Gx,Gd,A,B,C] = PreviewParameter(dt,h,g,preview_tick,Qe,Qx,R);

[com_desired_, dcm_desired_, x_hat, y_hat, zmp_x_hat, zmp_y_hat, ZyRef] = PreviewController(dt,sim_tick,preview_tick,ZxRef,ZyRef,A,B,C,Gi,Gx,Gd,wn,g,h);

%% Figure

T = dt
figure()
hold on
plot(T*[1:sim_tick],ZxRef([1:sim_tick]));
plot(T*[1:sim_tick],zmp_x_hat(1,[1:sim_tick]));
plot(T*[1:sim_tick],x_hat(1,[1:sim_tick]));plot([1:sim_tick]*dt,ZxRef([1:sim_tick])-0.14,'k--');
plot([1:sim_tick]*dt,ZxRef([1:sim_tick])+0.16,'k--');
plot([1:sim_tick]*dt,ZxRef([1:sim_tick])-0.14,'k--');
title('X')
legend('ZMP Ref', 'ZMP act', 'CoM','ZMP min', 'ZMP max')

figure()
hold on
plot(T*[1:sim_tick],ZyRef([1:sim_tick]));
plot(T*[1:sim_tick],zmp_y_hat(1,[1:sim_tick]));
plot(T*[1:sim_tick],y_hat(1,[1:sim_tick]));
plot(T*[1:sim_tick],ZyRef([1:sim_tick])-0.5*foot_length_y,'k--');
plot(T*[1:sim_tick],ZyRef([1:sim_tick])+0.5*foot_length_y,'k--');
title('Y')
legend('ZMP Ref', 'ZMP act', 'CoM','ZMP min', 'ZMP max')