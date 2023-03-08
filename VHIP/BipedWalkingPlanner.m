clc
close all
clear all

%% Preview control
%
% This code is biped walking planner made by Kwanwoo Lee.
% I dealt with the Time varying DCM Planner.
% I mention two papers that I read before to make this code.
%
% 1. S. Kajita et al., "Biped walking pattern generation by using preview control of zero-moment point,"
% 2003 IEEE International Conference on Robotics and Automation
% 2. M. A. Hopkins, D. W. Hong and A. Leonessa, "Humanoid locomotion on uneven terrain using the time-varying divergent component of motion,"
% 2014 IEEE-RAS International Conference on Humanoid Robots

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
[CzTraj, CzRef]  = ComHeightGenerator(dt, sim_tick, h, 0.3);
[w, w_dot, a] = FrequencyPlanner(dt,sim_tick,CzTraj,g);         % Runge Kutta 4 method
[VRPxRef, VRPyRef, VRPzRef] = VrpGenerator(dt, sim_tick, ZxRef, ZyRef, CzTraj, w, w_dot,g)

%% Planner
[Gi,Gx,Gd,A,B,C] = PreviewParameter(dt,h,g,preview_tick,Qe,Qx,R);

[com_desired_, dcm_desired_] = PreviewController(dt,sim_tick,preview_tick,VRPxRef,VRPyRef,VRPzRef,A,B,C,Gi,Gx,Gd,w,w_dot,wn,g,h);
% [com_desired_, dcm_desired_] = PreviewController_depracated(dt,sim_tick,preview_tick,VRPxRef,VRPyRef,VRPzRef,CzTraj,A,B,C,Gi,Gx,Gd,w,w_dot,wn,g,h);

% I believed that this code would allow me to reduce the jerk in the z-direction of the center of mass (CoM) by using preview control.
% However, it seems that the dynamics of the CoM in the z-axis cannot be addressed by the Cart Table Model. 
% Therefore, I need to consider further development of this method.

%% Figure
figure()
plot([1:sim_tick]*dt,com_desired_(:,1), [1:sim_tick]*dt, dcm_desired_(:,1), [1:sim_tick]*dt, VRPxRef([1:2000]))
title('X dir')
legend('com desired', 'dcm desired', 'vrp reference')
grid on

figure()
plot([1:sim_tick]*dt,com_desired_(:,2), [1:sim_tick]*dt, dcm_desired_(:,2), [1:sim_tick]*dt, VRPyRef([1:2000]))
title('Y dir')
legend('com desired', 'dcm desired', 'vrp reference')
grid on

figure()
plot([1:sim_tick]*dt,com_desired_(:,3), [1:sim_tick]*dt, dcm_desired_(:,3), [1:sim_tick]*dt, dcm_desired_(:,4), [1:sim_tick]*dt, VRPzRef([1:2000]),[1:sim_tick]*dt, CzTraj([1:2000]) )
title('Z dir')
legend('com desired', 'dcm desired (VHIP)', 'dcm desired (LIPM)', 'vrp reference','Z polynomial')
grid on

figure()
plot([1:sim_tick]*(dt),w(1:sim_tick,1), [1:sim_tick]*(dt), w_dot([1:sim_tick]))
title('Frequency')
legend('w', 'wdot')
grid on