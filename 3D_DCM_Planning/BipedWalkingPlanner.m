clc
close all
clear all

%% 3D DCM Planner

% J. Englsberger, C. Ott and A. Albu-Schäffer,
% "Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion," in IEEE Transactions on Robotics,
% vol. 31, no. 2, pp. 355-368, April 2015, doi: 10.1109/TRO.2015.2405592.

%% Parameters
dt = 0.005;                 % [s] : sampling time
hz = 1/dt;
h = 0.75;                   % Desired com height
g = 9.81;                   % gravity acceleration
sim_time = 10;
sim_tick = sim_time*hz;
w = sqrt(g/h);              % h = z_vrp

t_step = 1;                 % step time

preview_window = 3*t_step*hz;   % next three desired VRPs

DCDS = logical(0);           % Discontinuous Double Support
CDS = logical(1);            % Continuous Double Support
HT = logical(2);             % Heel-To-Toe

%% Planner

bool_ = DCDS;

[vrpRef] = VrpGenerator(dt,sim_tick,0.5,0.1,h);

[com_desired, dcm_desired, dcm_desired_dot] = DCM_MainPlanner(dt, sim_tick, preview_window, t_step, w, vrpRef, h, bool_);

%% Plot

figure()
plot([1:sim_tick]*dt, dcm_desired(1:sim_tick,1), [1:sim_tick]*dt, vrpRef(1:sim_tick,1),[1:sim_tick]*dt, com_desired(1:sim_tick,1))
legend('DCM', 'VRP', 'COM')
title('X dir')
grid on

figure()
plot([1:sim_tick]*dt, dcm_desired(1:sim_tick,2), [1:sim_tick]*dt, vrpRef(1:sim_tick,2),[1:sim_tick]*dt, com_desired(1:sim_tick,2))
legend('DCM', 'VRP', 'COM')
title('Y dir')
grid on

figure()
plot([1:sim_tick]*dt, dcm_desired(1:sim_tick,3), [1:sim_tick]*dt, vrpRef(1:sim_tick,3),[1:sim_tick]*dt, com_desired(1:sim_tick,3))
legend('DCM', 'VRP', 'COM')
title('Z dir')
grid on

figure()
plot([1:sim_tick]*dt, dcm_desired_dot(1:sim_tick,1))
legend('dcm dot')
title('X dir')
grid on

figure()
plot([1:sim_tick]*dt, dcm_desired_dot(1:sim_tick,2))
legend('dcm dot')
title('Y dir')
grid on
