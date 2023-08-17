clc
close all
clear all

%% 3D DCM Planner

% J. Englsberger, C. Ott and A. Albu-Schäffer,
% "Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion," in IEEE Transactions on Robotics,
% vol. 31, no. 2, pp. 355-368, April 2015, doi: 10.1109/TRO.2015.2405592.

%% Parameters
dt = 0.005;                     % [s] : sampling time
hz = 1/dt;
h = 0.75;                       % Desired com height
g = 9.81;                       % gravity acceleration
sim_time = 10;
sim_tick = sim_time*hz; 
w = sqrt(g/h);                  % h = z_vrp

step_time = 1.0                     % step time
dsp_time = 0.2;                 % Double support phase

step_length = 0.5;
step_width = 0.2;

foot_length = 0.15;

preview_window = 3*step_time*hz;   % next three desired VRPs

DCDS = 0;                  % Discontinuous Double Support
CDS = 1;                   % Continuous Double Support
HT = 2;                    % Heel-To-Toe

%% Planner

bool_ = 1;

if(or(bool_ == DCDS, bool_ == CDS))
        [vrpRef] = VrpGenerator(dt,sim_tick,step_time,step_length, step_width, h);
elseif(bool_==HT)
        [vrpRef] = VrpHTGenerator(dt,sim_tick,step_time,step_length,step_width,foot_length, h);
else
        vrpRef = zeros(sim_tick + 600,3) ;
end
     
[com_desired, dcm_desired, vrp_desired, dcm_desired_dot] = DCM_MainPlanner(dt, sim_tick, preview_window, step_time, dsp_time, w, vrpRef, h, bool_);

%% Plot
 
figure()
subplot(2,1,1)
plot([1:sim_tick]*dt, dcm_desired(1:sim_tick,1), [1:sim_tick]*dt, vrp_desired(1:sim_tick,1),[1:sim_tick]*dt, com_desired(1:sim_tick,1))
legend('DCM', 'VRP des', 'COM')
title('X dir')
grid on
subplot(2,1,2)
plot([1:sim_tick]*dt, dcm_desired(1:sim_tick,2), [1:sim_tick]*dt, vrp_desired(1:sim_tick,2),[1:sim_tick]*dt, com_desired(1:sim_tick,2))
legend('DCM', 'VRP des', 'COM')
title('Y dir')
grid on

% figure()
% plot([1:sim_tick]*dt, dcm_desired(1:sim_tick,3), [1:sim_tick]*dt, vrp_desired(1:sim_tick,3),[1:sim_tick]*dt, com_desired(1:sim_tick,3))
% legend('DCM', 'VRP des', 'COM')
% title('Z dir')
% grid on

% figure()
% plot([1:sim_tick]*dt, dcm_desired_dot(1:sim_tick,1))
% legend('dcm dot')
% title('X dir')
% grid on

% figure()
% plot([1:sim_tick]*dt, dcm_desired_dot(1:sim_tick,2))
% legend('dcm dot')
% title('Y dir')
% grid on
