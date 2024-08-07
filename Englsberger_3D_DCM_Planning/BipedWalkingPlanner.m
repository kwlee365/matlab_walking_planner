clc
% close all
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

vrpRef'
plot(vrpRef)
 
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
% hold on
% plot(dcm_desired(1:sim_tick,1), dcm_desired(1:sim_tick,2),vrp_desired(1:sim_tick,1),vrp_desired(1:sim_tick,2),com_desired(1:sim_tick,1), com_desired(1:sim_tick,2))

% total_step = sim_time / step_time;
% for i=1:1:total_step
%     i
%     a = int16(i * step_time * hz) 
%     b = int16((i*step_time - 0.5*dsp_time)*hz) 
%     c = int16((i*step_time + 0.5*dsp_time)*hz) 
% 
% 
%     dcm_eos_x(i,1) = dcm_desired(a,1);
%     dcm_eos_y(i,1) = dcm_desired(a,2);
%     dcm_iniDS_x(i,1) = dcm_desired(b,1);
%     dcm_iniDS_y(i,1) = dcm_desired(b,2);
%     if (i == total_step)
%         break
%     else
%         dcm_eoDS_x(i,1)  = dcm_desired(c,1);
%         dcm_eoDS_y(i,1)  = dcm_desired(c,2);
%     end
% end
% plot(dcm_eos_x,dcm_eos_y,'ro')
% plot(dcm_iniDS_x,dcm_iniDS_y,'bo')
% plot(dcm_eoDS_x,dcm_eoDS_y,'go')
% legend('DCM', 'VRP des', 'COM','eos', 'iniDS', 'eoDS')
% grid on
% hold off
