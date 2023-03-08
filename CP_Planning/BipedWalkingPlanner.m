clc
close all
clear all

%% Preview control

%% Parameters
dt = 0.005;             % [s] : sampling time
hz = 1/dt;
h = 0.75;               % Desired com height
g = 9.81;               % gravity acceleration
sim_time = 7;
sim_tick = sim_time*hz;
w = sqrt(g/h);

t_step = 1;             % step time
t_lim = 0.05;           % to prevent system instability
CPS = logical(1);       % CP end of step control
CPT = logical(0);       % CP tracking control

%% CPS/CPT
bool_ = CPT

b = CPGainCalculator(dt, sim_tick, w, t_step, t_lim, bool_);
cp_desired_ = CPDesiredGenerator(dt, sim_tick, w, b, t_step, 0.2, 0.1, bool_);
[cp_, zmp_, com_] = CPControl(dt, sim_tick, w, b, t_step, t_lim, cp_desired_, bool_);

%% Plot

figure()
plot([1:sim_tick]*dt, cp_(1:sim_tick,1), [1:sim_tick]*dt, cp_desired_(1:sim_tick,1))
title('X dir')
legend('cp','cp desired')
grid on

figure()
plot([1:sim_tick]*dt, cp_(1:sim_tick,1), [1:sim_tick]*dt, zmp_(1:sim_tick,1), [1:sim_tick]*dt, com_(1:sim_tick,1), [1:sim_tick]*dt, cp_desired_(1:sim_tick,1))
title('X dir')
legend('cp', 'zmp', 'com','cp desired')
grid on

figure()
plot([1:sim_tick]*dt, cp_(1:sim_tick,2), [1:sim_tick]*dt, cp_desired_(1:sim_tick,2))
title('Y dir')
legend('cp','cp desired')
grid on

figure()
plot([1:sim_tick]*dt, cp_(1:sim_tick,2), [1:sim_tick]*dt, zmp_(1:sim_tick,2), [1:sim_tick]*dt, com_(1:sim_tick,2), [1:sim_tick]*dt, cp_desired_(1:sim_tick,2))
title('Y dir')
legend('cp', 'zmp', 'com','cp desired')
grid on