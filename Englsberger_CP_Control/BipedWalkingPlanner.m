clc
% close all
% clear all

%% Parameters

dt = 0.005;             % [s] : sampling time
hz = 1/dt;
h = 0.75;               % Desired com height
g = 9.81;               % gravity acceleration
sim_time = 10;
sim_tick = sim_time*hz;
w = sqrt(g/h);

t_step = 1;             % step time
t_lim = 0.05;           % to prevent system instability
CPS = logical(1);       % CP end of step control
CPT = logical(0);       % CP tracking control

%% CPS/CPT
bool_ = CPS

b = CPGainCalculator(dt, sim_tick, w, t_step, t_lim, bool_);
cp_desired_ = CPDesiredGenerator(dt, sim_tick, w, b, t_step, 0.2, 0.1, bool_);
[cp_, zmp_, com_] = CPControl(dt, sim_tick, w, b, t_step, t_lim, cp_desired_, bool_);

%% Discussion
% Why did Johannes suggest using the ZMP projection method while using CPT?
% It's because the CP reference calculated using the equation 
% cp_ref(i,:) = zmp_desired_(i,:) + exp(w*(time-time_offset))*(cp_init(i,:) - zmp_desired_(i,:))
% has a discontinuity in its initial value, which could cause an impulse response in the ZMP. 
% To prevent the ZMP from getting outside the support polygon due to this discontinuity, He recommended using the ZMP projection method I believe.

%% Plot

figure()
subplot(2,1,1)
plot([1:sim_tick]*dt, cp_(1:sim_tick,1), [1:sim_tick]*dt, cp_desired_(1:sim_tick,1))
title('X dir')
legend('cp','cp desired')
grid on
subplot(2,1,2)
plot([1:sim_tick]*dt, cp_(1:sim_tick,1), [1:sim_tick]*dt, zmp_(1:sim_tick,1), [1:sim_tick]*dt, com_(1:sim_tick,1))
title('X dir')
legend('cp', 'zmp', 'com')
grid on

figure()
subplot(2,1,1)
plot([1:sim_tick]*dt, cp_(1:sim_tick,2), [1:sim_tick]*dt, cp_desired_(1:sim_tick,2))
title('Y dir')
legend('cp','cp desired')
grid on
subplot(2,1,2)
plot([1:sim_tick]*dt, cp_(1:sim_tick,2), [1:sim_tick]*dt, zmp_(1:sim_tick,2), [1:sim_tick]*dt, com_(1:sim_tick,2))
title('Y dir')
legend('cp', 'zmp', 'com')
grid on