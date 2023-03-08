function [w, w_dot,a] = FrequencyPlanner(dt,sim_tick,CzTraj,g);
%% Runge-Kutta 4 method
% w^2 - wdot = a^2(t)vg
% wdot_recursive = a^2(tr) - w^2
% Reverse time derivative and integrate backwards.

h = 2*dt;

    w     = zeros(sim_tick,1);
    w_dot = zeros(sim_tick,1);

    for i = 1:1:sim_tick
        num = CzTraj(i, 3) + g;
        den = CzTraj(i, 1);
        a(i,1) = sqrt(num / den);
    end

    a(sim_tick+1,1) = a(sim_tick,1);
    a(sim_tick+2,1) = a(sim_tick,1);

    for j = sim_tick:-1:2
    
        if j==sim_tick
            w(j,1)     = a(sim_tick+2,1);               % initial value
            w_dot(j,1) = a(sim_tick+2,1)^2 - w(j,1)^2;
        end
    
        k1 = h*(a(j+2 ,1)^2 -   w(j,1)^2          );
        k2 = h*(a(j+1 ,1)^2 -  (w(j,1) + 0.5*k1)^2);
        k3 = h*(a(j+1 ,1)^2 -  (w(j,1) + 0.5*k2)^2);
        k4 = h*(a(j   ,1)^2 -  (w(j,1) + k3)^2    );
    
        w_dot(j-1,1) = (a(j,1)^2 - w(j,1)^2);
        w(j-1,1) = w(j,1) + (k1 + 2*k2 + 3*k3 + k4)/6;
    end

end

