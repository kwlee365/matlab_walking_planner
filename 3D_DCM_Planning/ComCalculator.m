function [com_] = ComCalculator(dt, dcm_, sim_tick, w, height)

%% Runge-Kutta 4 method
% com dot = F = w(dcm - com)

h = 2*dt;

x      = zeros(sim_tick,3);

dcm_(sim_tick+1,:) = dcm_(sim_tick,:);
dcm_(sim_tick+2,:) = dcm_(sim_tick,:);

for j = 1:1:sim_tick

    if(j==1)
        x(1,3) = height
    end

    k1 = w*(dcm_(j   ,:) -  x(j,:)            );
    k2 = w*(dcm_(j+1 ,:) - (x(j,:) + 0.5*h*k1));
    k3 = w*(dcm_(j+1 ,:) - (x(j,:) + 0.5*h*k2));
    k4 = w*(dcm_(j+2 ,:) - (x(j,:) +     h*k3));

    if(j == sim_tick)
        break
    end

    x(j+1,:) = x(j,:) + h*(k1 + 2*k2 + 2*k3 + k4)/6;
end

com_ = x;

end

