function b = CPGainCalculator(dt, sim_tick, w, t_step, t_lim, CP_trigger)

j = 0;

for i = 1:1:sim_tick

time = i*dt;


    if(CP_trigger == 1)     % CPS control
        if (and(rem(time, t_step) == 0,time < 7))
            dT(i,1) = t_step;
            j = 0;
        elseif(time < 7)
            dT(i,1) = t_step - j;
            if(dT(i,1) <= t_lim)
                dT(i,1) = t_lim;
            end
        else
            dT(i,1) = t_lim;
        end 
    elseif (CP_trigger == 0)    % CPT contrl
        dT(i,1) = t_lim;
    end
   
j=j+dt;

end

b(:,1) = exp(w*dT(:,1));

figure()
plot([1:sim_tick]*dt, dT(1:sim_tick,1))
title('dT')
grid on

end

