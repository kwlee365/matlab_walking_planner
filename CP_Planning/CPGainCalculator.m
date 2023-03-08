function b = CPGainCalculator(dt, sim_tick, w, t_step, t_lim, CP_trigger)

j = 0;

for i = 1:1:sim_tick

time = i*dt;


    if(CP_trigger == 1)     % CPS control
        if (rem(time, t_step) == 0)
            dT(i,1) = t_step;
            j = 0;
        else
            dT(i,1) = t_step - j;
            if(dT(i,1) <= t_lim)
                dT(i,1) = t_lim;
            end
        end 
    elseif (CP_trigger == 0)    % CPT contrl
        dT(i,1) = t_lim;
    end
   
j=j+dt;

end

b(:,1) = exp(w*dT(:,1));
end

