function [cp_, zmp_, com_] = CPControl(dt, sim_tick, w, b, t_step, t_lim, cp_desired_, CP_trigger)

cp_ = zeros(sim_tick,2);
zmp_ = zeros(sim_tick,2);
com_ = zeros(sim_tick,2);

for i = 1:1:sim_tick

    if(CP_trigger == 1)     % CPS control
        [cp_(i+1,:), zmp_(i+1,:), com_(i+1,:)] = CPEndofStepControl(dt, b(i,1), w, cp_(i,:), zmp_(i,:), com_(i,:), cp_desired_(i,:));
    elseif(CP_trigger == 0)     % CPT control   
        [cp_(i+1,:), zmp_(i+1,:), com_(i+1,:)] = CPTrackingControl(dt, b(i,1), w, cp_(i,:), zmp_(i,:), com_(i,:), cp_desired_(i,:));
    end
end
