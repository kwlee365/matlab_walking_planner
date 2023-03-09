function [com_desired, dcm_desired, dcm_dot_desired] =DCM_MainPlanner(dt, sim_tick, preview_window, t_step, w, vrpRef, h, bool_)

current_step = 0;
j = 1;

for i=1:1:sim_tick
    
    time = i * dt;

    if(bool_ == 0)              % DCDS Planner
        [dcm_preview, dcm_dot_preview] = DCDS_DCM_Planner(dt, sim_tick, preview_window, t_step, current_step, w, vrpRef);

        dcm_desired(i,:) = dcm_preview(j,:);
        dcm_dot_desired(i,:) = dcm_dot_preview(j,:);

        j = j + 1;
    elseif(bool_ == 1)
        break
    elseif(bool_ == 2)
        break
    else
        break
    end

    if(i == sim_tick)
        break
    end

    if(rem(time, t_step) == 0)
        current_step = current_step + 1;
        j = 1;
    end

end

com_desired = ComCalculator(dt, dcm_desired, sim_tick, w, h);

end

