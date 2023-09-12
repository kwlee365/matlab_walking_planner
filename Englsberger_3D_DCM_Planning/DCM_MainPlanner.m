function [com_desired, dcm_desired, vrp_desired, dcm_dot_desired] =DCM_MainPlanner(dt, sim_tick, preview_window, step_time, dsp_time, w, vrpRef, h, bool_)

current_step = 0;
j = 1;
com_desired = zeros(sim_tick,3);
com_desired(1,3) = h;

for i=1:1:sim_tick
% for i=1:1:1
    
    time = i * dt;

    if(bool_ == 0)              % DCDS Planner
        [dcm_preview, dcm_dot_preview, vrp_preview] = DCDS_DCM_Planner(dt, sim_tick, preview_window, step_time, current_step, w, vrpRef);

        dcm_desired(i,:) = dcm_preview(j,:);
        dcm_dot_desired(i,:) = dcm_dot_preview(j,:);
        vrp_desired(i,:) = vrp_preview(j,:);

        j = j + 1;
    elseif(bool_ == 1)          % CDS Planner
        [dcm_preview, dcm_dot_preview, vrp_preview] = CDS_DCM_Planner(dt, sim_tick, preview_window, step_time, dsp_time, current_step, w, vrpRef);

        dcm_desired(i,:) = dcm_preview(j,:);
        dcm_dot_desired(i,:) = dcm_dot_preview(j,:);
        vrp_desired(i,:) = vrp_preview(j,:);

        j = j + 1;
    elseif(bool_ == 2)
        [dcm_preview, dcm_dot_preview, vrp_preview] = HT_DCM_Planner(dt, sim_tick, preview_window, step_time, dsp_time, current_step, w, vrpRef);

        dcm_desired(i,:) = dcm_preview(j,:);
        dcm_dot_desired(i,:) = dcm_dot_preview(j,:);
        vrp_desired(i,:) = vrp_preview(j,:);

        j = j + 1;
    else
        break
    end

    if(i == sim_tick)
        break
    end

    com_desired(i+1,:) = dcm_desired(i,:) + exp(-w*dt)*(com_desired(i,:) - dcm_desired(i,:));

    if(rem(time, step_time) == 0)
        current_step = current_step + 1;
        j = 1;
    end

end

% com_desired = ComCalculator(dt, dcm_desired, sim_tick, w, h);

end

