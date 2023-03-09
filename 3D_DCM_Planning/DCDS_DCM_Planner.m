function [dcm_desired, dcm_dot_desired] = DCDS_DCM_Planner(dt, sim_tick, preview_window, t_step, current_step, w, eCMPRef)
% ξ(t)= vrp + e^√(g/Δz)t * (ξ0 − vrp)

step_tick = t_step / dt;

vrp1 = eCMPRef(1+(current_step+0) * step_tick,:);
vrp2 = eCMPRef(1+(current_step+1) * step_tick,:);
vrp3 = eCMPRef(1+(current_step+2) * step_tick,:);
vrp4 = eCMPRef(1+(current_step+3) * step_tick,:);

dcm_eos3 = vrp4;
dcm_eos2 = vrp3 + exp(-w*t_step)*(dcm_eos3 - vrp3);
dcm_eos1 = vrp2 + exp(-w*t_step)*(dcm_eos2 - vrp2);

for i = 1:1:preview_window
    time = i * dt;

    if (time < 1)
        dcm_desired(i,:) = vrp1 + exp(w*(time - 1*t_step))*(dcm_eos1 - vrp1);
        dcm_dot_desired(i,:) = w*exp(w*(time - 1*t_step))*(dcm_eos1 - vrp1);
    elseif (time < 2)
        dcm_desired(i,:) = vrp2 + exp(w*(time - 2*t_step))*(dcm_eos2 - vrp2);
        dcm_dot_desired(i,:) = w*exp(w*(time - 2*t_step))*(dcm_eos2 - vrp2);
    else
        dcm_desired(i,:) = vrp3 + exp(w*(time - 3*t_step))*(dcm_eos3 - vrp3);
        dcm_dot_desired(i,:) = w*exp(w*(time - 3*t_step))*(dcm_eos3 - vrp3);
    end
end

end