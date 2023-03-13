function [dcm_desired, dcm_dot_desired, vrp_desired] = HT_DCM_Planner(dt, sim_tick, preview_window, step_time, dsp_time, current_step, w, vrpRef)

% ξ(t)= vrp + e^√(g/Δz)t * (ξ0 − vrp)

step_tick = step_time / dt;

alpha = 0.5;

% Single support phase

vrp1 = vrpRef(1+(current_step+0) * step_tick,:);
vrp2 = vrpRef(1+(current_step+1) * step_tick,:);
vrp3 = vrpRef(1+(current_step+2) * step_tick,:);
vrp4 = vrpRef(1+(current_step+3) * step_tick,:);

dcm_eos3 = vrp4;
dcm_eos2 = vrp3 + exp(-w*step_time)*(dcm_eos3 - vrp3);
dcm_eos1 = vrp2 + exp(-w*step_time)*(dcm_eos2 - vrp2);
dcm_eos0 = vrp1 + exp(-w*step_time)*(dcm_eos1 - vrp1);

dcm_eos3 = vrp4;
dcm_eos2 = vrp3 + exp(-w*step_time)*(dcm_eos3 - vrp3);
dcm_eos1 = vrp2 + exp(-w*step_time)*(dcm_eos2 - vrp2);
dcm_eos0 = vrp1 + exp(-w*step_time)*(dcm_eos1 - vrp1);

dcm_ini1 = dcm_eos0;
dcm_ini2 = dcm_eos1;
dcm_ini3 = dcm_eos2;
dcm_ini4 = dcm_eos3;

% Double suppot phase
% ini_DS : 4,5,6 columns
% eoDS : 1,2,3 columns

dcm_ini_DS2 = vrp1 + exp(-w*alpha*dsp_time)*(dcm_ini2 - vrp1);
dcm_ini_DS3 = vrp2 + exp(-w*alpha*dsp_time)*(dcm_ini3 - vrp2);
dcm_ini_DS4 = vrp3 + exp(-w*alpha*dsp_time)*(dcm_ini4 - vrp3);

dcm_end_DS1 = vrp1 + exp( w*alpha*dsp_time)*(dcm_ini1 - vrp1);
dcm_end_DS2 = vrp2 + exp( w*alpha*dsp_time)*(dcm_ini2 - vrp2);
dcm_end_DS3 = vrp3 + exp( w*alpha*dsp_time)*(dcm_ini3 - vrp3);

dcm_ini_dot_DS2 = w*exp(-w*alpha*dsp_time)*(dcm_ini2 - vrp1);
dcm_ini_dot_DS3 = w*exp(-w*alpha*dsp_time)*(dcm_ini3 - vrp2);
dcm_ini_dot_DS4 = w*exp(-w*alpha*dsp_time)*(dcm_ini4 - vrp3);

dcm_end_dot_DS1 = w*exp( w*alpha*dsp_time)*(dcm_ini1 - vrp1);
dcm_end_dot_DS2 = w*exp( w*alpha*dsp_time)*(dcm_ini2 - vrp2);
dcm_end_dot_DS3 = w*exp( w*alpha*dsp_time)*(dcm_ini3 - vrp3);

% Controller

for i = 1:1:preview_window
    time = i * dt;

    if(time < step_time - dsp_time)                            % SSP

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, 0.0, step_time - dsp_time, dcm_ini_DS2(1), dcm_end_DS2(1), dcm_ini_dot_DS2(1), dcm_end_dot_DS2(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, 0.0, step_time - dsp_time, dcm_ini_DS2(2), dcm_end_DS2(2), dcm_ini_dot_DS2(2), dcm_end_dot_DS2(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, 0.0, step_time - dsp_time, dcm_ini_DS2(3), dcm_end_DS2(3), dcm_ini_dot_DS2(3), dcm_end_dot_DS2(3));

    elseif(time < step_time)                                   % DSP

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, step_time - dsp_time, step_time, dcm_ini_DS2(1), dcm_end_DS2(1), dcm_ini_dot_DS2(1), dcm_end_dot_DS2(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, step_time - dsp_time, step_time, dcm_ini_DS2(2), dcm_end_DS2(2), dcm_ini_dot_DS2(2), dcm_end_dot_DS2(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, step_time - dsp_time, step_time, dcm_ini_DS2(3), dcm_end_DS2(3), dcm_ini_dot_DS2(3), dcm_end_dot_DS2(3));

    elseif(time < 2*step_time - dsp_time)

        dcm_desired(i,:) = vrp2 + exp(w*(time - (2*step_time - dsp_time)))*(dcm_ini_DS3 - vrp2);
        dcm_dot_desired(i,:) =  w*exp(w*(time - (2*step_time - dsp_time)))*(dcm_ini_DS3 - vrp2);

    elseif(time < 2*step_time)

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, 2*step_time - dsp_time, 2*step_time, dcm_ini_DS3(1), dcm_end_DS3(1), dcm_ini_dot_DS3(1), dcm_end_dot_DS3(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, 2*step_time - dsp_time, 2*step_time, dcm_ini_DS3(2), dcm_end_DS3(2), dcm_ini_dot_DS3(2), dcm_end_dot_DS3(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, 2*step_time - dsp_time, 2*step_time, dcm_ini_DS3(3), dcm_end_DS3(3), dcm_ini_dot_DS3(3), dcm_end_dot_DS3(3));
    else
        dcm_desired(i,:) = vrp3 + exp(w*(time - 3*step_time))*(dcm_eos3 - vrp3);
        dcm_dot_desired(i,:) =  w*exp(w*(time - 3*step_time))*(dcm_eos3 - vrp3);
    end

    vrp_desired(i,:) = dcm_desired(i,:) - (1/w)*dcm_dot_desired(i,:);
end

end
