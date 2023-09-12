function [dcm_desired, dcm_dot_desired, vrp_desired] = HT_DCM_Planner(dt, sim_tick, preview_window, step_time, dsp_time, current_step, w, vrpRef)

% ξ(t)= vrp + e^√(g/Δz)t * (ξ0 − vrp)

step_tick = step_time / dt;

alpha = 0.5;           % Double support phase parameter

beta_HT = 0.2;         % Heel to Toe parameter (I think 0.5 is bit weird.)
beta_TH = 1 - beta_HT;

% Single support phase

vrpH1 = vrpRef(1+(current_step+0) * step_tick, 1:3);
vrpH2 = vrpRef(1+(current_step+1) * step_tick, 1:3);
vrpH3 = vrpRef(1+(current_step+2) * step_tick, 1:3);
vrpH4 = vrpRef(1+(current_step+3) * step_tick, 1:3);

vrpT1 = vrpRef(1+(current_step+0) * step_tick, 4:6);
vrpT2 = vrpRef(1+(current_step+1) * step_tick, 4:6);
vrpT3 = vrpRef(1+(current_step+2) * step_tick, 4:6);
vrpT4 = vrpRef(1+(current_step+3) * step_tick, 4:6);

% Single support phase -> recursive way
dcm_ini_TH4 = vrpT4;
dcm_ini_HT4 = vrpH4 + exp(-w*beta_HT*step_time)*(dcm_ini_TH4 - vrpH4);

dcm_ini_TH3 = vrpT3 + exp(-w*beta_TH*step_time)*(dcm_ini_HT4 - vrpT3);
dcm_ini_HT3 = vrpH3 + exp(-w*beta_HT*step_time)*(dcm_ini_TH3 - vrpH3);

dcm_ini_TH2 = vrpT2 + exp(-w*beta_TH*step_time)*(dcm_ini_HT3 - vrpT2);
dcm_ini_HT2 = vrpH2 + exp(-w*beta_HT*step_time)*(dcm_ini_TH2 - vrpH2);

dcm_ini_TH1 = vrpT1 + exp(-w*beta_TH*step_time)*(dcm_ini_HT2 - vrpT1);
dcm_ini_HT1 = vrpH1 + exp(-w*beta_HT*step_time)*(dcm_ini_TH1 - vrpH1);

% Double support phase
dcm_ini_DS2 = vrpT1 + exp(-w*alpha*dsp_time)*(dcm_ini_HT2 - vrpT1);
dcm_ini_DS3 = vrpT2 + exp(-w*alpha*dsp_time)*(dcm_ini_HT3 - vrpT2);
dcm_ini_DS4 = vrpT3 + exp(-w*alpha*dsp_time)*(dcm_ini_HT4 - vrpT3);

dcm_end_DS1 = vrpH1 + exp( w*alpha*dsp_time)*(dcm_ini_HT1 - vrpH1);
dcm_end_DS2 = vrpH2 + exp( w*alpha*dsp_time)*(dcm_ini_HT2 - vrpH2);
dcm_end_DS3 = vrpH3 + exp( w*alpha*dsp_time)*(dcm_ini_HT3 - vrpH3);
dcm_end_DS4 = (vrpH4 + vrpT4) / 2;

dcm_ini_dot_DS2 = w*exp(-w*alpha*dsp_time)*(dcm_ini_HT2 - vrpT1);
dcm_ini_dot_DS3 = w*exp(-w*alpha*dsp_time)*(dcm_ini_HT3 - vrpT2);
dcm_ini_dot_DS4 = w*exp(-w*alpha*dsp_time)*(dcm_ini_HT4 - vrpT3);

dcm_end_dot_DS1 = w*exp( w*alpha*dsp_time)*(dcm_ini_HT1 - vrpH1);
dcm_end_dot_DS2 = w*exp( w*alpha*dsp_time)*(dcm_ini_HT2 - vrpH2);
dcm_end_dot_DS3 = w*exp( w*alpha*dsp_time)*(dcm_ini_HT3 - vrpH3);

% Controller

for i = 1:1:preview_window
    time = i * dt;

    if(time < step_time - dsp_time)                            % SSP

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, 0.0, step_time - dsp_time, dcm_end_DS1(1), dcm_ini_DS2(1), dcm_end_dot_DS1(1), dcm_ini_dot_DS2(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, 0.0, step_time - dsp_time, dcm_end_DS1(2), dcm_ini_DS2(2), dcm_end_dot_DS1(2), dcm_ini_dot_DS2(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, 0.0, step_time - dsp_time, dcm_end_DS1(3), dcm_ini_DS2(3), dcm_end_dot_DS1(3), dcm_ini_dot_DS2(3));

    elseif(time < step_time)                                   % DSP

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, step_time - dsp_time, step_time, dcm_ini_DS2(1), dcm_end_DS2(1), dcm_ini_dot_DS2(1), dcm_end_dot_DS2(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, step_time - dsp_time, step_time, dcm_ini_DS2(2), dcm_end_DS2(2), dcm_ini_dot_DS2(2), dcm_end_dot_DS2(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, step_time - dsp_time, step_time, dcm_ini_DS2(3), dcm_end_DS2(3), dcm_ini_dot_DS2(3), dcm_end_dot_DS2(3));

    elseif(time < 2*step_time - dsp_time)

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, step_time, 2*step_time - dsp_time, dcm_end_DS2(1), dcm_ini_DS3(1), dcm_end_dot_DS2(1), dcm_ini_dot_DS3(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, step_time, 2*step_time - dsp_time, dcm_end_DS2(2), dcm_ini_DS3(2), dcm_end_dot_DS2(2), dcm_ini_dot_DS3(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, step_time, 2*step_time - dsp_time, dcm_end_DS2(3), dcm_ini_DS3(3), dcm_end_dot_DS2(3), dcm_ini_dot_DS3(3));

    elseif(time < 2*step_time)

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, 2*step_time - dsp_time, 2*step_time, dcm_ini_DS3(1), dcm_end_DS3(1), dcm_ini_dot_DS3(1), dcm_end_dot_DS3(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, 2*step_time - dsp_time, 2*step_time, dcm_ini_DS3(2), dcm_end_DS3(2), dcm_ini_dot_DS3(2), dcm_end_dot_DS3(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, 2*step_time - dsp_time, 2*step_time, dcm_ini_DS3(3), dcm_end_DS3(3), dcm_ini_dot_DS3(3), dcm_end_dot_DS3(3));

    elseif(time <  3*step_time - dsp_time)

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, 2*step_time, 3*step_time - dsp_time, dcm_end_DS3(1), dcm_ini_DS4(1), dcm_end_dot_DS3(1), dcm_ini_dot_DS4(1));
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, 2*step_time, 3*step_time - dsp_time, dcm_end_DS3(2), dcm_ini_DS4(2), dcm_end_dot_DS3(2), dcm_ini_dot_DS4(2));
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, 2*step_time, 3*step_time - dsp_time, dcm_end_DS3(3), dcm_ini_DS4(3), dcm_end_dot_DS3(3), dcm_ini_dot_DS4(3));
    
    else

        [dcm_desired(i,1), dcm_dot_desired(i,1)] = ThirdOrderPolynomial(time, 3*step_time - dsp_time, 3*step_time, dcm_ini_DS4(1), dcm_end_DS4(1), dcm_ini_dot_DS4(1), 0.0);
        [dcm_desired(i,2), dcm_dot_desired(i,2)] = ThirdOrderPolynomial(time, 3*step_time - dsp_time, 3*step_time, dcm_ini_DS4(2), dcm_end_DS4(2), dcm_ini_dot_DS4(2), 0.0);
        [dcm_desired(i,3), dcm_dot_desired(i,3)] = ThirdOrderPolynomial(time, 3*step_time - dsp_time, 3*step_time, dcm_ini_DS4(3), dcm_end_DS4(3), dcm_ini_dot_DS4(3), 0.0);

    end

    vrp_desired(i,:) = dcm_desired(i,:) - (1/w)*dcm_dot_desired(i,:);
end

end
