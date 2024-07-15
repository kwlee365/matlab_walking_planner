function [pRef] = OneStepZMP(time,phase_time, v0, vT)
    lin_interpol = time / phase_time;
    pRef         = (1- lin_interpol) * v0 + lin_interpol * vT;
end

