function [vrpRef] = VrpGenerator(dt,sim_tick,step_time,step_length, step_width, h)

for i = 1:1:sim_tick + 600

    time = i*dt;

    if time < step_time;
        vrpRef(i, 1) = 0;
        vrpRef(i, 2) = 0;
        vrpRef(i, 3) = h;
    elseif time < 2*step_time;
        vrpRef(i, 1) = step_length;
        vrpRef(i, 2) = step_width;
        vrpRef(i, 3) = h+0.05;
    elseif time < 3*step_time;
        vrpRef(i, 1) = step_length*2;
        vrpRef(i, 2) = -step_width;
        vrpRef(i, 3) = h+0.1;

    elseif time < 4*step_time;
        vrpRef(i, 1) = step_length*3;
        vrpRef(i, 2) = step_width;
        vrpRef(i, 3) = h+0.05;

    elseif time < 5*step_time;
        vrpRef(i, 1) = step_length*4;
        vrpRef(i, 2) = -step_width;
        vrpRef(i, 3) = h;

    elseif time < 6*step_time;
        vrpRef(i, 1) = step_length*5;
        vrpRef(i, 2) = step_width;
        vrpRef(i, 3) = h;

    elseif time < 7*step_time;
        vrpRef(i, 1) = step_length*6;
        vrpRef(i, 2) = -step_width;
        vrpRef(i, 3) = h;
    else
        vrpRef(i, 1) = step_length*6;
        vrpRef(i, 2) = 0.;
        vrpRef(i, 3) = h;

    end
end

end