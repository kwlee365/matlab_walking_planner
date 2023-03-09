function [vrpRef] = VrpGenerator(dt,sim_tick,footstep_x,footstep_y,h)

for i = 1:1:sim_tick + 600

    time = i*dt;

    if time < 2;
        vrpRef(i, 1) = 0;
        vrpRef(i, 2) = 0;
        vrpRef(i, 3) = h;
    elseif time < 3;
        vrpRef(i, 1) = footstep_x;
        vrpRef(i, 2) = footstep_y;
        vrpRef(i, 3) = h+0.05;
    elseif time < 4;
        vrpRef(i, 1) = footstep_x*2;
        vrpRef(i, 2) = -footstep_y;
        vrpRef(i, 3) = h+0.1;

    elseif time < 5;
        vrpRef(i, 1) = footstep_x*3;
        vrpRef(i, 2) = footstep_y;
        vrpRef(i, 3) = h+0.05;

    elseif time < 6;
        vrpRef(i, 1) = footstep_x*4;
        vrpRef(i, 2) = -footstep_y;
        vrpRef(i, 3) = h;

    else
        vrpRef(i, 1) = footstep_x*5;
        vrpRef(i, 2) = 0.;
        vrpRef(i, 3) = h;

    end
end

end