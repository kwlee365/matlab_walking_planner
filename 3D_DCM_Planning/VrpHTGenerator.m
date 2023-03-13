function [vrpRef] = VrpHTGenerator(dt,sim_tick,step_time,step_length, step_width,foot_length, h);

% vrpRef(:,1) = vrp Heel in the x direction
% vrpRef(:,2) = vrp Heel in the y direction
% vrpRef(:,3) = vrp Heel in the z direction
% vrpRef(:,4) = vrp Toe in the x direction
% vrpRef(:,5) = vrp Toe in the y direction
% vrpRef(:,6) = vrp Toe in the z direction

for i = 1:1:sim_tick + 600

    time = i*dt;

    if time < step_time;
        vrpRef(i, 1) = 0;
        vrpRef(i, 2) = 0;
        vrpRef(i, 3) = h;
        vrpRef(i, 4) = 0;
        vrpRef(i, 5) = 0;
        vrpRef(i, 6) = h;
    elseif time < 2*step_time;
        vrpRef(i, 1) = step_length - foot_length * 0.35;
        vrpRef(i, 2) = step_width;
        vrpRef(i, 3) = h+0.05;
        vrpRef(i, 4) = step_length + foot_length * 0.35;
        vrpRef(i, 5) = step_width;
        vrpRef(i, 6) = h+0.05;
    elseif time < 3*step_time;
        vrpRef(i, 1) = step_length*2 - foot_length * 0.35;
        vrpRef(i, 2) = -step_width;
        vrpRef(i, 3) = h+0.1;       
        vrpRef(i, 4) = step_length*2 + foot_length * 0.35;
        vrpRef(i, 5) = -step_width;
        vrpRef(i, 6) = h+0.1;
    elseif time < 4*step_time;
        vrpRef(i, 1) = step_length*3 - foot_length * 0.35;
        vrpRef(i, 2) = step_width;
        vrpRef(i, 3) = h+0.05;
        vrpRef(i, 4) = step_length*3 + foot_length * 0.35;
        vrpRef(i, 5) = step_width;
        vrpRef(i, 6) = h+0.05;
    elseif time < 5*step_time;
        vrpRef(i, 1) = step_length*4 - foot_length * 0.35;
        vrpRef(i, 2) = -step_width;
        vrpRef(i, 3) = h;
        vrpRef(i, 4) = step_length*4 + foot_length * 0.35;
        vrpRef(i, 5) = -step_width;
        vrpRef(i, 6) = h;
    elseif time < 6*step_time;
        vrpRef(i, 1) = step_length*5 - foot_length * 0.35;
        vrpRef(i, 2) = step_width;
        vrpRef(i, 3) = h;
        vrpRef(i, 4) = step_length*5 + foot_length * 0.35;
        vrpRef(i, 5) = step_width;
        vrpRef(i, 6) = h;
    elseif time < 7*step_time;
        vrpRef(i, 1) = step_length*6 - foot_length * 0.35;
        vrpRef(i, 2) = -step_width;
        vrpRef(i, 3) = h;
        vrpRef(i, 4) = step_length*6;
        vrpRef(i, 5) = -step_width;
        vrpRef(i, 6) = h;
    else
        vrpRef(i, 1) = step_length*6;
        vrpRef(i, 2) = 0.0;
        vrpRef(i, 3) = h;
        vrpRef(i, 4) = step_length*6;
        vrpRef(i, 5) = 0.0;
        vrpRef(i, 6) = h;
    end
end

end