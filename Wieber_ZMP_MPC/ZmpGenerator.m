function [ZxRef, ZyRef, VxRef, VyRef] = ZmpGenerator(dt, sim_tick,footstep_x, footstep_y, footstep_time)

    for i = 1:1:sim_tick + 500
    
        time = i*dt;
    
        if time < 2;
            ZxRef(i) = 0;
            ZyRef(i) = 0;
            VxRef(i) = 0;
            VyRef(i) = 0;
        elseif time < 3;
            ZxRef(i) = footstep_x;
            ZyRef(i) = footstep_y;
            VxRef(i) = footstep_x / footstep_time;
            VyRef(i) = 0;
        elseif time < 4;
            ZxRef(i) = footstep_x*2;
            ZyRef(i) = -footstep_y;
            VxRef(i) = footstep_x / footstep_time;
            VyRef(i) = 0;
        elseif time < 5;
            ZxRef(i) = footstep_x*3;
            ZyRef(i) = footstep_y;
            VxRef(i) = footstep_x / footstep_time;
            VyRef(i) = 0;
        elseif time < 6;
            ZxRef(i) = footstep_x*4;
            ZyRef(i) = -footstep_y;
            VxRef(i) = footstep_x / footstep_time;
            VyRef(i) = 0;
        else
            ZxRef(i) = footstep_x*5;
            ZyRef(i) = 0.;
            VxRef(i) = 0;
            VyRef(i) = 0;
        end    
    end
    
    ZxRef = ZxRef';
    ZyRef = ZyRef';
    VxRef = VxRef';
    VyRef = VyRef';

end