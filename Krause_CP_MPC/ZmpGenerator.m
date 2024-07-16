function [ZxRef, ZyRef] = ZmpGenerator(dt, sim_tick, pRef, ssp_time, dsp_time, step_time, number_of_step)

    step_cnt = 1;
    step_tick = step_time / dt;

    for i = 1:1:sim_tick + 2.0 / dt

        if step_cnt > number_of_step + 2
            break
        end
    
        time = i*dt;
    
        local_time = time - (step_cnt-1) * step_time;
        v0_x = pRef(1, step_cnt);
        v0_y = pRef(2, step_cnt);
        vT_x = pRef(1, step_cnt+1);
        vT_y = pRef(2, step_cnt+1);

        if local_time < dsp_time;
            lin_interpol = local_time / dsp_time;
            ZxRef(i) = OneStepZMP(local_time, dsp_time, v0_x, vT_x);
            ZyRef(i) = OneStepZMP(local_time, dsp_time, v0_y, vT_y);
        else
            ZxRef(i) = vT_x;
            ZyRef(i) = vT_y;
        end
       
        if (rem(i, step_tick) == 0)
            i;
            step_cnt = step_cnt + 1;
        end
    end
    
    ZxRef = ZxRef';
    ZyRef = ZyRef';

end