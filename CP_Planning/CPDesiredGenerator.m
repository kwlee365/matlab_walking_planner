function cp_desired_ = CPDesiredGenerator(dt, sim_tick, w, b, t_step, footstep_x, footstep_y, CP_trigger)

cp_offset_x = 0.02;
time_offset = 0;
for i = 1:1:sim_tick + 100

    time = i*dt;

    if time < 2;
        cp_init(i,1) = 0;
        cp_init(i,2) = 0;
        cp_eos(i,1) = 0;
        cp_eos(i,2) = 0;
    elseif time < 3;
        cp_init(i,1) = cp_offset_x;
        cp_init(i,2) = 0;
        cp_eos(i,1) = footstep_x + cp_offset_x;
        cp_eos(i,2) = footstep_y;
    elseif time < 4;
        cp_init(i,1) = footstep_x + cp_offset_x;
        cp_init(i,2) = footstep_y;
        cp_eos(i,1) = footstep_x*2 + cp_offset_x;
        cp_eos(i,2) = -footstep_y;
    elseif time < 5;
        cp_init(i,1) = footstep_x*2 + cp_offset_x;
        cp_init(i,2) = -footstep_y;
        cp_eos(i,1) = footstep_x*3 + cp_offset_x;
        cp_eos(i,2) = footstep_y;
    elseif time < 6;
        cp_init(i,1) = footstep_x*3 + cp_offset_x;
        cp_init(i,2) = footstep_y;
        cp_eos(i,1) = footstep_x*4 + cp_offset_x;
        cp_eos(i,2) = -footstep_y;
    else
        cp_init(i,1) = footstep_x*4 + cp_offset_x;
        cp_init(i,2) = -footstep_y;
        cp_eos(i,1) = footstep_x*5;
        cp_eos(i,2) = 0.;
    end

    if(and(rem(time, t_step) == 0.0, time < sim_tick*dt))
        time_offset=time_offset+t_step + dt;
    end

    if(time < 7)
        zmp_desired_(i,:) = (1/(1-exp(w*t_step))) * cp_eos(i,:) - (exp(w*t_step)/(1-exp(w*t_step))) * cp_init(i,:);
        cp_ref(i,:) = zmp_desired_(i,:) + exp(w*(time-time_offset))*(cp_init(i,:) - zmp_desired_(i,:));
    else
        zmp_desired_(i,1) = footstep_x*5;
        zmp_desired_(i,2) = 0.0;
        cp_ref(i,1) = footstep_x*5;
        cp_ref(i,2) = 0.0;
    end


end

if(CP_trigger == 1) % CPS
    cp_desired_ = cp_eos;
elseif(CP_trigger == 0) 
    for j = 1:1:sim_tick
            cp_desired_(j,:) = cp_ref(j + 10,:);    % dT * hz_ = 10;
    end
%     figure()
%     plot([1:sim_tick]*dt, zmp_desired_(1:sim_tick,1), [1:sim_tick]*dt, cp_eos(1:sim_tick,1), [1:sim_tick]*dt, cp_desired_(1:sim_tick,1))
%     legend('zmp', 'cp eos', 'cp des')
%     title('X dir')
%     figure()
%     plot([1:sim_tick]*dt, zmp_desired_(1:sim_tick,2), [1:sim_tick]*dt, cp_eos(1:sim_tick,2), [1:sim_tick]*dt, cp_desired_(1:sim_tick,2))
%     legend('zmp', 'cp eos', 'cp des')
%     title('Y dir')
end

%% CPT



end