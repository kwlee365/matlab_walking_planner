function [com_desired_, dcm_desired_] = PreviewController_depracated(dt,sim_tick,preview_tick,VRPxRef,VRPyRef,VRPzRef,CzTraj,A,B,C,Gi,Gx,Gd,w,w_dot,wn,g,h)

    com_x_ = zeros(3,1);  % [c; cdot; cddot];
    com_y_ = zeros(3,1);  % [c; cdot; cddot];

    com_x_prev = zeros(3,1);  % [c; cdot; cddot];
    com_y_prev = zeros(3,1);  % [c; cdot; cddot];

    Ux = 0.0;
    Uy = 0.0;
    
    for i = 1:1:sim_tick
        sumGd_px = 0.0;
        sumGd_py = 0.0;

        vrp_ = [C*com_x_ ; 
                C*com_y_ ;
                g/(w(i)^2 - w_dot(i))];
    
        for j = 1:preview_tick
            sumGd_px = sumGd_px + Gd(j) * (VRPxRef(i + (j+1)) - VRPxRef(i + j));
            sumGd_py = sumGd_py + Gd(j) * (VRPyRef(i + (j+1)) - VRPyRef(i + j));
        end
    
        del_ux = -Gi*(vrp_(1) - VRPxRef(i,1)) - Gx*(com_x_ - com_x_prev) - sumGd_px;
        del_uy = -Gi*(vrp_(2) - VRPyRef(i,1)) - Gx*(com_y_ - com_y_prev) - sumGd_py;

        Ux = Ux + del_ux;
        Uy = Uy + del_uy;
    
        com_x_prev = com_x_;
        com_y_prev = com_y_;
    
        com_x_ = A * com_x_ + B * Ux;    
        com_y_ = A * com_y_ + B * Uy;
        
        com_desired_(i,1) = com_x_(1);
        com_desired_(i,2) = com_y_(1);
        com_desired_(i,3) = CzTraj(i);

        dcm_desired_(i,1) = com_x_(1) + com_x_(2)/ wn;
        dcm_desired_(i,2) = com_y_(1) + com_y_(2)/ wn;
        dcm_desired_(i,3) = CzTraj(i,1) + CzTraj(i,2)/ w(i,1);
        dcm_desired_(i,4) = CzTraj(i,1) + CzTraj(i,2)/ wn;

        vrp_desired_(i,1) = vrp_(1,1);
        vrp_desired_(i,2) = vrp_(2,1);
        vrp_desired_(i,3) = vrp_(3,1);
    end

%     figure()
%     plot([1:sim_tick]*dt,vrp_desired_(:,1), [1:sim_tick]*dt, VRPxRef(1:sim_tick,1))
%     title('X dir')
%     legend('vrp desired', 'vrp reference')
%     grid on
% 
%     figure()
%     plot([1:sim_tick]*dt,vrp_desired_(:,2), [1:sim_tick]*dt, VRPyRef(1:sim_tick,1))
%     title('Y dir')
%     legend('vrp desired', 'vrp reference')
%     grid on
% 
%     figure()
%     plot([1:sim_tick]*dt,vrp_desired_(:,3), [1:sim_tick]*dt, VRPzRef(1:sim_tick,1))
%     title('Z dir')
%     legend('vrp desired', 'vrp reference')
%     grid on
end