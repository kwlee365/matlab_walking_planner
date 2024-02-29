function [com_desired_, dcm_desired_, x_hat, y_hat, zmp_x_hat, zmp_y_hat, ZyRef] = PreviewController(dt,sim_tick,preview_tick,ZxRef,ZyRef,A,B,C,Gi,Gx,Gd,wn,g,h);

    hz = 1/dt

    com_x_ = zeros(3,1);  % [c; cdot; cddot];
    com_y_ = zeros(3,1);  % [c; cdot; cddot];
    
    com_x_prev = zeros(3,1);  % [c; cdot; cddot];
    com_y_prev = zeros(3,1);  % [c; cdot; cddot];
   
    Ux = 0.0;
    Uy = 0.0;

    x_hat = zeros(3, sim_tick)
    y_hat = zeros(3, sim_tick)

    zmp_x_hat = zeros(1, sim_tick)
    zmp_y_hat = zeros(1, sim_tick)

    for i = 1:1:sim_tick

        if i == 4.5*hz            
            ZyRef(i+(0.5*hz):i+(1.5*hz-1)) = ZyRef(i+(0.5*hz):i+(1.5*hz-1)) - 0.6;
        end

        sumGd_px = 0.0;
        sumGd_py = 0.0;

        zmp_ = [C*com_x_ ; 
                C*com_y_ ];

        zmp_x_hat(1,i) = zmp_(1);
        zmp_y_hat(1,i) = zmp_(2);

        for j = 1:preview_tick
            sumGd_px = sumGd_px + Gd(j) * (ZxRef(i + (j+1)) - ZxRef(i + j));
            sumGd_py = sumGd_py + Gd(j) * (ZyRef(i + (j+1)) - ZyRef(i + j));
        end
    
        del_ux = -Gi*(zmp_(1) - ZxRef(i,1)) - Gx*(com_x_ - com_x_prev) - sumGd_px;
        del_uy = -Gi*(zmp_(2) - ZyRef(i,1)) - Gx*(com_y_ - com_y_prev) - sumGd_py;

        Ux = Ux + del_ux;
        Uy = Uy + del_uy;
    
        com_x_prev = com_x_;
        com_y_prev = com_y_;
    
        com_x_ = A * com_x_ + B * Ux;    
        com_y_ = A * com_y_ + B * Uy;

        x_hat(:,i) = com_x_;
        y_hat(:,i) = com_y_;
        
        com_desired_(i,1) = com_x_(1);
        com_desired_(i,2) = com_y_(1);

        dcm_desired_(i,1) = com_x_(1) + com_x_(2)/ wn;
        dcm_desired_(i,2) = com_y_(1) + com_y_(2)/ wn;

        zmp_desired_(i,1) = zmp_(1,1);
        zmp_desired_(i,2) = zmp_(2,1);
    end
end