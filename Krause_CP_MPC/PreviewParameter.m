function [Gi,Gx,Gd,A,B,C] = PreviewParameter(dt,h,g,preview_tick,Qe,Qx,R)
    
    %% Discretized cart table model
    A = [1  dt  dt*dt/2;
         0  1   dt;
         0  0   1];
    
    B = [0;0;1];
    
    C = [1 0 -h/g];
    
    %% Discrete riccati equation
    
    Bbar = [C*B;B];
    Ibar = [1;0;0;0];
    Fbar = [C*A;A];
    Qbar = [Qe zeros(1,3);
            zeros(3,1) Qx];
    Abar = [Ibar Fbar];
    
    Kbar = dare(Abar,Bbar,Qbar,R);
    
    INV_mat = inv(R + Bbar'*Kbar*Bbar);
    
    Gi = INV_mat * Bbar' * Kbar * Ibar;
    Gx = INV_mat * Bbar' * Kbar * Fbar;
    
    Acbar = Abar - Bbar*INV_mat*Bbar'*Kbar*Abar;
    
    Xbar(1:4,1) = -Acbar'*Kbar*Ibar;
    Gd(1,1) = -Gi;
    
    for i = 2:preview_tick
        Xbar(1+4*(i-1):4*(i))= Acbar'*Xbar(1+4*(i-2):4*(i-1));
        Gd(i,1) = INV_mat*Bbar'*Xbar(1+4*(i-2):4*(i-1));
    end

end