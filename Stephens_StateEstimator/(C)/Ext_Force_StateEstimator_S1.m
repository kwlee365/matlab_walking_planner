function sys = Ext_Force_StateEstimator_S1(iter, x, sys)

global g h w T

if iter == 1
    
    sys.A = [1 T 0 0; 
             w^2*T 1 -w^2*T T; 
             0 0 1 0;
             0 0 0 1];
    sys.B = [0;0;T;0];
    sys.H = [1 0 0 0];
    
    sys.sigma_com    = 1e-8;
    sys.sigma_comdot = 1e-4;
    sys.sigma_cop    = 1e-4;
    sys.sigma_extforce = 1e-4;
    
    sys.Q = [sys.sigma_com 0 0 0;
             0 sys.sigma_comdot 0 0;
             0 0 sys.sigma_cop 0;
             0 0 0 sys.sigma_extforce];
    
    sys.R = 1e-5; 
    sys.P = 1e2*eye(4);

    sys.u = 0;
    sys.B = zeros(4,1); % only estimation

    sys.xhat = x;
    % sys.yhat
    
end
    sys(end).yhat = x(1,1);  % com offset
    sys(end+1) = KalmanFilter(sys(end));  
    
end

