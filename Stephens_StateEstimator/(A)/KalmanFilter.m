%Copyright 2017, Louis Hawley and Wael Suleiman

function sys = KalmanFilter(sys)
   
   % Prediction step:
   sys.xhat = sys.A * sys.xhat       + sys.B*sys.u;
   sys.P    = sys.A * sys.P * sys.A' + sys.Q;

   % Kalman matrix gain:
   K = sys.P*sys.H'*inv(sys.H*sys.P*sys.H'+sys.R);

   % Correction step:
   sys.xhat = sys.xhat + K*(sys.yhat - sys.H*sys.xhat);
   sys.P    = sys.P    - K*sys.H*sys.P;

return