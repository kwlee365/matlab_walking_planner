function [cp_cotrolled, zmp_controlled] = CapturePointMPC(w, T, sim_tick, N, step_time, cp_preview, zmp_controlled, A, B, F_xi, F_p, Theta, e1, x_foot_temp, y_foot_temp)

% bias = linspace(1,10,N);
% for i = 1:1:N
%     Q(2*(i-1)+1:2*i, 2*(i-1)+1:2*i) = bias(i)*eye(2);
% end

Q = eye(2*N, 2*N);
R = 0.01 * eye(2*N,2*N);

H = Theta' * R * Theta + F_p' * Q * F_p;
g = F_p' * Q * (F_xi * cp_preview(1:2, 1) - cp_preview) - Theta' * R * e1* zmp_controlled(1:2,1);
% cp_preview(1:3, 2) : In the simulation, we have to insert the real CP values.
% zmp_controlled(1:2,1) : ZMP Control Input of the previous tick.

% ZMP constraints
Amax = eye(2*N, 2*N);
Amin = -1.0 * eye(2*N, 2*N);

for j = 1:1:N
    b_max(2*(j-1)+1,1) = + 0.16 + x_foot_temp(j,1);
    b_max(2*(j-1)+2,1) = + 0.13 + y_foot_temp(j,1);
    b_min(2*(j-1)+1,1) = + 0.14 - x_foot_temp(j,1); 
    b_min(2*(j-1)+2,1) = + 0.13 - y_foot_temp(j,1);
end


Aconst = [Amax;
          Amin];
bconst = [b_max(1:2*N,1);
          b_min(1:2*N,1)];

zmp_controlled = quadprog(H,g,Aconst,bconst);
cp_cotrolled = F_xi * cp_preview(1:2, 1) + F_p * zmp_controlled;
end
