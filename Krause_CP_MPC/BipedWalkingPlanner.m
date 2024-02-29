%% CP-MPC

clc
close all
clear all

% Krause, Manuel, et al. "Stabilization of the capture point dynamics for bipedal walking based on model predictive control." 
% IFAC Proceedings Volumes 45.22 (2012): 165-171.

% Parameters
T = 0.01;                     % [s] : sampling time
hz = 1/T;
h = 0.75;                       % Desired com height
g = 9.81;                       % gravity acceleration
sim_time = 10;
sim_tick = sim_time*hz; 
w = sqrt(g/h);                  

step_time = 1.0;                % step time

step_length = 2.0;
step_width = 0.2;

preview_window = 2*step_time*hz;

Qe = 1.0;            % preview gain
Qx = zeros(3,3);     % preview gain
R = 1.0*10^(-6);

N = preview_window;
%% CP-MPC

[FootRef_x, FootRef_y] = FootStepGenerator(T, sim_tick, step_length, step_width);
FootRef = [FootRef_x FootRef_y];
[Gi,Gx,Gd,A,B,C] = PreviewParameter(T,h,g,N,Qe,Qx,R);
[cp_ref, zmp_ref] = PreviewController(T,sim_tick,N,FootRef_x,FootRef_y,A,B,C,Gi,Gx,Gd,w,g,h);

% Matrix manipulation
A =   exp(w*T);
B = 1-exp(w*T); 

F_xi = zeros(N, 1);
F_p  = zeros(N, N);
Theta = zeros(N, N);
e1 = zeros(N, 1);
e1(1,1) = 1;

for i = 1:1:N

    F_xi(i, 1) = A^i;
    Theta(i,i) = 1;

    for j = 1:1:N
        % (F_p)
        if(i < j)
            F_p(i,j) = 0;
        else;
            F_p(i,j) = A^(i-j)*B;
        end
        % (Theta)
        if(i == j+1)
            Theta(i,j) = -1;
        end
    end
end

current_step = 0;
j = 1;

%% MPC data
cp_x_ref_mpc = zeros(N, sim_tick);
lb_x = zeros(N, sim_tick);
ub_x = zeros(N, sim_tick);

cp_y_ref_mpc = zeros(N, sim_tick);
lb_y = zeros(N, sim_tick);
ub_y = zeros(N, sim_tick);

 
for i = 1:1:sim_tick
    cp_x_ref_mpc(1:N, i) = cp_ref(i:i+N-1, 1);
    cp_y_ref_mpc(1:N, i) = cp_ref(i:i+N-1, 2);

    lb_x(1:N, i) = FootRef_x(i:i+N-1, 1) - 1.3;
    ub_x(1:N, i) = FootRef_x(i:i+N-1, 1) + 1.7;

    lb_y(1:N, i) = FootRef_y(i:i+N-1, 1) - 1;
    ub_y(1:N, i) = FootRef_y(i:i+N-1, 1) + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DSP is not considered in this planner. So, inequality conditions are  % 
% dominant to plan the trajectories.                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Quadratic programming

cp_x_hat = zeros(1,sim_tick); % real value  
p_x_hat = zeros(1,sim_tick);
cp_x_opt = zeros(N, sim_tick);  % opt value
p_x_opt = zeros(N, sim_tick);

cp_y_hat = zeros(1,sim_tick); % real value  
p_y_hat = zeros(1,sim_tick);
cp_y_opt = zeros(N, sim_tick);  % opt value
p_y_opt = zeros(N, sim_tick);

Q = eye(N, N);
R = 0.00001 * eye(N, N);

for k=1:1:sim_tick

    % Cost function

    H = transpose(Theta) * R * Theta + transpose(F_p) * Q * F_p;
    
    if k == 2.5*hz            
        cp_y_hat(:,k) = cp_y_hat(:,k) + 0.08;
    end

    if k == 4.5*hz            
        cp_x_hat(:,k) = cp_x_hat(:,k) + 0.1;
    end

    if k == 1
        gx = transpose(F_p) * Q * (F_xi * cp_x_hat(1, k) - cp_x_ref_mpc(1:N,k)) - transpose(Theta) * R * e1 * p_x_hat(1,k);
        gy = transpose(F_p) * Q * (F_xi * cp_y_hat(1, k) - cp_y_ref_mpc(1:N,k)) - transpose(Theta) * R * e1 * p_y_hat(1,k);
    else
        gx = transpose(F_p) * Q * (F_xi * cp_x_hat(1, k) - cp_x_ref_mpc(1:N,k)) - transpose(Theta) * R * e1 * p_x_hat(1,k-1);
        gy = transpose(F_p) * Q * (F_xi * cp_y_hat(1, k) - cp_y_ref_mpc(1:N,k)) - transpose(Theta) * R * e1 * p_y_hat(1,k-1);
    end

    px_opt(1:N,k) = quadprog(H, gx,[],[],[],[], lb_x(1:N,k), ub_x(1:N,k));
    py_opt(1:N,k) = quadprog(H, gy,[],[],[],[], lb_y(1:N,k), ub_y(1:N,k));

    % cp_x_opt(1:N,k) = F_xi * cp_x_hat(1, k) + F_p * px_opt(1:N,k);
    % cp_y_opt(1:N,k) = F_xi * cp_x_hat(1, k) + F_p * py_opt(1:N,k);

    px_hat(1,k) = px_opt(1,k);
    py_hat(1,k) = py_opt(1,k);

    cp_x_hat(1,k+1) = cp_x_opt(1,k);
    cp_y_hat(1,k+1) = cp_y_opt(1,k);

    cp_x_hat(1,k+1) = A*cp_x_hat(1,k) + B*px_hat(1,k);
    cp_y_hat(1,k+1) = A*cp_y_hat(1,k) + B*py_hat(1,k);
end

%%
figure()
hold on
plot([1:sim_tick], cp_ref(1:sim_tick,1), [1:sim_tick], cp_x_hat(1,1:sim_tick)')
plot([1:sim_tick], FootRef_x(1:sim_tick,1), [1:sim_tick], px_hat(1,1:sim_tick)')
legend('cp ref', 'cp mpc','p ref', 'p mpc')
title('x')
ylim([-2 10])


figure()
hold on
plot([1:sim_tick], cp_ref(1:sim_tick,2), [1:sim_tick], cp_y_hat(1,1:sim_tick)')
plot([1:sim_tick], FootRef_y(1:sim_tick,1), [1:sim_tick], py_hat(1,1:sim_tick)')
legend('cp ref', 'cp mpc','p ref', 'p mpc')
title('y')
ylim([-2 2])