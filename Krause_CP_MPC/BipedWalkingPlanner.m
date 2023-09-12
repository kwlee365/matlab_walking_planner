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

preview_window = 0.8*step_time*hz;

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
A =    exp(w*T)  * eye(2);
B = (1-exp(w*T)) * eye(2); 

F_xi = zeros(2*N, 2);
F_p  = zeros(2*N, 2*N);
Theta = zeros(2*N, 2*N);
e1 = zeros(2*N, 2);
e1(1:2,1:2) = eye(2);

for i = 1:1:N

    F_xi(2*(i-1)+1:2*i, 1:2) = A^i;
    Theta(2*(i-1)+1:2*i, 2*(i-1)+1:2*i) = eye(2);

    for j = 1:1:N
        % (F_p)
        if(i < j)
            F_p(2*(i-1)+1:2*i, 2*(j-1)+1:2*j) = zeros(2,2);
        else
            F_p(2*(i-1)+1:2*i, 2*(j-1)+1:2*j) = A^(i-j)*B;
        end
        % (Theta)
        if(i == j+1)
            Theta(2*(i-1)+1:2*i, 2*(j-1)+1:2*j) = -eye(2);
        end
    end
end

current_step = 0;
j = 1;

%% MPC data
cp_ref_mpc = zeros(2*sim_tick, 1);
Foot_ref_mpc = zeros(2*sim_tick, 1);
b_max = zeros(2*N, sim_tick);
b_min = zeros(2*N, sim_tick);

for i = 1:1:sim_tick
    for j = 1:1:N
        cp_ref_mpc(2*(j-1)+1,i) = cp_ref(i+j-1,1);
        cp_ref_mpc(2*(j-1)+2,i) = cp_ref(i+j-1,2);  
        
        % x
        b_max(2*(j-1)+1,i) = FootRef_x(i+j-1,1) + 0.16;
        b_min(2*(j-1)+1,i) = - FootRef_x(i+j-1,1) + 0.14;
        
        % y
        b_max(2*(j-1)+2,i) = FootRef_y(i+j-1,1) + 0.1;
        b_min(2*(j-1)+2,i) = - FootRef_y(i+j-1,1) + 0.1;
    end
end

%% Quadratic programming

cp_hat = zeros(2,sim_tick); % real value  
p_hat = zeros(2,sim_tick);
cp_opt = zeros(2*N, sim_tick);  % opt value
p_opt = zeros(2*N, sim_tick);

for k=1:1:sim_tick

    % Cost function
    Q = eye(2*N, 2*N);
    R = 0.1 * eye(2*N, 2*N);
    H = Theta' * R * Theta + F_p' * Q * F_p;
    if k == 1
        g = F_p' * Q * (F_xi * cp_hat(1:2, k) - cp_ref_mpc(1:2*N,k)) - Theta' * R * e1 * p_hat(1:2,k);
    else
        g = F_p' * Q * (F_xi * cp_hat(1:2, k) - cp_ref_mpc(1:2*N,k)) - Theta' * R * e1 * p_hat(1:2,k-1);
    end

    % Constraints
    Amax = eye(2*N, 2*N);
    Amin = -1.0 * eye(2*N, 2*N);

    Aconst = [Amax;
              Amin];

    bconst(1:4*N,k) = [b_max(1:2*N,k);
                       b_min(1:2*N,k)];

    p_opt(1:2*N,k) = quadprog(H, g, Aconst, bconst(1,4*N,k));
    % p_opt(1:2*N,k) = quadprog(H, g,[],[],[],[], b_min(1:2*N,k), b_max(1:2*N,k));
    cp_opt(1:2*N,k+1) = F_xi * cp_hat(1:2, k) + F_p * p_opt(1:2*N,k);
    
    p_hat(1:2,k) = p_opt(1:2,k);
    cp_hat(1:2,k+1) = A*cp_hat(1:2,k) + B*p_hat(1:2,k);
end

%%
figure()
plot([1:sim_tick], cp_ref(1:sim_tick,1), [1:sim_tick], cp_hat(1,1:sim_tick)')
legend('cp ref', 'cp mpc')
title('x')
ylim([-2 10])

figure()
plot([1:sim_tick], cp_ref(1:sim_tick,2), [1:sim_tick], cp_hat(2,1:sim_tick)')
legend('cp ref', 'cp mpc')
title('y')
ylim([-2 2])

figure()
plot([1:sim_tick], FootRef_x(1:sim_tick,1), [1:sim_tick], p_hat(1,1:sim_tick)')
legend('p ref', 'p mpc')
title('x')
ylim([-2 10])

figure()
plot([1:sim_tick], FootRef_y(1:sim_tick,1), [1:sim_tick], p_hat(2,1:sim_tick)')
legend('p ref', 'p mpc')
title('y')
ylim([-2 2])
