clc
clear all
close all

%% Ref

% min  w1 * ||CoM jerk||^2 + w2 * ||CoM vel||^2  + w3 * ||C - C_ref||^2
% u = CoM jerk

%% Parameter setting

T = 0.05;                      % [s] : sampling time
hz = 1/T;
h = 0.75;                       % Desired com height
g = 9.81;                       % gravity acceleration
sim_time = 10;
sim_tick = sim_time*hz; 
wn = sqrt(g/h);                 

step_time = 1.0;                % step time
step_tick = step_time*hz;
preview_time = 2*step_time;
N = preview_time*hz;            % 3 step preview
m = 3;

step_length = 0.2;
step_width = 0.1;

%% Reference ZMP

[ZxRef, ZyRef, VxRef, VyRef] = ZmpGenerator(T, sim_tick, step_length, step_width, step_time);

%% State space representation

A = [1 T T^2/2;
     0 1 T;
     0 0 1];

B = [T^3/6;
     T^2/2;
     T];

C = [1 0 -h/g];

%% Matrix manipulation

for i = 1:1:N
    P_ps(i,1) = 1;;
    P_ps(i,2) = i*T;
    P_ps(i,3) = (i*T)^2/2;

    P_vs(i,1) = 0;
    P_vs(i,2) = 1;
    P_vs(i,3) = i*T;

    P_as(i,1) = 0;
    P_as(i,2) = 0;
    P_as(i,3) = 1;

    P_zs(i,1) = 1;;
    P_zs(i,2) = i*T;
    P_zs(i,3) = (i*T)^2/2 - h/g;

    for j = 1:1:N
        if(i < j)
            P_pu(i,j) = 0.0;
            P_vu(i,j) = 0.0;
            P_zu(i,j) = 0.0;
        else % (i >= j)
            P_pu(i,j) = (1 + 3*(i-j) + 3*(i-j)^2) * T^3 / 6;
            P_vu(i,j) = (1 + 2*(i-j)) * T^2 / 2;
            P_zu(i,j) = (1 + 3*(i-j) + 3*(i-j)^2) * T^3 / 6 - T*h/g;
        end
    end
end

%% Cost function

w1 = 1e-2;  % CoM jerk regulation
w2 = 1e-1;  % CoM velocity regulation
w3 = 1;     % CoM position tracking
w4 = 0;     % CoM velocity tracking

%% Quadratic Programming

x_hat = zeros(3,sim_tick);  % store initial values
y_hat = zeros(3,sim_tick);

Q = zeros(N, N)
px = zeros(N);
py = zeros(N)

ux = zeros(N);
uy = zeros(N);

Ax_max = zeros(N, N);
Ax_min = zeros(N, N);

current_step_num = 0;

for k = 1:1:sim_tick
    % Step matrix
    if k == 4.5*hz            
            ZyRef(k+(0.5*hz):k+(1.5*hz-1)) = ZyRef(k+(0.5*hz):k+(1.5*hz-1)) - 0.1;
    end

    % Hessian
    Q(1:N,1:N) = w1 * eye(N,N) + w2 * P_vu'*P_vu + w3 * P_pu'*P_pu + w4 * P_vu'*P_vu;

    % Gradient
    px(1:N,k)  =  w2 * P_vu' * P_vs * x_hat(:,k) + w3 * P_pu' * (P_ps * x_hat(:,k) - ZxRef((k+1:k+N),:)) + w4 * P_vu' * (P_vs * x_hat(:,k) - VxRef((k+1:k+N),:));
    py(1:N,k)  =  w2 * P_vu' * P_vs * y_hat(:,k) + w3 * P_pu' * (P_ps * y_hat(:,k) - ZyRef((k+1:k+N),:)) + w4 * P_vu' * (P_vs * y_hat(:,k) - VyRef((k+1:k+N),:));;

    % Inequality constraint
    Ax_max = [P_zu];
    bx_max(1:N,k) = + 0.05 + ZxRef((k+1:k+N),:) - P_zs*x_hat(:,k);
    Ax_min = [-P_zu];
    bx_min(1:N,k) = + 0.05 - ZxRef((k+1:k+N),:) + P_zs*x_hat(:,k);

    Ay_max = [P_zu];
    by_max(1:N,k) = + 0.05 + ZyRef((k+1:k+N),:) - P_zs*y_hat(:,k);
    Ay_min = [-P_zu];
    by_min(1:N,k) = + 0.05 - ZyRef((k+1:k+N),:) + P_zs*y_hat(:,k);

    Ax_const = [Ax_max;
                Ax_min];
    bx_const(1:2*N,k) = [bx_max(1:N,k);
                         bx_min(1:N,k)];
    Ay_const = [Ay_max;
                Ay_min];
    by_const(1:2*N,k) = [by_max(1:N,k);
                         by_min(1:N,k)];

    % QP
    ux(1:N,k) = quadprog(Q, px(:,k), Ax_const, bx_const(1:2*N,k));
    uy(1:N,k) = quadprog(Q, py(:,k), Ay_const, by_const(1:2*N,k));

    x_hat(:,k+1) = A*x_hat(:,k) + B*ux(1,k);
    y_hat(:,k+1) = A*y_hat(:,k) + B*uy(1,k);

    % Step change
    if(rem(k+1, step_tick) == 0)
        current_step_num = current_step_num + 1;
    end
end

%% Graph

figure()
hold on;
plot(T*[1:sim_tick],ZxRef([1:sim_tick]));
plot(T*[1:sim_tick],C*x_hat(1:3,[1:sim_tick]));
plot(T*[1:sim_tick],x_hat(1,[1:sim_tick]));
plot(T*[1:sim_tick],ZxRef([1:sim_tick])-0.05,'k--');
plot(T*[1:sim_tick],ZxRef([1:sim_tick])+0.05,'k--');
title('X')
legend('ZMP Ref', 'ZMP act', 'CoM','ZMP min', 'ZMP max')
figure()
hold on;
plot(T*[1:sim_tick],ZyRef([1:sim_tick]));
plot(T*[1:sim_tick],C*y_hat(1:3,[1:sim_tick]));
plot(T*[1:sim_tick],y_hat(1,[1:sim_tick]));
plot(T*[1:sim_tick],ZyRef([1:sim_tick])-0.05,'k--');
plot(T*[1:sim_tick],ZyRef([1:sim_tick])+0.05,'k--');
ylim([-0.3 0.3])
title('Y')
legend('ZMP Ref', 'ZMP act', 'CoM','ZMP min', 'ZMP max')