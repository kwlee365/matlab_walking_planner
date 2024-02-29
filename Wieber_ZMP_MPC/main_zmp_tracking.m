clc
clear all
close all

%% Ref

% min  w1 * ||u||^2 + w2 * ||Z - Z_ref||^2
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
preview_time = 2*step_time;
N = preview_time*hz;            % 3 step preview

step_length = 0.1;
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

w1 = 1e-6;
w2 = 1;

Q_prime = w1*eye(N,N) + w2*P_zu'*P_zu;
Q = [Q_prime zeros(N,N);
     zeros(N,N) Q_prime];

%% Quadratic Programming

x_hat = zeros(3,sim_tick);  % store initial values
y_hat = zeros(3,sim_tick);
u = zeros(2*N, sim_tick);
p = zeros(2*N, sim_tick);

for k = 1:1:sim_tick

    if k == 4.5*hz            
            ZyRef(k+(0.5*hz):k+(1.5*hz-1)) = ZyRef(k+(0.5*hz):k+(1.5*hz-1)) - 0.2;
    end
    
    pk(1:2*N,k) = [w2 * P_zu' * (P_zs*x_hat(1:3,k) - ZxRef((k+1:k+N),1));
                   w2 * P_zu' * (P_zs*y_hat(1:3,k) - ZyRef((k+1:k+N),1))];

    % ZMP constraints
    Amax = [P_zu zeros(N,N); zeros(N,N) P_zu];
    bx_max(1:N,k) = + 0.05 + ZxRef((k+1:k+N),:) - P_zs*x_hat(:,k);
    by_max(1:N,k) = + 0.05 + ZyRef((k+1:k+N),:) - P_zs*y_hat(:,k);

    Amin = [-P_zu zeros(N,N); zeros(N,N) -P_zu];
    bx_min(1:N,k) = + 0.05 - ZxRef((k+1:k+N),:) + P_zs*x_hat(:,k); 
    by_min(1:N,k) = + 0.05 - ZyRef((k+1:k+N),:) + P_zs*y_hat(:,k);

    Aconst = [Amax;
              Amin];
    bconst(1:4*N,k) = [bx_max(1:N,k);
                       by_max(1:N,k);
                       bx_min(1:N,k);
                       by_min(1:N,k)];

    u(1:2*N,k) = quadprog(Q, pk(:,k),Aconst,bconst(1:4*N,k));

    x_hat(:,k+1) = A*x_hat(:,k) + B*u(1,k);
    y_hat(:,k+1) = A*y_hat(:,k) + B*u(N+1,k);
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
title('Y')
legend('ZMP Ref', 'ZMP act', 'CoM','ZMP min', 'ZMP max')