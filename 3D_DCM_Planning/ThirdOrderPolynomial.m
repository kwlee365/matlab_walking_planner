function [traj, traj_dot] = ThirdOrderPolynomial(time, time_ini, time_end, x_ini, x_end, x_ini_dot, x_end_dot)

time_diff = time_end - time_ini;

elapsed_time = time - time_ini;

a0 = x_ini;
a1 = x_ini_dot;
a2 = ( 3/(time_diff^2))*(x_end - x_ini) - (1/ time_diff)    *(2*x_ini_dot + x_end_dot);
a3 = (-2/(time_diff^3))*(x_end - x_ini) + (1/(time_diff^2)) *  (x_ini_dot + x_end_dot);

traj =     a0 + a1*elapsed_time + a2*elapsed_time^2 +   a3*elapsed_time^3;
traj_dot =      a1            + 2*a2*elapsed_time   + 3*a3*elapsed_time^2;

end