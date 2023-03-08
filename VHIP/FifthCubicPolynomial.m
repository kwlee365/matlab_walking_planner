function [a0, a1, a2, a3, a4, a5] = FifthCubicPolynomial(zi, zf, ti, tf)

A = [1 ti ti^2 ti^3 ti^4 ti^5;
     0 1 2*ti 3*ti^2 4*ti^3 5*ti^4;
     0 0 2 6*ti 12*ti^2 20*ti^3;
     1 tf tf^2 tf^3 tf^4 tf^5;
     0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
     0 0 2 6*tf 12*tf^2 20*tf^3];

b = [zi;0;0;zf;0;0];

ans = inv(A)*b;

a0 = ans(1);
a1 = ans(2);
a2 = ans(3);
a3 = ans(4);
a4 = ans(5);
a5 = ans(6);
end