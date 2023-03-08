function [CzTraj, CzRef] = ComHeightGenerator(dt, sim_tick, h, height_increment);

for i = 1:1:sim_tick + 500

    time = i*dt;

    if time < 3;
        CzRef(i) = h;
        [a0,a1,a2,a3,a4,a5] = FifthCubicPolynomial(h, h, 0, 3);
        CzTraj(i,1) = a0 + a1*time + a2*time^2 + a3*time^3 +   a4*time^4+    a5*time^5;
        CzTraj(i,2) =      a1    + 2*a2*time + 3*a3*time^2 + 4*a4*time^3+  5*a5*time^4;
        CzTraj(i,3) =              2*a2      + 6*a3*time  + 12*a4*time^2+ 20*a5*time^3;
    elseif time < 4;
        CzRef(i) = h + height_increment;
        [a0,a1,a2,a3,a4,a5] = FifthCubicPolynomial(h, h + height_increment, 3, 4);
        CzTraj(i,1) = a0 + a1*time + a2*time^2 + a3*time^3 +   a4*time^4+    a5*time^5;
        CzTraj(i,2) =      a1    + 2*a2*time + 3*a3*time^2 + 4*a4*time^3+  5*a5*time^4;
        CzTraj(i,3) =              2*a2      + 6*a3*time  + 12*a4*time^2+ 20*a5*time^3;
    elseif time < 5;
        CzRef(i) = h + height_increment*2;
        [a0,a1,a2,a3,a4,a5] = FifthCubicPolynomial(h + height_increment, h + height_increment*2, 4, 5);
        CzTraj(i,1) = a0 + a1*time + a2*time^2 + a3*time^3 +   a4*time^4+    a5*time^5;
        CzTraj(i,2) =      a1    + 2*a2*time + 3*a3*time^2 + 4*a4*time^3+  5*a5*time^4;
        CzTraj(i,3) =              2*a2      + 6*a3*time  + 12*a4*time^2+ 20*a5*time^3;
    elseif time < 6;
        CzRef(i) = h + height_increment*3;
        [a0,a1,a2,a3,a4,a5] = FifthCubicPolynomial(h + height_increment*2, h + height_increment*3, 5, 6);
        CzTraj(i,1) = a0 + a1*time + a2*time^2 + a3*time^3 +   a4*time^4+    a5*time^5;
        CzTraj(i,2) =      a1    + 2*a2*time + 3*a3*time^2 + 4*a4*time^3+  5*a5*time^4;
        CzTraj(i,3) =              2*a2      + 6*a3*time  + 12*a4*time^2+ 20*a5*time^3;
    else
        CzRef(i) = h + height_increment*4;
        [a0,a1,a2,a3,a4,a5] = FifthCubicPolynomial(h + height_increment*3, h + height_increment*4, 6,7);
        CzTraj(i,1) = a0 + a1*time + a2*time^2 + a3*time^3 +   a4*time^4+    a5*time^5;
        CzTraj(i,2) =      a1    + 2*a2*time + 3*a3*time^2 + 4*a4*time^3+  5*a5*time^4;
        CzTraj(i,3) =              2*a2      + 6*a3*time  + 12*a4*time^2+ 20*a5*time^3;
        if time > 7
            CzTraj(i,1) = h + height_increment*4;
            CzTraj(i,2) = 0.0;
            CzTraj(i,3) = 0.0;
        end
    end


end

CzRef = CzRef';
end

