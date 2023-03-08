function [VRPxRef, VRPyRef, VRPzRef] = VrpGenerator(dt, sim_tick, ZxRef, ZyRef, CzTraj, w, w_dot,g)

VRPxRef = zeros(sim_tick,1);
VRPyRef = zeros(sim_tick,1);   
VRPzRef = zeros(sim_tick,1);

    for i = 1:1:sim_tick + 500

        VRPxRef(i,1) = ZxRef(i,1);
        VRPyRef(i,1) = ZyRef(i,1);
        if i <= sim_tick
            VRPzRef(i,1) = g / (w(i,1)^2 - w_dot(i,1));
        else
            VRPzRef(i,1) = VRPzRef(sim_tick,1);
        end
    end
end

