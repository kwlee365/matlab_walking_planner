function [rot_yaw] = rotateZaxis(yaw)

c = cos(yaw);
s = sin(yaw);

rot_yaw = [c -s;
           s  c];

end

