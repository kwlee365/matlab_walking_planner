function [pRef, fRef] = FootStepGenerator(number_of_step, step_length, step_width, L_or_R)

% FR = [x, y, yaw]
% FL = [x, y, yaw]

pRef = zeros(3, number_of_step + 3);
fRef = zeros(3, number_of_step + 3);

y_target   = 0.2;
yaw_target = deg2rad(20);
y_pert     = zeros(number_of_step);
yaw_pert   = zeros(number_of_step);

% for i = 1:1:number_of_step
% 
%     if i <= number_of_step / 2.0
%         lin_interpol = (i-1) / (number_of_step / 2.0 - 1)
% 
%         y_pert(i)    = (1 - lin_interpol) * 0.0 + lin_interpol * 0.0; 
%         % yaw_pert(i)  = (1 - lin_interpol) * 0.0 + lin_interpol * yaw_target;
%     else
%         lin_interpol = (i-(number_of_step / 2.0) - 1) / (number_of_step - number_of_step / 2.0 - 1)
% 
%         % y_pert(i)    = (1 - lin_interpol) * 0.0 + lin_interpol * y_target; 
%         y_pert(i) = y_target;
%         % yaw_pert(i)  = (1 - lin_interpol) * 0.0 + lin_interpol * -0.0;
% 
%     end
% end
% 

yaw_pert(5)  = yaw_target;
yaw_pert(6)  = yaw_target;
yaw_pert(7)  = yaw_target
yaw_pert(8)  = yaw_target;



% figure()
% plot(y_pert)
% figure()
% plot(yaw_pert)

for i = 1:1:number_of_step + 3

    if i == 1
        pRef(:,i) = zeros(3, 1);
        fRef(:,i) = zeros(3, 1) + [0.0;         0.5 * L_or_R * step_width;0.0];
    elseif i == 2
        pRef(:,i) = pRef(:,i-1) + [0.0;         0.5 * L_or_R * step_width;0.0];
        fRef(:,i) = pRef(:,i-1) + [0.0;         0.5 * L_or_R * step_width;0.0];
    elseif i == number_of_step + 3
        pRef(:,i) = pRef(:,i-1) + [0.0; 0.0; yaw_pert(i)];
        fRef(:,i) = pRef(:,i-1) + [0.0; 0.0; yaw_pert(i)];

        pRef([1:2],i) = pRef([1:2],i-1) + rotateZaxis(pRef(3,i)) * [0.0; 0.5 * L_or_R * step_width]; 
        fRef([1:2],i) = fRef([1:2],i-1) + rotateZaxis(fRef(3,i)) * [0.0; 1.0 * L_or_R * step_width]; 
    else
        pRef(:,i) = pRef(:,i-1) + [0.0; 0.0; yaw_pert(i)];
        fRef(:,i) = pRef(:,i-1) + [0.0; 0.0; yaw_pert(i)];

        pRef([1:2],i) = pRef([1:2],i-1) + rotateZaxis(pRef(3,i)) * [step_length; L_or_R * step_width]; 
        fRef([1:2],i) = fRef([1:2],i-1) + rotateZaxis(fRef(3,i)) * [step_length; L_or_R * step_width]; 
    end
    
    L_or_R = (-1) * L_or_R;
end

end