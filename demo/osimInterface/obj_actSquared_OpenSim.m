function [dObj, dObjGrad] = obj_actSquared_OpenSim(u)
% [dObj, dObjGrad] = obj_actSquared_OpenSim(u)
%
% This function computes the torque-squared objective function and its
% gradients.
%

if nargout == 1 % numerical gradients
    
    dObj = sum(u.^2);
    
% else  %Analytic gradients
%     
%     [dObj,fz,fzi] = autoGen_obj_torqueSquared(u(1,:),u(2,:),u(3,:),u(4,:),u(5,:));
%     dObjGrad = zeros(16,length(dObj));  % 16 = 1 + 5 + 5 + 5 = time + angle + rate + control
%     dObjGrad(fzi,:) = fz;
%     
% end

end