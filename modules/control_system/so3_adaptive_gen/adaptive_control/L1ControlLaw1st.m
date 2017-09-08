function [obj,out] = L1ControlLaw1st(dt,obj)
omega = obj.x(3);
theta = obj.x(4:5);
sigma = obj.x(6);
if obj.Gamma > 1
    theta(2) = (obj.km(2)*obj.b(2) - obj.actuator_estimator.x(3)) / obj.b(2);
    theta(2) = float_constrain(theta(2),-10,10);
    obj.x(5) = theta(2);
    sigma = obj.actuator_estimator.x(6)/obj.b(2);
    obj.x(6) = sigma;
end

%Jb = obj.g_by_x(2,1:2) / obj.b(2);
%gb = obj.g(2,1) / obj.b(2);

eta = float_constrain((obj.kg_rate * obj.rdot + ...
    - sigma  - theta(2)* obj.x_real(2) )/omega,-1,1);
[obj,out] = l1_filter_control(dt,obj,eta);
end