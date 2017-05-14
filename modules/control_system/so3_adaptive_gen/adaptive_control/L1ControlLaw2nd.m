function [obj,out] = L1ControlLaw2nd(dt,obj)
omega = obj.x(3);
%omega = obj.actuator_estimator.x(5)/obj.b(2);


theta = obj.x(4:5);
theta(2) = (obj.km(2) -  obj.actuator_estimator.x(3)/obj.actuator_estimator.x(5))*omega;
if theta(2) < 0
    theta(2) = 0;
end

obj.x(5) = theta(2);

sigma = obj.actuator_estimator.x(6)/obj.actuator_estimator.x(5)*omega;
obj.x(6) = sigma;

sigma_1nd = obj.x(7);
kgr = obj.kg_rate;

Jb = obj.g_by_x(2,1:2) / obj.b(2);
Jd = obj.g_by_x(1,1:2);

gb = obj.g(2,1) / obj.b(2);
gd = obj.g(1,1);
etad =  - kgr*(gd + Jd*obj.x(1:2));
%obj.eta = float_constrain((obj.kg * obj.r + kgr*rfb - sigma - gb - (theta'+Jb)* obj.x(1:2) + etad  )/omega,-1,1);
eta = float_constrain((obj.kg * obj.r + obj.kg_rate * obj.rdot + etad ...
    - sigma  - (theta')* obj.x_real(1:2) )/omega,-1,1);
[obj,out] = l1_filter_control(dt,obj,eta);
end