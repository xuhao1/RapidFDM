function [obj,out] = L1ControlLaw2nd(dt,obj,rfb)
omega = obj.x(3);
theta = obj.x(4:5);
sigma = obj.x(6);
kgr = obj.kg_rate;

Jb = obj.g_by_x(2,1:2) / obj.b(2);
Jd = obj.g_by_x(1,1:2) / obj.b(2);
gb = obj.g(2,1) / obj.b(2);
gd = obj.g(1,1) / obj.b(2);
etad =  - kgr*(gd+Jd*obj.x(1:2));
%obj.eta = float_constrain((obj.kg * obj.r + kgr*rfb - sigma - gb - (theta'+Jb)* obj.x(1:2) + etad  )/omega,-1,1);
obj.eta = float_constrain((obj.kg * obj.r - sigma  - (theta')* obj.x_real(1:2) )/omega,-1,1);
[obj.u,obj.u_filter] = IterTransform1st(obj.eta,obj.u_filter);
out = obj.u;%- obj.km'*obj.x(1:2);
end