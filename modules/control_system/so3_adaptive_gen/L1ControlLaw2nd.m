function obj = L1ControlLaw2nd(dt,obj,r)
omega = obj.x(3);
theta = obj.x(4:5);
sigma = obj.x(6);
kg = obj.kg;
eta = float_constrain((kg*r - sigma - theta'* obj.x(1:2))/omega,-1,1);
[u,obj.u_filter] = IterTransform1st(eta,obj.u_filter);
obj.u  = u;
obj.eta = eta;
end