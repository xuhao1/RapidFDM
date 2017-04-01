function [obj,u] = L1AdaptiveControl2nd(dt,obj,x_real,r)
obj.r = r;
%obj.x(1) = DeltaSpX + obj.x(1);
obj.x_real = x_real;
if not(obj.inited)
    obj.x(1) = x_real(1);
    obj.x(2) = x_real(2);
    obj.inited = true;
    u = 0;
return
end


obj.err(1:2) = obj.x(1:2) - x_real(1:2);
%obj.g(1) = - rfb;

opts_1 = odeset('MaxStep',4);
[~,x]=ode23(@(t,x) L1Updater(t,x,obj),...
    [obj.t,obj.t + dt],obj.x,opts_1);

[obj,out] = L1ControlLaw2nd(dt,obj);
obj.x = ctrl_x_constrain(x(end,1:6)');
obj.err(1:2) = obj.x(1:2) - x_real(1:2);
obj.t = obj.t + dt;
u = out;