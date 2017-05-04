function [obj,u] = L1AdaptiveControl2nd(dt,obj,x_real,r)
obj.r = r;
u = 0;
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

if obj.fixed_step_ode
    if obj.ode_steps > 1
        subdt = dt/obj.ode_steps;
        [~,obj.x]=ode4user(@(t,x) L1_ODE_2nd(t,x,obj),obj.t:subdt:obj.t+dt,obj.x);
    else
      [~,obj.x]=ode4user(@(t,x) L1_ODE_2nd(t,x,obj),[obj.t,obj.t + dt],obj.x);
    end 
    obj.x = ctrl_x_constrain(obj.x);
else
opts_1 =  odeset('RelTol',1e-3,'AbsTol',1e-6);
[~,x]=ode23(@(t,x) L1_ODE_2nd(t,x,obj),...
    [obj.t,obj.t + dt],obj.x,opts_1);
obj.x = ctrl_x_constrain(x(end,1:7)');
end


[obj,u] = L1ControlLaw2nd(dt,obj);


obj.err(1:2) = obj.x(1:2) - x_real(1:2);
obj.t = obj.t + dt;