function [obj,u] = L1AdaptiveControl1st(dt,obj,x_real,rdot)
obj.r = 0;
obj.rdot = rdot;
obj.x_real = x_real;
if not(obj.inited)
    obj.x(1) = x_real(1);
    obj.x(2) = x_real(2);
    obj.inited = true;
    u = 0;
return
end


obj.err(1:2) = obj.x(1:2) - x_real(1:2);
obj.x(1) = x_real(1);

if obj.fixed_step_ode
    if obj.ode_steps > 1
    tspan = obj.t:1/obj.ode_steps:obj.t+dt;
    else
        tspan = [obj.t,obj.t + dt];
    end
    [~,x]=ode4user(@(t,x) L1_ODE_1st(t,x,obj),tspan,obj.x);
else
opts_1 =  odeset('RelTol',1e-3,'AbsTol',1e-6);
[~,x]=ode23(@(t,x) L1_ODE_1st(t,x,obj),...
    [obj.t,obj.t + dt],obj.x,opts_1);
end
    
[obj,out] = L1ControlLaw1st(dt,obj);
obj.x = ctrl_x_constrain(x(end,1:7)');
obj.x(1) = x_real(1);
obj.err(1:2) = obj.x(1:2) - x_real(1:2);
obj.t = obj.t + dt;
u = out;