function [obj,u] = L1AdaptiveControl1st(dt,obj,x_real,rdot)
obj.r = 0;
obj.rdot = rdot;

[obj.actuator_estimator,~] = IterActuatorEst(obj.actuator_estimator,x_real(2),obj.out,dt);

x_real(2) = obj.actuator_estimator.x(1);
obj.x_real = x_real;

if not(obj.inited)
    obj.x(1) = x_real(1);
    obj.x(2) = x_real(2);
    obj.inited = true;
    u = 0;
return
end


obj.x(1) = x_real(1);
    
if obj.fixed_step_ode
    if obj.ode_steps > 1
        subdt = dt/obj.ode_steps;
        [~,obj.x]=ode4user(@(t,x) L1_ODE_1st(t,x,obj),obj.t:subdt:obj.t+dt,obj.x);
    else
      [~,obj.x]=ode4user(@(t,x) L1_ODE_1st(t,x,obj),[obj.t,obj.t + dt],obj.x);
    end 
    obj.x = ctrl_x_constrain(obj.x,obj);
else
    opts_1 =  odeset('RelTol',1e-3,'AbsTol',1e-6);
    [~,x]=ode23(@(t,x) L1_ODE_1st(t,x,obj),...
        [obj.t,obj.t + dt],obj.x,opts_1);
    obj.x = ctrl_x_constrain(x(end,1:7)',obj);
end
    
[obj,out] = L1ControlLaw1st(dt,obj);

obj.err(1:2) = obj.x(1:2) - x_real(1:2);
obj.t = obj.t + dt;
u = out;