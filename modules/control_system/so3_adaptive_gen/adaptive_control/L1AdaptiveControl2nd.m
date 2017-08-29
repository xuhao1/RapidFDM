function [obj,u] = L1AdaptiveControl2nd(dt,obj,x_real,r)
obj.r = r;

[obj.g(1),obj.g_filter] = IterTransform(obj.g(1),obj.g_filter);

[obj.actuator_estimator,~] = IterActuatorEst(obj.actuator_estimator,x_real(2),obj.out,dt);

%x_real(2) = obj.actuator_estimator.x(1);
obj.x_real = x_real;

if not(obj.inited)
    obj.x(1) = x_real(1);
    obj.x(2) = x_real(2);
    obj.inited = true;
    u = 0;
return
end


obj.err(1:2) = obj.x(1:2) - x_real(1:2);

obj.x=ode4user(@(t,x) L1_ODE_2nd(t,x,obj),[obj.t,obj.t + dt],obj.x,obj.ode_steps);
obj.x = ctrl_x_constrain(obj.x,obj);
%     [~,x]=ode45(@(t,x) L1_ODE_2nd(t,x,obj),...
%         [obj.t,obj.t + dt],obj.x);
%     obj.x = ctrl_x_constrain(x(end,1:7)',obj);

[obj,u] = L1ControlLaw2nd(dt,obj);


obj.err(1:2) = obj.x(1:2) - x_real(1:2);
obj.t = obj.t + dt;