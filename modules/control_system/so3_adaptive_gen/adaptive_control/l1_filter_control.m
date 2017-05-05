function [obj,out] = l1_filter_control(dt,obj,eta)
obj.eta = eta;
[obj.u,obj.u_filter] = IterTransform(eta,obj.u_filter);
% if obj.u_lead_enable
%     [out,obj.u_lead] = IterTransform(obj.u,obj.u_lead);
% else
%     out = obj.u;
% end
[obj.actuator_estimator,ys] = IterActuatorEst(obj.actuator_estimator,obj.x_real(2),obj.out,dt);
out = obj.u + float_constrain((obj.u - ys) * obj.p_actuator,-0.1,0.1);
obj.out = out;
end