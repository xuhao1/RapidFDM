function [obj,out] = l1_filter_control(dt,obj,eta)
obj.eta = eta;
[obj.u,obj.u_filter] = IterTransform(eta,obj.u_filter);
obj.u = float_constrain(obj.u - obj.km' * obj.x_real / obj.x(3),-1,1);
ys = obj.actuator_estimator.actuator_real;
out = obj.u + float_constrain((obj.u - ys) * obj.p_actuator,-0.15,0.15);
obj.out = out;
end