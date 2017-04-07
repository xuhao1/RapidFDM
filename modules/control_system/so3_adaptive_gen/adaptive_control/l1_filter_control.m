function [obj,out] = l1_filter_control(obj,eta)
obj.eta = eta;
[obj.u,obj.u_filter] = IterTransform(eta,obj.u_filter);
[out,obj.u_lead] = IterTransform(obj.u,obj.u_lead);
out = float_constrain(out - obj.km'*obj.x_real,-1,1);
end