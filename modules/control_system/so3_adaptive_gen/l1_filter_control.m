function [obj,out] = l1_filter_control(obj,eta)
obj.eta = eta;
[obj.u,obj.u_filter] = IterTransform1st(eta,obj.u_filter);
[out,obj.u_lead] = IterTransform1st(obj.u,obj.u_lead);
out = float_constrain(out,-1,1);
end