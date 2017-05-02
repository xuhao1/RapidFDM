function [obj,out] = l1_filter_control(obj,eta)
obj.eta = eta;
[obj.u,obj.u_filter] = IterTransform(eta,obj.u_filter);
if obj.u_lead_enable
    [out,obj.u_lead] = IterTransform(obj.u,obj.u_lead);
else
    out = obj.u;
end
out = float_constrain(out - obj.km'*obj.x_real,-1,1);
end