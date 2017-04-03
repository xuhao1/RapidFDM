function obj = L1ControllerUpdateParams(obj,p,epi,b2,gamma,lag_fc,lag_alpha,lead_fc,lead_alpha)
%obj = init_adaptive_controller();
pd = 2*epi*sqrt(p/b2);
Am = [0 0;0 0];
Am(1,2) = 1.0;
Am(2,1) = - p *b2;
Am(2,2) = - pd *b2;

P = [0 0;0 0];
P(1,1) = ((Am(2,1) - 1) * Am(2,1) + Am(2,2) * Am(2,2)) / (2*Am(2,1)*Am(2,2));
P(2,1) = - (1/(2*Am(2,1)));
P(1,2) = - (1/(2*Am(2,1)));
P(2,2) = (1 - Am(2,1)) / (2*Am(2,1)*Am(2,2));



%filter_obj = 
obj.Gamma = gamma;
obj.P = P;
obj.Am = Am;
obj.b = [0;b2];
obj.g_filter = make_filter_obj(200,15);
obj.kg = - Am(2,1) / b2;
obj.kg_rate = - Am(2,2)/ b2;
if not(obj.inited)
obj.x(4) = p;
obj.x(5) = pd;
end
%obj.u_filter = make_filter_obj(200,fc);
obj.u_filter = make_lag_obj(lag_fc,lag_alpha);
obj.u_lead = make_lead_obj(lead_fc,lead_alpha);
obj.km = [p;pd];
end
