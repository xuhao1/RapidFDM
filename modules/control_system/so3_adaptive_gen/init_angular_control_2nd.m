function obj = init_angular_control_2nd(am21,am22,gamma,lag_fc,lag_alpha)
obj = init_adaptive_controller();
Am = [0 0;0 0];
Am(1,2) = 1.0;
Am(2,1) = - am21;
Am(2,2) = - am22;

P = [0 0;0 0];
P(1,1) = ((Am(2,1) - 1) * Am(2,1) + Am(2,2) * Am(2,2)) / (2*Am(2,1)*Am(2,2));
P(2,1) = - (1/(2*Am(2,1)));
P(1,2) = - (1/(2*Am(2,1)));
P(2,2) = (1 - Am(2,1)) / (2*Am(2,1)*Am(2,2));


b2 = 1 / 0.06;
%filter_obj = 
obj.Gamma = gamma;
obj.P = P;
obj.Am = Am;
obj.b = [0;b2];
obj.g_filter = make_lag_obj(10,15);
obj.kg = - Am(2,1) / b2;
obj.kg_rate = - Am(2,2)/ b2;
if not(obj.inited)
obj.x(4) = 13.3;
%obj.x(5) = pd;
end
obj.u_filter = make_lag_obj(lag_fc,lag_alpha);

obj.u_lead = make_lead_obj(1,1);
%obj.km = [p;pd];
end