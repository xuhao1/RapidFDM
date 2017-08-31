function obj = L1ControllerUpdateParams(obj,p,pd,b2,gamma,lag_fc,lag_alpha,p_actuator,ekf_p_noise)
%obj = init_adaptive_controller();
%pd = 2*epi*sqrt(p/b2);
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
obj.g_filter = make_filter_obj(10);
obj.kg = - Am(2,1) / b2;
obj.kg_rate = - Am(2,2)/ b2;
if not(obj.inited)
    obj.x(4) = 0;%p;
    obj.x(5) = 0;%pd/2;
    obj.actuator_estimator.x(5) = b2;
end
obj.u_filter = make_lag_obj(lag_fc,lag_alpha);
obj.km(1) = p;
obj.km(2) = pd;
obj.p_actuator = p_actuator;
obj.actuator_estimator.R = ekf_p_noise;
end
