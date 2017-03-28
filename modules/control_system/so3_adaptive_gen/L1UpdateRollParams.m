function RollCtrl = L1UpdateRollParams(p,pd,b2,fc,gamma)
RollCtrl = init_adaptive_controller();

Am = [0 0;0 0];
Am(1,2) = 1.0;
Am(2,1) = - p *b2;
Am(2,2) = - pd *b2;

P = [0 0;0 0];
P(1,1) = ((Am(2,1) - 1) * Am(2,1) + Am(2,2) * Am(2,2)) / (2*Am(2,1)*Am(2,2));
P(2,1) = - (1/(2*Am(2,1)));
P(1,2) = - (1/(2*Am(2,1)));
P(2,2) = (1 - Am(2,1)) / (2*Am(2,1)*Am(2,2));



filter_obj = make_filter_obj(200,2*pi*7);
RollCtrl.Gamma = gamma;
RollCtrl.P = P;
RollCtrl.Am = Am;
RollCtrl.b = [0;b2];
RollCtrl.u_filter = filter_obj;
RollCtrl.kg = - Am(2,1) / b2;
RollCtrl.x(4) = p;
RollCtrl.x(5) = pd;
%RollCtrl.err = [0;0];
end
