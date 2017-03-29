function dydt = L1Updater(t,x,obj)
Ad = obj.Am;
b = obj.b;
x_real = obj.x_real;
u = obj.u;
Gamma = obj.Gamma;
P = obj.P;
b = obj.b;
%Predict Law
dydt = [0;0;0;0;0;0];
%dydt(1:2,1) = Ad*x(1:2) + b*(x(3)*u+x(4:5)'*x(1:2)+x(6)) + g;
dydt(1:2,1) = Ad*x(1:2) + b*(x(3)*u+x(4:5)'*x(1:2)+x(6));
err = x(1:2) - x_real;
dydt(3,1) = - Gamma*err'*P*b*u;
dydt(4:5,1) = - Gamma*err'*P*b*x(1:2);
dydt(6,1) = - Gamma*err'*P*b;