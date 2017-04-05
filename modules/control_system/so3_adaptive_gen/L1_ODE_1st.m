function dydt = L1_ODE_1st(t,x,obj)
Ad = obj.Am;
b = obj.b;
x_real = obj.x_real;
u = obj.u;
Gamma = obj.Gamma;
P = obj.P;
b = obj.b;
%Predict Law
dydt = [0;0;0;0;0;0;0];
dydt(2,1) = Ad(2,1:2)*x(1:2) + b(2)*(x(3)*u+x(5)*x(2)+x(6)) + obj.g(2);

err = x(2) - x_real(2);
dydt(3,1) = - Gamma*err*P(2,2)*b(2)*u;
dydt(4:5,1) = - Gamma*err*P(2,2)*b(2)*x(2);
dydt(6,1) = - Gamma*err*P(2,2)*b(2);