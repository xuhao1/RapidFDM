function dydt = L1_ODE_2nd(t,x,obj)
Ad = obj.Am;
b = obj.b;
x_real = obj.x_real;
u = obj.actuator_estimator.actuator_real;
Gamma = obj.Gamma;
P = obj.P;
b = obj.b;
%Predict Law
dydt = [0;0;0;0;0;0;0];
dydt(1:2,1) = Ad*x(1:2) + b*(x(3)*u+x(4:5)'*x_real+x(6)) + obj.g +[1;0]*x(7);
err = x(1:2) - x_real;
dydt(3,1) = - Gamma*err'*P*b*u; % omega
dydt(4:5,1) = - Gamma*err'*P*b*x_real; % theta
dydt(6,1) = - Gamma*err'*P*b; %sigma
dydt(7,1) = - Gamma*err'*P*[1;0]*100; % sigma1nd