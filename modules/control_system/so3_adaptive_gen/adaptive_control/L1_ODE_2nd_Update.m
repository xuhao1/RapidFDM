function dydt = L1_ODE_2nd_Update(t,x,obj)
b = obj.b;
u = obj.u;
Gamma = obj.Gamma;
P = obj.P;
err = obj.err;
%Predict Law
dydt = [0;0;0;0;0;0;0];
dydt(3,1) = - Gamma*err'*P*b*u; % omega
dydt(4:5,1) = - Gamma*err'*P*b*x(1:2); % theta
dydt(6,1) = - Gamma*err'*P*b; %sigma
dydt(7,1) = - Gamma*err'*P*[1;0]*10; % sigma1nd