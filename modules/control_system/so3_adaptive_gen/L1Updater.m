function dydt = L1Updater(t,x,Ad,b,P,Gamma,u,g,x_real)
%Predict Law
dydt = [0;0;0;0;0;0];
dydt(1:2,1) = Ad*x(1:2) + b*(x(3)*u+x(4:5)'*x(1:2)+x(6)) + g;
err = x(1:2) - x_real;
dydt(3,1) = - Gamma*err'*P*b*u;
dydt(4:5,1) = - Gamma*err'*P*b*x(1:2);
dydt(6,1) = - Gamma*err'*P*b;