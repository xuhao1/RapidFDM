function dydt = L1_ODE_2nd_PRE(t,x,obj)
Ad = obj.Am;
u = obj.u;
b = obj.b;
%Predict Law
dydt = [0;0;0;0;0;0;0];
dydt(1:2,1) = Ad*x(1:2) + b*(x(3)*u+x(4:5)'*x(1:2)+x(6)) + obj.g +[1;0]*x(7);