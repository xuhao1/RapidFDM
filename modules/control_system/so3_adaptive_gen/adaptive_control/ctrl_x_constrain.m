function x = ctrl_x_constrain(v)
x = v;
x(1) = float_constrain(x(1),-pi,pi);
x(2) = float_constrain(x(2),-10*pi,10*pi);
x(3) = float_constrain(x(3),0.01,10); % omega
x(4) = float_constrain(x(4),0.1,15); % theta0
x(5) = float_constrain(x(5),0.1,15); % theta1
x(6) = float_constrain(x(6),-0.3*x(3),0.3*x(3)); % sigma
x(7) = float_constrain(x(7),-0.1,0.1); %sigma 1nd
end