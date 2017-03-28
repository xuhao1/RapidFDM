function x = ctrl_x_constrain(v)
x = v;
x(1) = float_constrain(x(1),-pi,pi);
x(2) = float_constrain(x(2),-10*pi,10*pi);
x(3) = float_constrain(x(3),0.1,100);
x(4) = float_constrain(x(4),0.1,3);
x(5) = float_constrain(x(5),0.1,3);
x(6) = float_constrain(x(6),-1,1);
end