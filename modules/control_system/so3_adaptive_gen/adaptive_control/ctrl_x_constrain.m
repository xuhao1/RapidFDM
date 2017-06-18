function x = ctrl_x_constrain(v,obj)
x = v;
x(1) = float_constrain(x(1),-pi,pi);
x(2) = float_constrain(x(2),-10*pi,10*pi);
x(3) = float_constrain(x(3),0.5,100); % omega
    
% x(4) = float_constrain(x(4),0.1,obj.km(1)*1.5*x(3)); % theta0
% x(5) = float_constrain(x(5),0.01,obj.km(2)*1.5*x(3)); % theta1

x(4) = float_constrain(x(4),-obj.km(1)*1.5*x(3),obj.km(1)*1.5*x(3)); % theta0
x(5) = float_constrain(x(5),-obj.km(2)*1.5*x(3),obj.km(2)*1.5*x(3)); % theta1

x(6) = float_constrain(x(6),-10,10); % sigma
x(7) = float_constrain(x(7),-0.3,0.3); %sigma 1nd
for i=1:7
    if isnan(x(i))
        if i == 3
            x(i) = 1;
        else
            x(i) = 0;
        end
    end
end