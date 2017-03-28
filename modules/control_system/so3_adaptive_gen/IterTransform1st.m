function [y,obj] = IterTransform1st(x,obj)
if not( obj.inited)
    obj.inited = true;
    obj.x(1) = x;
    obj.x(2) = x;
    obj.y(1) = x;
    obj.y(2) = x;
    y = x;
    return
end
obj.x(2) = obj.x(1);
obj.x(1) = x;
%Bz/Az
obj.y(2) = obj.y(1);
obj.y(1) = obj.x(1) * obj.B(1) + obj.x(2) * obj.B(2) - obj.y(2) * obj.A(2);

y = obj.y(1);
end