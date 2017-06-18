function [y,obj] = IterTransform(x,obj)
n = obj.n + 1;
n = 2;
if not( obj.inited)
    obj.inited = true;
    obj.x(1:n) = x;
    obj.y(1:n) = x;
    y = x;
    return
end
%'fuck'
for i = n:-1:2
    obj.x(i) = obj.x(i-1);
    obj.y(i) = obj.y(i-1);
end
obj.x(1) = x;
%Bz/Az
%obj.y(1) = obj.x(1) * obj.B(1) + obj.x(2) * obj.B(2) - obj.y(2) * obj.A(2);
y = 0;
for i = 1:n
    y = y + obj.x(i) * obj.B(i);
end
for i = 2:n
    y = y - obj.y(i) * obj.A(i);
end
obj.y(1) = y;
end