function [obj,ys] = IterActuatorEst(obj,p,u,dt)
for i =1:5
    q = obj.x(1);
    ys = obj.x(2);
    kd = obj.x(3);
    wc = obj.x(4);
    wp = obj.x(5);
    sigma = obj.x(6);
    dtp = dt / 5;
    F = [1-kd*dtp,dtp*wp*wp,-dtp*q,0,2*dtp*wp*(ys),dtp;...
        0,1-dtp*wc,0,dtp*(u-ys),0,0;...
        0, 0, 1, 0 0 0;...
        0,0,0,1,0,0;...
        0 0 0 0 1,0;...
        0 0 0 0 0 1
        ];
%    F = [1-kd*dtp,dtp*wp,-dtp*q,0,dtp*(ys+sigma),dtp*wp;...
%         0,1-dtp*wc,0,dtp*(u-ys),0,0;...
%         0, 0, 1, 0 0 0;...
%         0,0,0,1,0,0;...
%         0 0 0 0 1,0;...
%         0 0 0 0 0 1
%         ];
    obj.F = F;
    obj = EKFPredict(obj,u,dtp,@Servoffunc);
end

obj = EKFUpdate(obj,p,@Servohfunc);
obj.x(3) = float_constrain(obj.x(3),0,100);
obj.x(4) = float_constrain(obj.x(4),0,50);
obj.x(5) = float_constrain(obj.x(5),0,12);
obj.x(6) = float_constrain(obj.x(6),-10,10);
ys = obj.x(2);
if obj.P(2,2) < 0.05
    obj.actuator_real = obj.x(2);
else
    obj.actuator_real = u;
end
end