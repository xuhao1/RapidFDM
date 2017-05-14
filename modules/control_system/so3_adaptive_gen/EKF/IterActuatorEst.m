function [obj,ys] = IterActuatorEst(obj,u,dt)
for i =1:5
    q = obj.x(1);
    ys = obj.x(2);
    kd = obj.x(3);
    wc = obj.x(4);
    wp = obj.x(5);
    sigma = obj.x(6);
    dtp = dt / 5;
    F = [1-kd*dtp,dtp*wp,-dtp*q,0,dtp*(ys+sigma),dtp*wp;...
        0,1-dtp*wc,0,dtp*(u-ys),0,0;...
        0, 0, 1, 0 0 0;...
        0,0,0,1,0,0;...
        0 0 0 0 1,0;...
        0 0 0 0 0 1
        ];
    obj.F = F;
    obj = EKFPredict(obj,u,dt/5,@Servoffunc);
end
obj.x(3) = float_constrain(obj.x(3),0,200);
ys = obj.x(2);
obj.actuator_real = obj.x(2);
end