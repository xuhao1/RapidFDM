function [obj,ys] = IterActuatorEst(obj,p,u,dt)
obj = EKFUpdate(obj,p,@Servohfunc);
for i =1:5
    q = obj.x(1);
    ys = obj.x(2);
    kd = obj.x(3);
    wc = obj.x(4);
    wp = obj.x(5);
    %sigma = obj.x(6);
    F = [1-kd*dt,dt*wp,-dt*q,0,dt*(ys);...
        0,1-dt*wc,0,dt*(u-ys),0;...
        0, 0, 1, 0 0;...
        0,0,0,1,0;...
        0 0 0 0 1 ];
    obj.F = F;
    obj = EKFPredict(obj,u,dt/5,@Servoffunc);
end
obj.x(3) = float_constrain(obj.x(3),0,200);
obj.z = p;
ys = obj.x(2);% + obj.x(6);
obj.actuator_real = obj.x(2);
end