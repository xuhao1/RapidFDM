function obj = EKFPredict(obj,u,dt,f)
    obj.x = f(obj.x,u,dt);
    obj.P = obj.F*obj.P*obj.F' + obj.Q*dt;
    obj.t = obj.t + dt;
end