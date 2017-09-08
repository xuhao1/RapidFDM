function  obj  = InitActuatorEstimator()
% p ys kd wc wp sigma

obj.x = [0;0;3;2.7;10;0];
%obj.f = @Servoffunc;
%obj.h = @Servohfunc;
obj.F = zeros(6);
obj.H = [1 0 0 0 0 0];
%obj.Q = [0.1 0.2 2 0.1 3 6.0]'*[0.1 0.2 2 0.1 3 6.0]*0.005;
% q ys kd wc wp sigma
obj.Q = diag([1 0.2 0.5 0.01 2.0 20]);
%obj.Q = diag([0.1 0.1 0.01 0.01 2.0 1.0]);
obj.R = 0.1;
obj.P = diag([100 10 100 10 20 2]);
obj.yres = 0;
obj.t = 0;
obj.z = 0;
obj.actuator_real = 0;
coder.cstructname(obj,'ActuatorEstimatior');
end

