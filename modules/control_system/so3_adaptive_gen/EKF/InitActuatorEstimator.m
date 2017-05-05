function  obj  = InitActuatorEstimator()
% p ys kd wc wp
%coder.structname(obj,'ActuatorDynamicsEstimatior')
obj.x = [0;0;0;15;30];
%obj.f = @Servoffunc;
%obj.h = @Servohfunc;
obj.F = zeros(5);
obj.H = [1 0 0 0 0];
%obj.Q = [0.1 0.2 2 4 3 1.0]'*[0.1 0.2 2 4 3 1.0]*0.005;
obj.Q = diag([1 0.2 20 5 10]);
obj.R = 0.1;
obj.P = diag([100 10 100 10 20]);
obj.yres = 0;
obj.t = 0;
obj.z = 0;
obj.actuator_real = 0;
%coder.cstructname(obj,'ActuatorDynamicsEstimatior')

end

