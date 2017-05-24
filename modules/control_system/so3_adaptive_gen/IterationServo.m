function obj = IterationServo(obj,dt,sys_state,u)
obj.u = u;
[obj.RollCtrl.actuator_estimator,~] = IterActuatorEst(obj.RollCtrl.actuator_estimator,sys_state.angular_rate(1),u(1),dt);
[obj.PitchCtrl.actuator_estimator,~] = IterActuatorEst(obj.RollCtrl.actuator_estimator,sys_state.angular_rate(2),u(2),dt);
[obj.YawCtrl.actuator_estimator,~] = IterActuatorEst(obj.RollCtrl.actuator_estimator,sys_state.angular_rate(3),u(3),dt);
end