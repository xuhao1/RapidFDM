function obj = init_attitude_controller(fixed_step_ode,ode_steps)
obj.RollCtrl = init_adaptive_controller(fixed_step_ode,ode_steps);
obj.PitchCtrl = init_adaptive_controller(fixed_step_ode,ode_steps);
obj.YawCtrl = init_adaptive_controller(fixed_step_ode,ode_steps);
obj.quat_sp = [1 0 0 0];
obj.quat = [1 0 0 0];
obj.u = [0 0 0];
obj.debug_time_used = 0;
coder.cstructname(obj,'AttitudeCtrlT')
end