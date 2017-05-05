function obj = L1ControlAngularVelocity(obj,dt,ang_vel_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.YawCtrl, 'AdaptiveCtrlT');

obj.quat_sp = sys_state.quat;
obj.quat = sys_state.quat;

x_real_roll = [0;sys_state.angular_rate(1)];
x_real_pitch = [0;sys_state.angular_rate(2)];
x_real_yaw = [0;sys_state.angular_rate(3)];

[obj.RollCtrl,obj.u(1)] = L1AdaptiveControl1st(dt,obj.RollCtrl,x_real_roll,ang_vel_sp(1));
[obj.PitchCtrl,obj.u(2)] = L1AdaptiveControl1st(dt,obj.PitchCtrl,x_real_pitch,ang_vel_sp(2));
[obj.YawCtrl,obj.u(3)] = L1AdaptiveControl1st(dt,obj.YawCtrl,x_real_yaw,ang_vel_sp(3));

end