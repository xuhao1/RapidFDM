function obj = L1ControlAttitude(obj,dt,quat_control_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.YawCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');
coder.cstructname(quat_control_sp,'QuatControlSetpoint');

quat_sp = quat_control_sp.quat;
quat_sp_last = obj.quat_sp;
quat_last = obj.quat;

rov_invsp = quat_err_rov(quat_sp,sys_state.quat);

%the changed value of quat_sp because of rov
delta_err_sp = quat_err_rov(quat_sp,quat_last) - quat_err_rov(quat_sp_last,quat_last);
obj.quat_sp = quat_sp;
obj.quat = sys_state.quat;

x_real_roll = [rov_invsp(1);sys_state.angular_rate(1)];
x_real_pitch = [rov_invsp(2);sys_state.angular_rate(2)];
x_real_yaw = [rov_invsp(3);sys_state.angular_rate(3)];
[obj.PitchCtrl.g,obj.RollCtrl.g,obj.YawCtrl.g] = so3_external_dynamics(rov_invsp,sys_state.angular_rate);

obj.RollCtrl.g(1) =delta_err_sp(1)/dt + obj.RollCtrl.g(1);
obj.PitchCtrl.g(1) =  delta_err_sp(2)/dt + obj.PitchCtrl.g(1);
obj.YawCtrl.g(1) = delta_err_sp(3)/dt + obj.YawCtrl.g(1);

[obj.RollCtrl,obj.u(1)] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,0);
[obj.PitchCtrl,obj.u(2)] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,0);

if quat_control_sp.yaw_sp_is_rate
    [obj.YawCtrl,obj.u(3)] = L1AdaptiveControl1st(dt,obj.YawCtrl,x_real_yaw,quat_control_sp.yaw_rate);
else
    [obj.YawCtrl,obj.u(3)] = L1AdaptiveControl2nd(dt,obj.YawCtrl,x_real_yaw,0);
end

end