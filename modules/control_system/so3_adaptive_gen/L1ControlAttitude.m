function [obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');

quat_sp_last = obj.quat_sp;
quat_last = obj.quat;

err_rov = quat_err_rov(quat_sp,sys_state.quat);

err_rov_delta_by_sp = quat_err_rov(quat_sp,quat_last) - quat_err_rov(quat_sp_last,quat_last);

x_real_roll = [err_rov(1);sys_state.angular_rate(1)];
x_real_pitch = [err_rov(2);sys_state.angular_rate(2)];

[obj.RollCtrl,u_roll] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,err_rov_delta_by_sp(1));
[obj.PitchCtrl,u_pitch] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,err_rov_delta_by_sp(2));

obj.quat_sp = quat_sp;
obj.quat = sys_state.quat;
end