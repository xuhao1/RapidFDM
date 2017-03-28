function [obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');

quat_sp_no_yaw = get_quat_no_yaw(quat_sp);
rov_sp_no_yaw = quat2rov(quat_sp_no_yaw);
rov_no_yaw = quat2rov(get_quat_no_yaw(sys_state.quat));

x_real_roll = [rov_no_yaw(1);sys_state.angular_rate(1)];
x_real_pitch = [rov_no_yaw(2);sys_state.angular_rate(2)];

[obj.RollCtrl,u_roll] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,rov_sp_no_yaw(1));
[obj.PitchCtrl,u_pitch] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,rov_sp_no_yaw(2));
end