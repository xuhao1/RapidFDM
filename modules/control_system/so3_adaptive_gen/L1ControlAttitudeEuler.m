function [obj,u_roll,u_pitch] = L1ControlAttitudeEuler(obj,dt,pitch_sp,roll_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');

eul = quat2eul(sys_state.quat);
quat_sp = eul2quat([eul(1) pitch_sp roll_sp]);

[obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_sp,sys_state);
return
%----Control by pure euler----

x_real_roll = [eul(3);sys_state.angular_rate(1)];
x_real_pitch = [eul(2);sys_state.angular_rate(2)];
err_rov_delta_by_sp = [0 0 0];
rfb = 0;
[obj.RollCtrl,u_roll] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,err_rov_delta_by_sp(1),roll_sp,0);
[obj.PitchCtrl,u_pitch] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,err_rov_delta_by_sp(2),pitch_sp,rfb);

%obj.quat_sp = quat_sp;
%obj.quat = sys_state.quat;
end