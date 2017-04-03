function [obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_control_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');
coder.cstructname(quat_control_sp,'QuatControlSetpoint');

quat_sp = quat_control_sp.quat;
quat_sp_last = obj.quat_sp;
quat_last = obj.quat;

%inv(quat_sp)*quat the rov with using sp as origin 
rov_invsp = quat_err_rov(quat_sp,sys_state.quat);

%the changed value of quat_sp because of rov
delta_err_sp = quat_err_rov(quat_sp,quat_last) - quat_err_rov(quat_sp_last,quat_last);
obj.drovsp_init = obj.drovsp_init + delta_err_sp;

x_real_roll = [rov_invsp(1);sys_state.angular_rate(1)];
x_real_pitch = [rov_invsp(2);sys_state.angular_rate(2)];

[obj.PitchCtrl.g,~] = pitch_external_dynamics(rov_invsp,sys_state.angular_rate);
[obj.RollCtrl.g,~] = roll_external_dynamics(rov_invsp,sys_state.angular_rate);

obj.RollCtrl.g(1) = obj.RollCtrl.g(1)+ delta_err_sp(1)/dt;
obj.PitchCtrl.g(1) = obj.PitchCtrl.g(1)+ delta_err_sp(2)/dt;

[obj.RollCtrl.g(1),obj.RollCtrl.g_filter] = IterTransform1st(obj.RollCtrl.g(1),obj.RollCtrl.g_filter);
[obj.PitchCtrl.g(1),obj.PitchCtrl.g_filter] = IterTransform1st(obj.PitchCtrl.g(1),obj.PitchCtrl.g_filter);


[obj.RollCtrl,u_roll] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,...
    0);
[obj.PitchCtrl,u_pitch] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,...
    0);

obj.quat_sp = quat_sp;
obj.quat = sys_state.quat;
end