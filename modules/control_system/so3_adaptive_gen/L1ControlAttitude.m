function [obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');

yawrate = get_yaw_rate(sys_state);

[phi,theta,~] = get_sys_euler(sys_state);

pitch_feedforward = - yawrate * cos(theta)*sin(phi);

quat_sp_last = obj.quat_sp;
quat_last = obj.quat;

%inv(quat_sp)*quat 以sp为原点的rov
rov_invsp = quat_err_rov(quat_sp,sys_state.quat);

%因为quat_sp改变量而产生的rov变化
delta_err_sp = quat_err_rov(quat_sp,quat_last) - quat_err_rov(quat_sp_last,quat_last);
obj.drovsp_init = obj.drovsp_init + delta_err_sp;

x_real_roll = [rov_invsp(1) - obj.drovsp_init(1);sys_state.angular_rate(1)];
x_real_pitch = [rov_invsp(2) - obj.drovsp_init(2);sys_state.angular_rate(2)];

%[obj.PitchCtrl.g,obj.PitchCtrl.g_by_x] = pitch_external_dynamics(rov_invsp,sys_state.angular_rate);
%[obj.RollCtrl.g,~] = roll_external_dynamics(rov_invsp,sys_state.angular_rate);

%rfb = 0;
[obj.RollCtrl,u_roll] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,...
    -obj.drovsp_init(1),0);
[obj.PitchCtrl,u_pitch] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,...
    -obj.drovsp_init(2),pitch_feedforward);

obj.quat_sp = quat_sp;
obj.quat = sys_state.quat;
end