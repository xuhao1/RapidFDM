function [obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_control_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');
coder.cstructname(quat_control_sp,'QuatControlSetpoint');
%--using Euler for now
% eul_sp = quat2eul(quat_control_sp.quat);
% if quat_control_sp.yaw_sp_is_rate
%     euler_sp = struct('pitch',eul_sp(2),'roll',eul_sp(3),'yaw',quat_control_sp.yaw_rate,'yaw_sp_is_rate',true);
% else
%     euler_sp = struct('pitch',eul_sp(2),'roll',eul_sp(3),'yaw',eul_sp(1),'yaw_sp_is_rate',true);
% end
% 
% [obj,u_roll,u_pitch] = L1ControlAttitudeEuler(obj,dt,euler_sp,sys_state);
% return;

yawrate = get_yaw_rate(sys_state);


quat_sp_last = obj.quat_sp;
quat_last = obj.quat;

%inv(quat_sp)*quat the rov with using sp as origin 
rov_invsp = quat_err_rov(quat_sp,sys_state.quat);

%the changed value of quat_sp because of rov
delta_err_sp = quat_err_rov(quat_sp,quat_last) - quat_err_rov(quat_sp_last,quat_last);
obj.drovsp_init = obj.drovsp_init + delta_err_sp;

x_real_roll = [rov_invsp(1) - obj.drovsp_init(1);sys_state.angular_rate(1)];
x_real_pitch = [rov_invsp(2) - obj.drovsp_init(2);sys_state.angular_rate(2)];

[obj.PitchCtrl.g,~] = pitch_external_dynamics(rov_invsp,sys_state.angular_rate);
[obj.RollCtrl.g,~] = roll_external_dynamics(rov_invsp,sys_state.angular_rate);

[obj.RollCtrl,u_roll] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,...
    -obj.drovsp_init(1));
[obj.PitchCtrl,u_pitch] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,...
    -obj.drovsp_init(2));

obj.quat_sp = quat_sp;
obj.quat = sys_state.quat;
end