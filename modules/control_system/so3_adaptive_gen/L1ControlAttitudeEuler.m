function [obj,u_roll,u_pitch] = L1ControlAttitudeEuler(obj,dt,euler_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');
coder.cstructname(euler_sp,'EulerControlSetPoint');
[phi,theta,psi] = get_sys_euler(sys_state);
%----Control by pure euler----
yawrate = get_yaw_rate(sys_state);
q = sys_state.angular_rate(2);
r = sys_state.angular_rate(3);

% euler nonlinear fix
pitch_feedforward = q*(cos(phi)-1) - r*sin(phi);
roll_feedforward = tan(theta)*(q*sin(phi)+r*cos(phi));

x_real_roll = [phi;sys_state.angular_rate(1)];
x_real_pitch = [theta;sys_state.angular_rate(2)];
obj.RollCtrl.g(1) = roll_feedforward;
obj.PitchCtrl.g(1) = pitch_feedforward;
[obj.RollCtrl,u_roll] = L1AdaptiveControl2nd(dt,obj.RollCtrl,x_real_roll,euler_sp.roll);
[obj.PitchCtrl,u_pitch] = L1AdaptiveControl2nd(dt,obj.PitchCtrl,x_real_pitch,euler_sp.pitch);

%obj.quat_sp = quat_sp;
%obj.quat = sys_state.quat;
end