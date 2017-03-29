function [obj,u_roll,u_pitch] = L1ControlAttitudeEuler(obj,dt,pitch_sp,roll_sp,sys_state)
coder.cstructname(obj,'AttitudeCtrlT')
coder.cstructname(obj.RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(obj.PitchCtrl, 'AdaptiveCtrlT');
coder.cstructname(sys_state,'AdaptiveSysT');

eul = quat2eul(sys_state.quat);
quat_sp = eul2quat([eul(1) pitch_sp roll_sp]);
[obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_sp,sys_state);
end