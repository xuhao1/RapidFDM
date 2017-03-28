function [obj,u_roll,u_pitch] = L1ControlAttitudeEuler(obj,dt,pitch_sp,roll_sp,sys_state)
quat_sp = eul2quat([0 pitch_sp roll_sp]);
[obj,u_roll,u_pitch] = L1ControlAttitude(obj,dt,quat_sp,sys_state);
end