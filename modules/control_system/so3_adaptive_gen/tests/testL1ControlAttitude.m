function testL1ControlAttitude
sys_state = struct('quat',[1 0 0 0],'angular_rate',[0;0;0],'angular_rate_dot',[0;0;0],'acc',[0;0;0]);
attcon = init_attitude_controller();
euler_control_sp = struct('pitch',0,'roll',0,'yaw',0,'yaw_sp_is_rate',true); 
quat_control_sp = struct('quat',[1 0 0 0],'yaw_rate',0,'yaw_sp_is_rate',true);
attcon.RollCtrl = L1ControllerUpdateParams(attcon.RollCtrl,6.0,1.0,30,1000,3,2.5,3,3);
attcon.PitchCtrl = L1ControllerUpdateParams(attcon.PitchCtrl,6.0,1.0,30,1000,3,2.5,3,3);
L1ControlAttitude(attcon,0.005,quat_control_sp,sys_state)
L1ControlAttitudeEuler(attcon,0.005,euler_control_sp,sys_state)

obj = init_angular_control_2nd(573,17,1000,7,15);
angular_velocity_control_2nd(obj,0.005,sys_state,0);
end