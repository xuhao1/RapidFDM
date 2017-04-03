function testL1ControlAttitude
sys_state = struct('quat',[1 0 0 0],'angular_rate',[0;0;0],'acc',[0;0;0]);
attcon = init_attitude_controller();
euler_control_sp = struct('pitch',0,'roll',0,'yaw',0,'yaw_sp_is_rate',true); 
quat_control_sp = struct('quat',[1 0 0 0],'yaw_rate',0,'yaw_sp_is_rate',true);
attcon.RollCtrl = L1ControllerUpdateParams(6.0,1.0,30,10,1000,attcon.RollCtrl);
attcon.PitchCtrl = L1ControllerUpdateParams(6.0,1.0,30,10,1000,attcon.RollCtrl);
L1ControlAttitude(attcon,0.005,quat_control_sp,sys_state)
L1ControlAttitudeEuler(attcon,0.005,euler_control_sp,sys_state)

end