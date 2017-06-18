function testL1ControlAttitude
feature('SetPrecision', 24)
sys_state = struct('quat',[1 0 0 0],'angular_rate',[0;0;0],'angular_rate_dot',[0;0;0],'acc',[0;0;0]);
attcon = init_attitude_controller(false,2);
euler_control_sp = struct('pitch',0,'roll',0,'yaw',0,'yaw_sp_is_rate',true); 
quat_control_sp = struct('quat',[1 0 0 0],'yaw_rate',0,'yaw_sp_is_rate',true);
attcon.RollCtrl = L1ControllerUpdateParams(attcon.RollCtrl,6.0,1.0,30,1000,3,2.5,3);
attcon.PitchCtrl = L1ControllerUpdateParams(attcon.PitchCtrl,6.0,1.0,30,1000,3,2.5,3);
L1ControlAngularVelocity(attcon,0.005,[0,0,0],sys_state)
L1ControlAttitude(attcon,0.005,quat_control_sp,sys_state)
L1ControlAttitude(attcon,0.005,quat_control_sp,sys_state)
L1ControlAttitude(attcon,0.005,quat_control_sp,sys_state)
IterationServo(attcon,0.005,sys_state,[0,0,0])
L1_ODE_2nd(0,[0; 0; 0; 0; 0; 0; 0],attcon.RollCtrl);
end