function testL1ControlAttitude
sys_state = struct('quat',[1 0 0 0],'angular_rate',[0;0;0],'acc',[0;0;0]);
attcon = struct('RollCtrl',init_adaptive_controller(),'PitchCtrl',init_adaptive_controller(),...
    'quat_sp',[1 0 0 0],'quat',[1 0 0 0]);
attcon.RollCtrl = L1ControllerUpdateParams(6.0,1.0,30,10,1000);
L1ControlAttitude(attcon,0.005,[1 0 0 0],sys_state)
L1ControlAttitudeEuler(attcon,0.005,0,0,sys_state)
end