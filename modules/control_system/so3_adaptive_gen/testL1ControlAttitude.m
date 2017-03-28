function testL1ControlAttitude
sys_state = struct('quat',[1 0 0 0],'angular_rate',[0;0;0],'acc',[0;0;0]);
attcon = struct('RollCtrl',init_adaptive_controller(),'PitchCtrl',init_adaptive_controller());
L1ControlAttitude(attcon,0.005,[1 0 0 0],sys_state)
L1ControlAttitudeEuler(attcon,0.005,0,0,sys_state)
end