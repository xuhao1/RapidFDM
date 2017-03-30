function obj = init_attitude_controller()
obj = struct('RollCtrl',init_adaptive_controller(),'PitchCtrl',init_adaptive_controller(),...
    'quat_sp',[1 0 0 0],'quat',[1 0 0 0],'drovsp_init',[0 0 0]);
coder.cstructname(obj,'AttitudeCtrlT')
end