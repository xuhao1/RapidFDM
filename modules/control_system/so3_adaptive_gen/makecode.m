function makecode
global s_ctrl sys_state filter_obj
%x:[psi psidot omega theta0 theta1 sigma]
s_ctrl = init_adaptive_controller();
filter_obj = s_ctrl.u_filter;
sys_state = struct('quat',[1 0 0 0],'angular_rate',[0;0;0],'acc',[0;0;0]);