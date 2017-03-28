function test_step2nd_roll
filter_obj = struct('A',[0,0],'B',[0,0],'x',[0,0],'y',[0,0]);
s_ctrl = L1UpdateRollParams(2.0,2.0,32,5,100);
sys_state = struct('quat',[0.9330    0.2500    0.2500   -0.0670],'angular_rate',[1.0;1.2;0],'acc',[0;0;0]);
s_ctrl = L1Step2ndRoll(0.005,s_ctrl,sys_state,0);
s_ctrl.x;
sys_state = struct('quat',[0.9330    0.2500    0.2500   -0.0670],'angular_rate',[1.01;1.20;0],'acc',[0;0;0]);
s_ctrl = L1Step2ndRoll(0.005,s_ctrl,sys_state,0);
s_ctrl.x;
end