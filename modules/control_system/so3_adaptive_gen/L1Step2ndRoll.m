function res = L1Step2ndRoll(dt,RollCtrl,SysState,r)
coder.cstructname(RollCtrl, 'AdaptiveCtrlT');
coder.cstructname(SysState, 'AdaptiveSysT');

res = RollCtrl;
res.r = r;
%coder.cstructname(res, 'AdaptiveCtrlT');

rov = quat2rov(SysState.quat);

if not(RollCtrl.inited)
    %res = init_adaptive_controller();
    res.x(1) = rov(1);
    res.x(2) = rov(2);
    res.inited = true;
return
end

x0 = res.x;
Ad = res.Am;

b = res.b;
P = res.P;

Gamma = res.Gamma;
u = res.u;
g = [0;0];

x_real = [rov(1);SysState.angular_rate(1)];
res.err = x_real(1:2) - res.x(1:2);

opts_1 = odeset('MaxStep',4);
[~,x]=ode23(@(t,x) L1Updater(t,x,Ad,b,P,Gamma,u,g,x_real),...
    [RollCtrl.t,RollCtrl.t + dt],x0,opts_1);

res = L1ControlLaw2nd(dt,res,SysState,r);


res.x = ctrl_x_constrain(x(end,1:6)');

res.t = RollCtrl.t + dt;